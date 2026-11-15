//! gromecu: read and flash Honda Grom / Monkey ECUs over a K-line cable.

const std = @import("std");
const builtin = @import("builtin");
const Io = std.Io;

const ecu_mod = @import("ecu.zig");
const Ecu = ecu_mod.Ecu;
const emu = @import("emu.zig");
const models = @import("models.zig");
const protocol = @import("protocol.zig");
const serial = @import("serial.zig");
const sim_mod = @import("sim.zig");
const Transport = @import("transport.zig").Transport;

pub const version = "0.1.0";

const usage =
    \\gromecu - read and flash Honda Grom / Monkey ECUs over a K-line cable
    \\
    \\Usage: gromecu [options] <command> [arguments]
    \\
    \\Commands:
    \\  ports                 list serial ports that could be the cable
    \\  info                  show ECU state, ECM ID, model and flash count
    \\  dtc [--clear]         show (or clear) trouble codes
    \\  tables                dump the ECU's data tables
    \\  live [--count N]      show live sensor data
    \\  read <out.bin>        read the ECU flash into a file
    \\  write <in.bin>        erase the ECU and flash a file
    \\  check <file.bin>      inspect an image file (size, checksum, model)
    \\  models                list supported ECUs
    \\  emu                   run a simulated ECU on a pty (Linux, for testing)
    \\
    \\Options:
    \\  -p, --port <port>     serial port (/dev/ttyUSB0, COM3, ...); default:
    \\                        $GROMECU_PORT, else the only USB serial port found.
    \\                        "sim:<model>" uses a built-in simulated ECU.
    \\      --baud <n>        baud rate (default 10400)
    \\      --timeout <ms>    how long to wait for an answer (default 200)
    \\      --retries <n>     extra attempts per command (default 3)
    \\      --no-echo         the adapter does not echo what it sends
    \\      --trace           print every frame sent and received
    \\  -y, --yes             do not ask for confirmation before flashing
    \\      --force           override the safety checks named in the error
    \\  -h, --help            show this help
    \\      --version         show the version
    \\
    \\read:
    \\      --start <addr>    first address to read (default 0)
    \\      --length <n>      bytes to read (default: until the ECU refuses)
    \\      --password <pw>   14 characters or 28 hex digits
    \\                        (default "HelloHowAreYou")
    \\      --manual          press Enter instead of having the tool sense the
    \\                        ignition (implied by --no-echo)
    \\write:
    \\      --offset <addr>   flash address to write at (default: from the model)
    \\      --model <name>    treat the image as being for this ECU (see `models`)
    \\      --fix-checksum <addr>
    \\                        recompute the checksum byte at this address
    \\check:
    \\      --fix-checksum <addr> -o <out.bin>
    \\                        write a copy with the checksum byte corrected
    \\emu:
    \\      --model <name>    ECU to simulate (default K26-911)
    \\      --image <file>    flash contents (default: a synthetic image)
    \\      --quiet           do not log frames
    \\
    \\Numbers may be decimal or 0x-prefixed hexadecimal.
    \\
;

const Args = struct {
    command: ?[]const u8 = null,
    positional: [2]?[]const u8 = .{ null, null },
    port: ?[]const u8 = null,
    baud: u32 = serial.default_baud,
    timeout_ms: u32 = 200,
    retries: u8 = 3,
    manual: bool = false,
    no_echo: bool = false,
    trace: bool = false,
    yes: bool = false,
    force: bool = false,
    help: bool = false,
    show_version: bool = false,
    clear: bool = false,
    quiet: bool = false,
    count: ?u32 = null,
    start: u32 = 0,
    length: ?u32 = null,
    password: [14]u8 = ecu_mod.default_password,
    offset: ?u32 = null,
    model: ?[]const u8 = null,
    fix_checksum: ?u32 = null,
    output: ?[]const u8 = null,
    image: ?[]const u8 = null,
};

const Fatal = error{Fatal};

const App = struct {
    io: Io,
    gpa: std.mem.Allocator,
    out: *Io.Writer,
    /// One reader for the whole run, so buffered input is not lost between
    /// prompts.
    stdin: *Io.Reader,
    args: Args,
    env_port: ?[]const u8,
    progress_open: bool = false,
    /// Progress lines are only drawn on a terminal.
    stderr_is_tty: bool = false,
    /// `read` saves data as it arrives, so an interrupted dump is not lost.
    dump: ?struct { file: Io.File, data: *std.ArrayList(u8), written: usize = 0, failed: bool = false } = null,

    fn saveDump(app: *App) void {
        const d = &(app.dump orelse return);
        if (d.failed or d.written >= d.data.items.len) return;
        d.file.writeStreamingAll(app.io, d.data.items[d.written..]) catch {
            d.failed = true;
            return;
        };
        d.written = d.data.items.len;
    }

    fn print(app: *App, comptime fmt: []const u8, a: anytype) void {
        app.out.print(fmt, a) catch {};
    }

    fn flush(app: *App) void {
        app.out.flush() catch {};
    }

    /// Report an error and unwind to main.
    fn fail(app: *App, comptime fmt: []const u8, a: anytype) Fatal {
        app.flush();
        app.endProgress();
        std.debug.print("error: " ++ fmt ++ "\n", a);
        return error.Fatal;
    }

    fn warn(app: *App, comptime fmt: []const u8, a: anytype) void {
        app.flush();
        app.endProgress();
        std.debug.print("warning: " ++ fmt ++ "\n", a);
    }

    /// A status line on stderr that overwrites itself.
    fn progress(app: *App, comptime fmt: []const u8, a: anytype) void {
        if (!app.stderr_is_tty) return;
        app.flush();
        std.debug.print("\r\x1b[K" ++ fmt, a);
        app.progress_open = true;
    }

    fn endProgress(app: *App) void {
        if (app.progress_open) std.debug.print("\n", .{});
        app.progress_open = false;
    }
};

// ---------------------------------------------------------------------------
// Argument parsing
// ---------------------------------------------------------------------------

fn parseNumber(s: []const u8) ?u32 {
    return std.fmt.parseInt(u32, s, 0) catch null;
}

fn parsePassword(s: []const u8) ?[14]u8 {
    var pw: [14]u8 = undefined;
    if (s.len == 14) {
        @memcpy(&pw, s);
        return pw;
    }
    if (s.len == 28) {
        _ = std.fmt.hexToBytes(&pw, s) catch return null;
        return pw;
    }
    return null;
}

fn parseArgs(argv: []const [:0]const u8) error{BadUsage}!Args {
    var a: Args = .{};
    var i: usize = 1;
    var n_pos: usize = 0;
    while (i < argv.len) : (i += 1) {
        const arg: []const u8 = argv[i];
        const eql = std.mem.eql;
        if (arg.len < 2 or arg[0] != '-') {
            if (a.command == null) {
                a.command = arg;
            } else if (n_pos < a.positional.len) {
                a.positional[n_pos] = arg;
                n_pos += 1;
            } else {
                std.debug.print("error: unexpected argument '{s}'\n", .{arg});
                return error.BadUsage;
            }
            continue;
        }

        // Flags without a value.
        if (eql(u8, arg, "-h") or eql(u8, arg, "--help")) {
            a.help = true;
        } else if (eql(u8, arg, "--version")) {
            a.show_version = true;
        } else if (eql(u8, arg, "--no-echo")) {
            a.no_echo = true;
        } else if (eql(u8, arg, "--trace")) {
            a.trace = true;
        } else if (eql(u8, arg, "-y") or eql(u8, arg, "--yes")) {
            a.yes = true;
        } else if (eql(u8, arg, "--force")) {
            a.force = true;
        } else if (eql(u8, arg, "--clear")) {
            a.clear = true;
        } else if (eql(u8, arg, "--quiet")) {
            a.quiet = true;
        } else if (eql(u8, arg, "--manual")) {
            a.manual = true;
        } else {
            // Flags with a value.
            if (i + 1 >= argv.len) {
                std.debug.print("error: '{s}' is unknown or needs a value\n", .{arg});
                return error.BadUsage;
            }
            i += 1;
            const value: []const u8 = argv[i];
            var ok = true;
            if (eql(u8, arg, "-p") or eql(u8, arg, "--port")) {
                a.port = value;
            } else if (eql(u8, arg, "-o") or eql(u8, arg, "--output")) {
                a.output = value;
            } else if (eql(u8, arg, "--model")) {
                a.model = value;
            } else if (eql(u8, arg, "--image")) {
                a.image = value;
            } else if (eql(u8, arg, "--baud")) {
                a.baud = parseNumber(value) orelse 0;
                ok = a.baud != 0;
            } else if (eql(u8, arg, "--timeout")) {
                a.timeout_ms = parseNumber(value) orelse 0;
                ok = a.timeout_ms != 0;
            } else if (eql(u8, arg, "--retries")) {
                if (parseNumber(value)) |v| {
                    if (v <= 20) a.retries = @intCast(v) else ok = false;
                } else ok = false;
            } else if (eql(u8, arg, "--count")) {
                a.count = parseNumber(value);
                ok = a.count != null;
            } else if (eql(u8, arg, "--start")) {
                if (parseNumber(value)) |v| a.start = v else ok = false;
            } else if (eql(u8, arg, "--length")) {
                a.length = parseNumber(value);
                ok = a.length != null and a.length.? != 0;
            } else if (eql(u8, arg, "--offset")) {
                a.offset = parseNumber(value);
                ok = a.offset != null;
            } else if (eql(u8, arg, "--fix-checksum")) {
                a.fix_checksum = parseNumber(value);
                ok = a.fix_checksum != null;
            } else if (eql(u8, arg, "--password")) {
                if (parsePassword(value)) |pw| a.password = pw else ok = false;
            } else {
                std.debug.print("error: unknown option '{s}'\n", .{arg});
                return error.BadUsage;
            }
            if (!ok) {
                std.debug.print("error: bad value '{s}' for {s}\n", .{ value, arg });
                return error.BadUsage;
            }
        }
    }
    return a;
}

// ---------------------------------------------------------------------------
// Connecting
// ---------------------------------------------------------------------------

const Link = struct {
    kind: union(enum) {
        serial: serial.Serial,
        sim: struct {
            sim: sim_mod.Sim,
            st: sim_mod.SimTransport,
            flash: []u8,
        },
    },
    ecu: Ecu,

    fn close(link: *Link, gpa: std.mem.Allocator) void {
        switch (link.kind) {
            .serial => |*s| s.close(),
            .sim => |*s| gpa.free(s.flash),
        }
        gpa.destroy(link);
    }
};

fn connect(app: *App) (Fatal || std.mem.Allocator.Error)!*Link {
    const port = try choosePort(app);
    const link = try app.gpa.create(Link);
    errdefer app.gpa.destroy(link);

    if (std.mem.startsWith(u8, port, "sim:")) {
        const model = models.byName(port[4..]) orelse
            return app.fail("unknown model '{s}' (see `gromecu models`)", .{port[4..]});
        const flash = try app.gpa.alloc(u8, model.rom_size);
        sim_mod.fillSyntheticImage(model, flash, 1);
        link.kind = .{ .sim = .{ .sim = .init(model, flash), .st = undefined, .flash = flash } };
        // Nobody is there to turn the key, so the simulator does it itself.
        link.kind.sim.st = .{ .sim = &link.kind.sim.sim, .cycle_power_after_probes = 8 };
        link.ecu = .{ .t = link.kind.sim.st.asTransport() };
        app.warn("using the built-in simulated {s} ECU, not a real one", .{model.short()});
    } else {
        const s = serial.Serial.open(app.io, port, app.args.baud) catch |err| return switch (err) {
            error.PortNotFound => app.fail("cannot open {s}: no such port", .{port}),
            error.PortAccessDenied => app.fail(
                "cannot open {s}: permission denied\n" ++
                    "       on Linux, add yourself to the group that owns the port\n" ++
                    "       (usually `dialout` or `uucp`) and log in again",
                .{port},
            ),
            error.PortBusy => app.fail("cannot open {s}: in use by another program", .{port}),
            error.PortConfigFailed => app.fail(
                "cannot configure {s} for {d} baud, 8N1 (does the adapter support this rate?)",
                .{ port, app.args.baud },
            ),
            error.NameTooLong => app.fail("port name too long", .{}),
        };
        link.kind = .{ .serial = s };
        link.ecu = .{ .t = link.kind.serial.asTransport() };
    }
    link.ecu.timeout_ms = app.args.timeout_ms;
    link.ecu.retries = app.args.retries;
    link.ecu.expect_echo = !app.args.no_echo;
    link.ecu.trace = app.args.trace;
    return link;
}

fn choosePort(app: *App) Fatal![]const u8 {
    if (app.args.port) |p| return p;
    if (app.env_port) |p| return p;

    var found: [8][64]u8 = undefined;
    const n = listPorts(app.io, &found, null);
    if (n == 1) return app.gpa.dupe(u8, std.mem.sliceTo(&found[0], 0)) catch return error.Fatal;
    if (n == 0) return app.fail("no USB serial port found; plug in the cable or use --port", .{});
    return app.fail("{d} USB serial ports found; choose one with --port (see `gromecu ports`)", .{n});
}

/// Fills `found` with NUL-terminated port paths and returns how many there
/// are. With `app` given, also prints them.
fn listPorts(io: Io, found: [][64]u8, app: ?*App) usize {
    const Source = struct { dir: []const u8, prefix: []const u8 };
    const sources: []const Source = switch (builtin.os.tag) {
        .linux => &.{
            .{ .dir = "/dev", .prefix = "ttyUSB" },
            .{ .dir = "/dev", .prefix = "ttyACM" },
        },
        .windows => &.{},
        else => &.{.{ .dir = "/dev", .prefix = "cu.usb" }},
    };
    var n: usize = 0;
    for (sources) |src| {
        var dir = Io.Dir.openDirAbsolute(io, src.dir, .{ .iterate = true }) catch continue;
        defer dir.close(io);
        var it = dir.iterate();
        while (it.next(io) catch null) |entry| {
            if (!std.mem.startsWith(u8, entry.name, src.prefix)) continue;
            if (n < found.len) {
                _ = std.fmt.bufPrint(&found[n], "{s}/{s}\x00", .{ src.dir, entry.name }) catch continue;
                if (app) |a| a.print("{s}\n", .{std.mem.sliceTo(&found[n], 0)});
            }
            n += 1;
        }
    }
    return n;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

fn hexId(buf: *[14]u8, id: []const u8) []const u8 {
    return std.fmt.bufPrint(buf, "{x:0>2} {x:0>2} {x:0>2} {x:0>2} {x:0>2}", .{ id[0], id[1], id[2], id[3], id[4] }) catch unreachable;
}

fn printModel(app: *App, m: *const models.Model) void {
    app.print("Model:       {s} {s}, {s}", .{ m.name, m.years, m.part_number });
    if (m.confidence == .partial) app.print(" (flash layout not fully confirmed)", .{});
    app.print("\n", .{});
}

fn cmdPorts(app: *App) Fatal!void {
    if (builtin.os.tag == .windows) {
        app.print("Port listing is not implemented on Windows; look up the COM port\nnumber in Device Manager and pass it with --port COMn.\n", .{});
        return;
    }
    var found: [8][64]u8 = undefined;
    if (listPorts(app.io, &found, app) == 0) app.print("No USB serial ports found.\n", .{});
}

fn cmdModels(app: *App) Fatal!void {
    app.print("{s:<10} {s:<15} {s:<10} {s:<15} {s:>7} {s:>7}\n", .{ "NAME", "MODEL", "YEARS", "ECM ID", "SIZE", "OFFSET" });
    for (&models.all) |*m| {
        var idbuf: [14]u8 = undefined;
        app.print("{s:<10} {s:<15} {s:<10} {s:<15} {d:>6}k  0x{x}{s}\n", .{
            m.short(),                       m.name,         m.years,
            hexId(&idbuf, &m.ecm_id),        m.rom_size / 1024, m.write_offset,
            if (m.confidence == .partial) "  *" else "",
        });
    }
    app.print("\n* flash layout taken from HondaECU's README and stock image only\n", .{});
}

/// Detect the ECU state and require normal operation.
fn requireOk(app: *App, e: *Ecu) Fatal!void {
    const state = e.detectState() catch |err| return app.fail("{t}", .{err});
    if (state != .ok) return app.fail(
        "ECU state is: {s}\n       this command needs a running ECU (ignition on, kill switch on run)",
        .{state.describe()},
    );
}

fn cmdInfo(app: *App) (Fatal || std.mem.Allocator.Error)!void {
    const link = try connect(app);
    defer link.close(app.gpa);
    const e = &link.ecu;

    const state = e.detectState() catch |err| return app.fail("{t}", .{err});
    app.print("State:       {s}\n", .{state.describe()});
    if (state == .flash) if (e.write_status) |ws|
        app.print("Flash step:  0x{x:0>2} ({s})\n", .{ ws, ecu_mod.writeStatusName(ws) });
    if (e.ecm_id) |id| {
        var idbuf: [14]u8 = undefined;
        app.print("ECM ID:      {s}\n", .{hexId(&idbuf, &id)});
        if (models.byEcmId(&id)) |m| {
            printModel(app, m);
        } else if (state == .ok) {
            app.print("Model:       not in the table of supported ECUs\n", .{});
        }
    }
    if (state == .ok) {
        if (e.flashCount() catch null) |n| app.print("Flash count: {d}\n", .{n});
    }
    switch (state) {
        .off => app.print("\nSwitch the ignition on (kill switch on run) and check the cable.\n", .{}),
        .recover_old, .recover_new, .flash => app.print("\nThe ECU is waiting to be flashed: run `gromecu write <image>`.\n", .{}),
        else => {},
    }
}

fn cmdDtc(app: *App) (Fatal || std.mem.Allocator.Error)!void {
    const link = try connect(app);
    defer link.close(app.gpa);
    const e = &link.ecu;
    try requireOk(app, e);

    if (app.args.clear) {
        const ok = e.clearDtcs() catch |err| return app.fail("{t}", .{err});
        if (!ok) return app.fail("the ECU did not confirm clearing the codes", .{});
        app.print("Trouble codes cleared.\n", .{});
        return;
    }
    const list = (e.readDtcs() catch |err| return app.fail("{t}", .{err})) orelse
        return app.fail("the ECU stopped answering while reading trouble codes", .{});
    if (list.len == 0) app.print("No trouble codes.\n", .{});
    for (list.slice()) |d| {
        app.print("{s:<8} {d:0>2}-{d:0>2}  {s}\n", .{
            if (d.current) "current" else "stored",
            d.major,
            d.minor,
            models.dtcDescription(d.major, d.minor) orelse "(unknown code)",
        });
    }
}

const probe_tables = [_]u8{ 0x10, 0x11, 0x17, 0x20, 0x21, 0x60, 0x61, 0x67, 0x70, 0x71, 0xd0, 0xd1 };

fn cmdTables(app: *App) (Fatal || std.mem.Allocator.Error)!void {
    const link = try connect(app);
    defer link.close(app.gpa);
    const e = &link.ecu;
    try requireOk(app, e);

    for (probe_tables) |t| {
        const f = (e.readTable(t) catch |err| return app.fail("{t}", .{err})) orelse
            return app.fail("the ECU stopped answering at table 0x{x:0>2}", .{t});
        const d = f.data();
        if (d.len <= 2) continue; // table not present on this ECU
        app.print("table 0x{x:0>2}:", .{t});
        for (d[2..]) |b| app.print(" {x:0>2}", .{b});
        app.print("\n", .{});
    }
}

/// Decode the main sensor table (0x10 / 0x11), as HondaECU's data panel does.
fn printSensors(app: *App, d: []const u8) void {
    if (d.len < 14) return;
    const rpm = std.mem.readInt(u16, d[0..2], .big);
    const volts = struct {
        fn f(b: u8) f32 {
            return @as(f32, @floatFromInt(b)) * 5.0 / 255.0;
        }
    }.f;
    app.print("rpm {d:>5}  tps {d:>5.1}% ({d:.2}V)  ect {d}C  iat {d}C  map {d:>3}kPa  batt {d:>4.1}V  speed {d:>3}km/h", .{
        rpm,
        @as(f32, @floatFromInt(d[3])) / 1.6,
        volts(d[2]),
        @as(i32, d[5]) - 40,
        @as(i32, d[7]) - 40,
        d[9],
        @as(f32, @floatFromInt(d[12])) / 10.0,
        d[13],
    });
    if (d.len >= 17) {
        const inj = std.mem.readInt(u16, d[14..16], .big);
        app.print("  inj {d:.2}ms  ign {d:.1}deg", .{
            @as(f32, @floatFromInt(inj)) / 65535.0 * 265.5,
            -64.0 + @as(f32, @floatFromInt(d[16])) / 255.0 * 127.5,
        });
    }
    app.print("\n", .{});
}

fn cmdLive(app: *App) (Fatal || std.mem.Allocator.Error)!void {
    const link = try connect(app);
    defer link.close(app.gpa);
    const e = &link.ecu;
    try requireOk(app, e);

    var table: u8 = 0x11;
    for ([_]u8{ 0x11, 0x10 }) |t| {
        const f = (e.readTable(t) catch null) orelse continue;
        if (f.data().len >= 16) {
            table = t;
            break;
        }
    } else return app.fail("this ECU has neither sensor table 0x11 nor 0x10", .{});

    var shown: u32 = 0;
    while (app.args.count == null or shown < app.args.count.?) : (shown += 1) {
        const f = (e.readTable(table) catch |err| return app.fail("{t}", .{err})) orelse
            return app.fail("the ECU stopped answering", .{});
        printSensors(app, f.data()[2..]);
        app.flush();
        e.t.sleepMs(200);
    }
}

fn loadImage(app: *App, path: []const u8) Fatal![]u8 {
    return Io.Dir.cwd().readFileAlloc(app.io, path, app.gpa, .limited(4 << 20)) catch |err|
        return app.fail("cannot read {s}: {t}", .{ path, err });
}

fn fixChecksum(app: *App, image: []u8, addr: u32) Fatal!void {
    if (addr >= image.len) return app.fail("checksum address 0x{x} is outside the image", .{addr});
    const old = image[addr];
    image[addr] = 0;
    image[addr] = protocol.checksumHonda(image);
    if (old != image[addr])
        app.print("Checksum byte at 0x{x}: 0x{x:0>2} -> 0x{x:0>2}\n", .{ addr, old, image[addr] });
}

fn describeImage(app: *App, image: []const u8) ?*const models.Model {
    app.print("Size:        {d} bytes (0x{x})\n", .{ image.len, image.len });
    app.print("Checksum:    {s}\n", .{if (protocol.sumIsZero(image)) "good" else "BAD (8-bit sum is not zero)"});
    const m = models.identifyImage(image);
    if (m) |model| {
        printModel(app, model);
        if (models.keihinString(model, image)) |k| app.print("Calibration: {s}\n", .{k});
    } else {
        app.print("Model:       not recognised (size or embedded ECM ID match no supported ECU)\n", .{});
    }
    return m;
}

fn cmdCheck(app: *App) Fatal!void {
    const path = app.args.positional[0] orelse return app.fail("usage: gromecu check <file.bin>", .{});
    const image = try loadImage(app, path);
    defer app.gpa.free(image);

    if (app.args.fix_checksum) |addr| {
        const out_path = app.args.output orelse
            return app.fail("--fix-checksum needs -o <out.bin>; the input file is never modified", .{});
        try fixChecksum(app, image, addr);
        _ = describeImage(app, image);
        Io.Dir.cwd().writeFile(app.io, .{ .sub_path = out_path, .data = image, .flags = .{ .exclusive = !app.args.force } }) catch |err|
            return app.fail("cannot write {s}: {t}", .{ out_path, err });
        app.print("Wrote {s}\n", .{out_path});
        return;
    }
    _ = describeImage(app, image);
    if (!protocol.sumIsZero(image)) return error.Fatal;
}

fn report(ptr: ?*anyopaque, ev: ecu_mod.Event) void {
    const app: *App = @ptrCast(@alignCast(ptr.?));
    switch (ev) {
        .stage => |s| {
            app.endProgress();
            std.debug.print("{s}\n", .{switch (s) {
                .init_write => "Entering flash mode...",
                .init_recover => "Entering flash mode (recovery)...",
                .erase_wait => "Preparing to erase...",
                .erasing => "Erasing...",
                .writing => "Writing...",
                .finalizing => "Finalizing...",
                .reading => "Reading...",
            }});
        },
        .prompt => |p| {
            app.endProgress();
            std.debug.print("{s}\n", .{switch (p) {
                .ignition_off => ">>> Turn the ignition OFF.",
                .ignition_on => ">>> Turn the ignition ON (kill switch on run).",
            }});
        },
        .countdown => |s| app.progress("  waiting {d} s", .{s}),
        .erase_poll => |n| app.progress("  erasing {d:.1} s", .{@as(f32, @floatFromInt(n)) / 10.0}),
        .write_progress => |w| app.progress("  {d:>3}%  {d} of {d} bytes", .{ w.done * 100 / w.total, w.done, w.total }),
        .read_progress => |r| if (r.total) |total| {
            app.saveDump();
            app.progress("  {d:>3}%  {d} of {d} bytes", .{ r.done * 100 / total, r.done, total });
        } else {
            app.saveDump();
            app.progress("  {d} bytes", .{r.done});
        },
        .note => |text| {
            app.endProgress();
            std.debug.print("note: {s}\n", .{text});
        },
    }
}

fn cmdRead(app: *App) (Fatal || std.mem.Allocator.Error)!void {
    const path = app.args.positional[0] orelse return app.fail("usage: gromecu read <out.bin>", .{});
    if (!app.args.force) {
        if (Io.Dir.cwd().access(app.io, path, .{})) |_| {
            return app.fail("{s} already exists (use --force to overwrite)", .{path});
        } else |_| {}
    }

    const link = try connect(app);
    defer link.close(app.gpa);
    const e = &link.ecu;
    const reporter: ecu_mod.Reporter = .{ .ptr = app, .func = report };

    // Identify the ECU first if it is running, so the result can be checked.
    var expected: ?*const models.Model = null;
    if ((e.detectState() catch |err| return app.fail("{t}", .{err})) == .ok) {
        if (e.ecm_id) |id| expected = models.byEcmId(&id);
        if (expected) |m| printModel(app, m);
        app.flush();
    }

    std.debug.print("Reading needs security access, which the ECU only grants right after power-on.\n", .{});
    const unlocked = if (app.args.manual or app.args.no_echo) manual: {
        // The ignition cannot be sensed (or the user prefers not to rely on
        // it): let the user say when it has been switched.
        if (!waitEnter(app, ">>> Turn the ignition OFF, then press Enter.")) return app.fail("cancelled", .{});
        if (!waitEnter(app, ">>> Turn the ignition ON (kill switch on run) and press Enter right away.")) return app.fail("cancelled", .{});
        break :manual e.unlockAfterPowerOn(&app.args.password);
    } else e.enterSecure(&app.args.password, reporter, 120_000);
    unlocked catch |err| return switch (err) {
        error.IgnitionTimeout => app.fail(
            "gave up waiting for the ignition to be switched.\n" ++
                "       If the tool does not notice the ignition (some cables keep the K-line\n" ++
                "       powered), use --manual.",
            .{},
        ),
        error.EcuNotReady => app.fail("the ECU did not start normally after power-on (try `gromecu info`)", .{}),
        error.SecurityAccessDenied => app.fail("the ECU did not grant read access (wrong password, or it is not supported on this ECU)", .{}),
        else => app.fail("{t}", .{err}),
    };

    var data: std.ArrayList(u8) = .empty;
    defer data.deinit(app.gpa);
    const file = Io.Dir.cwd().createFile(app.io, path, .{}) catch |err|
        return app.fail("cannot create {s}: {t}", .{ path, err });
    defer file.close(app.io);
    app.dump = .{ .file = file, .data = &data };
    defer app.dump = null;

    const result = e.readFlash(app.gpa, &data, app.args.start, app.args.length, reporter);
    app.saveDump();
    app.endProgress();
    if (app.dump.?.failed) return app.fail("could not write to {s}", .{path});

    if (data.items.len == 0) return app.fail("nothing could be read", .{});
    result catch |err| return app.fail(
        "read stopped early ({t}); the {d} bytes read so far are in {s}",
        .{ err, data.items.len, path },
    );

    app.print("Read {d} bytes into {s}\n", .{ data.items.len, path });
    const whole = app.args.start == 0 and app.args.length == null;
    if (whole) {
        const got = describeImage(app, data.items);
        if (expected) |m| {
            if (data.items.len != m.rom_size)
                app.warn("expected {d} bytes for this ECU; the read may have ended early", .{m.rom_size})
            else if (got != m)
                app.warn(
                    "the image does not carry this ECU's ECM ID where expected.\n" ++
                        "         Reading from address 0 is HondaECU's default but is unconfirmed for\n" ++
                        "         this ECU; compare with --start 0x{x} --length 0x{x}.",
                    .{ m.write_offset, m.rom_size },
                );
        }
        if (!protocol.sumIsZero(data.items)) app.warn("bad checksum: do not flash this file back", .{});
    }
    app.print("Turn the ignition off and on again to return the ECU to normal operation.\n", .{});
}

/// Show a prompt and wait for Enter. False on end of input.
fn waitEnter(app: *App, prompt: []const u8) bool {
    app.flush();
    std.debug.print("{s} ", .{prompt});
    return (app.stdin.takeDelimiter('\n') catch return false) != null;
}

fn confirm(app: *App) bool {
    app.flush();
    std.debug.print("Type 'yes' to erase and flash the ECU: ", .{});
    const line = (app.stdin.takeDelimiter('\n') catch return false) orelse return false;
    return std.mem.eql(u8, std.mem.trim(u8, line, " \r\t"), "yes");
}

fn ignoreInterrupt() void {
    if (builtin.os.tag == .windows) return;
    const posix = std.posix;
    posix.sigaction(.INT, &.{
        .handler = .{ .handler = posix.SIG.IGN },
        .mask = posix.sigemptyset(),
        .flags = 0,
    }, null);
}

fn cmdWrite(app: *App) (Fatal || std.mem.Allocator.Error)!void {
    const path = app.args.positional[0] orelse return app.fail("usage: gromecu write <in.bin>", .{});
    const image = try loadImage(app, path);
    defer app.gpa.free(image);
    const force = app.args.force;

    // -- the image ----------------------------------------------------------
    app.print("Image:       {s}\n", .{path});
    if (app.args.fix_checksum) |addr| try fixChecksum(app, image, addr);
    var image_model = describeImage(app, image);
    if (!protocol.sumIsZero(image)) return app.fail(
        "the image checksum is bad; the ECU would not run it.\n" ++
            "       If this is an edited image, pass --fix-checksum <addr> with the\n" ++
            "       address of its checksum byte.",
        .{},
    );
    if (image.len == 0 or image.len % 128 != 0)
        return app.fail("image size must be a multiple of 128 bytes", .{});
    if (app.args.model) |name| {
        const m = models.byName(name) orelse return app.fail("unknown model '{s}' (see `gromecu models`)", .{name});
        if (image.len != m.rom_size)
            return app.fail("{s} images are {d} bytes, this file is {d}", .{ m.short(), m.rom_size, image.len });
        if (image_model != m) app.warn("treating the image as {s} on your say-so", .{m.short()});
        image_model = m;
    }
    const offset = app.args.offset orelse (if (image_model) |m| m.write_offset else
        return app.fail("cannot tell which ECU this image is for; pass --model or --offset", .{}));
    if (image_model) |m| {
        if (offset != m.write_offset and !force)
            return app.fail("offset 0x{x} differs from the 0x{x} known for {s} (--force to override)", .{ offset, m.write_offset, m.short() });
    } else if (!force) {
        return app.fail("the image is not recognised as being for a supported ECU (--force to flash it anyway)", .{});
    }

    // -- the ECU ------------------------------------------------------------
    const link = try connect(app);
    defer link.close(app.gpa);
    const e = &link.ecu;

    const state = e.detectState() catch |err| return app.fail("{t}", .{err});
    app.print("ECU state:   {s}\n", .{state.describe()});
    switch (state) {
        .ok, .recover_old, .recover_new, .flash => {},
        .off => return app.fail("no ECU: switch the ignition on (kill switch on run) and check the cable", .{}),
        .secure => return app.fail("the ECU is in read (secure) mode: cycle the ignition and try again", .{}),
        .unknown => return app.fail("the ECU is not answering: cycle the ignition and try again", .{}),
    }
    const id_known = if (e.ecm_id) |id| id[0] != 0 or id[1] != 0 or id[2] != 0 else false;
    if (id_known) {
        const id = e.ecm_id.?;
        var idbuf: [14]u8 = undefined;
        app.print("ECU ECM ID:  {s}\n", .{hexId(&idbuf, &id)});
        if (image_model) |m| {
            if (!models.sameFamily(&id, &m.ecm_id)) {
                if (!force) return app.fail(
                    "this image is for {s} {s} ({s}), which is a different ECU from the one connected.\n" ++
                        "       Flashing it would very likely leave the bike unable to run. (--force to override)",
                    .{ m.name, m.years, m.part_number },
                );
                app.warn("image and ECU are different ECU types; continuing because of --force", .{});
            } else if (state == .ok and !std.mem.eql(u8, &id, &m.ecm_id)) {
                app.warn("the image is for {s}, a different market/revision of the connected ECU", .{m.part_number});
            }
        }
        if (image_model == null) if (models.byEcmId(&id)) |ecu_model| {
            // --force lets an unrecognised image through, but the ECU's own
            // geometry is not negotiable.
            if (image.len != ecu_model.rom_size) return app.fail(
                "the connected {s} takes {d}-byte images; this file is {d} bytes",
                .{ ecu_model.short(), ecu_model.rom_size, image.len },
            );
            if (offset != ecu_model.write_offset) return app.fail(
                "the connected {s} is written at 0x{x}, not 0x{x}",
                .{ ecu_model.short(), ecu_model.write_offset, offset },
            );
        };
    } else {
        app.warn("the ECU does not report its ID in this state, so the image cannot be checked against it", .{});
    }
    if (image_model) |m| if (m.confidence == .partial)
        app.warn("the flash layout for {s} comes from HondaECU's README and the stock image only", .{m.short()});

    // -- go -----------------------------------------------------------------
    app.print(
        \\
        \\About to ERASE the ECU and write {d} bytes at 0x{x}.
        \\  - The battery must be healthy; connect a charger if in doubt.
        \\  - Keep the ignition on and the cable connected until it finishes.
        \\  - If it is interrupted, leave the ignition on and run the same
        \\    command again.
        \\
    , .{ image.len, offset });
    if (!app.args.yes and !confirm(app)) return app.fail("not confirmed, nothing was changed", .{});
    app.flush();

    ignoreInterrupt();
    const reporter: ecu_mod.Reporter = .{ .ptr = app, .func = report };
    e.flashImage(image, offset, reporter) catch |err| {
        app.endProgress();
        const again = "Leave the ignition on and run the same command again; if that fails,\n" ++
            "       switch the ignition off and on and run it once more (the ECU stays\n" ++
            "       in recovery mode until a complete image has been written).";
        return switch (err) {
            error.EcuNotReady => app.fail("the ECU is not ready to be flashed; nothing was changed", .{}),
            error.EcuFlashError => app.fail(
                "the ECU is in flash mode and reports an error.\n" ++
                    "       Switch the ignition off and on, then run the same command again.",
                .{},
            ),
            error.BadImageSize, error.BadOffset => app.fail("{t}; nothing was changed", .{err}),
            error.VerifyFailed => app.fail("the image was written but the ECU did not confirm it.\n       " ++ again, .{}),
            else => app.fail("flashing failed: {t}\n       " ++ again, .{err}),
        };
    };
    app.endProgress();
    app.print("Flash complete and confirmed by the ECU.\n", .{});
    if (e.stats.retries > 0) app.print("({d} retries, {d} checksum errors on the line)\n", .{ e.stats.retries, e.stats.checksum_errors });
    app.print("Turn the ignition off, wait a few seconds, and turn it back on.\n", .{});
}

fn announcePty(ctx: *anyopaque, path: []const u8) void {
    const app: *App = @ptrCast(@alignCast(ctx));
    app.print("Simulated ECU listening on {s}\n", .{path});
    app.print("  try:  gromecu --port {s} info\n", .{path});
    app.print("Press Enter here to switch the ignition off/on. Ctrl-C to quit.\n", .{});
    app.flush();
}

fn cmdEmu(app: *App) (Fatal || std.mem.Allocator.Error)!void {
    if (!emu.supported) return app.fail("the emulator needs Linux pseudo-terminals; use --port sim:<model> instead", .{});
    const name = app.args.model orelse "K26-911";
    const model = models.byName(name) orelse return app.fail("unknown model '{s}' (see `gromecu models`)", .{name});
    const flash = try app.gpa.alloc(u8, model.rom_size);
    defer app.gpa.free(flash);
    if (app.args.image) |path| {
        const image = try loadImage(app, path);
        defer app.gpa.free(image);
        if (image.len != flash.len) return app.fail("{s} images are {d} bytes, this file is {d}", .{ model.short(), flash.len, image.len });
        @memcpy(flash, image);
    } else {
        sim_mod.fillSyntheticImage(model, flash, 1);
    }
    var sim: sim_mod.Sim = .init(model, flash);
    emu.run(app.io, &sim, app.args.quiet, app, announcePty) catch |err|
        return app.fail("emulator: {t}", .{err});
}

pub fn main(init: std.process.Init) u8 {
    const arena = init.arena.allocator();
    var stdout_buffer: [4096]u8 = undefined;
    var stdout_writer: Io.File.Writer = .init(.stdout(), init.io, &stdout_buffer);
    const out = &stdout_writer.interface;
    defer out.flush() catch {};

    const argv = init.minimal.args.toSlice(arena) catch return 2;
    const args = parseArgs(argv) catch {
        std.debug.print("try `gromecu --help`\n", .{});
        return 2;
    };

    var stdin_buffer: [256]u8 = undefined;
    var stdin_reader = Io.File.stdin().reader(init.io, &stdin_buffer);

    var app: App = .{
        .stdin = &stdin_reader.interface,
        .io = init.io,
        .gpa = arena,
        .out = out,
        .args = args,
        .env_port = init.environ_map.get("GROMECU_PORT"),
        .stderr_is_tty = Io.File.stderr().isTty(init.io) catch false,
    };

    if (args.show_version) {
        app.print("gromecu {s}\n", .{version});
        return 0;
    }
    const command = args.command orelse {
        app.print("{s}", .{usage});
        return if (args.help) 0 else 2;
    };
    if (args.help) {
        app.print("{s}", .{usage});
        return 0;
    }

    const eql = std.mem.eql;
    const result = if (eql(u8, command, "ports"))
        cmdPorts(&app)
    else if (eql(u8, command, "models"))
        cmdModels(&app)
    else if (eql(u8, command, "info"))
        cmdInfo(&app)
    else if (eql(u8, command, "dtc"))
        cmdDtc(&app)
    else if (eql(u8, command, "tables"))
        cmdTables(&app)
    else if (eql(u8, command, "live"))
        cmdLive(&app)
    else if (eql(u8, command, "read"))
        cmdRead(&app)
    else if (eql(u8, command, "write"))
        cmdWrite(&app)
    else if (eql(u8, command, "check"))
        cmdCheck(&app)
    else if (eql(u8, command, "emu"))
        cmdEmu(&app)
    else {
        std.debug.print("error: unknown command '{s}'\ntry `gromecu --help`\n", .{command});
        return 2;
    };

    result catch |err| switch (err) {
        error.Fatal => {
            app.endProgress();
            return 1;
        },
        error.OutOfMemory => {
            std.debug.print("error: out of memory\n", .{});
            return 1;
        },
    };
    app.endProgress();
    return 0;
}

test {
    _ = @import("protocol.zig");
    _ = @import("models.zig");
    _ = @import("ecu.zig");
    _ = @import("sim.zig");
    _ = @import("serial.zig");
    _ = @import("emu.zig");
    _ = @import("tests.zig");
}

test "argument parsing" {
    const a = try parseArgs(&.{ "gromecu", "-p", "/dev/ttyUSB1", "write", "tune.bin", "--offset", "0x4000", "--yes" });
    try std.testing.expectEqualStrings("write", a.command.?);
    try std.testing.expectEqualStrings("tune.bin", a.positional[0].?);
    try std.testing.expectEqualStrings("/dev/ttyUSB1", a.port.?);
    try std.testing.expectEqual(@as(?u32, 0x4000), a.offset);
    try std.testing.expect(a.yes and !a.force);

    const pw = (try parseArgs(&.{ "gromecu", "read", "x.bin", "--password", "48656c6c6f486f77417265596f75" })).password;
    try std.testing.expectEqualStrings("HelloHowAreYou", &pw);
}
