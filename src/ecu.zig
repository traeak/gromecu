//! Honda/Keihin ECU operations over a K-line transport.
//!
//! This is a port of the K-line side of HondaECU and eculib by Ryan M. Hope
//! (GPL-3). The command bytes, their order and the delays between them are
//! kept exactly as in the reference, because that sequence is the only one
//! known to work on real ECUs. Where the reference source was not available
//! the code says "inferred".
//!
//! Sources:
//!   eculib 1.0.9            framing, wake-up, state detection, init/erase/
//!                           post-write command lists
//!   HondaECU kline.py       flash block loop, erase polling, read loop,
//!                           password, DTCs, flash counter
//!   ECUemu                  break timing and real-ECU captures

const std = @import("std");
const protocol = @import("protocol.zig");
const transport = @import("transport.zig");
const Transport = transport.Transport;
const Frame = protocol.Frame;

pub const Error = transport.Error || protocol.BuildError;

/// The default "password" that unlocks flash reading (HondaECU's default).
pub const default_password: [14]u8 = "HelloHowAreYou".*;

pub const State = enum {
    /// No echo on the K-line: ignition off or adapter not connected.
    off,
    /// The line is alive but the ECU answers none of the known probes.
    unknown,
    /// Normal operation.
    ok,
    /// Flash is blank or incomplete; the ECU answers the 0x7d init commands.
    recover_old,
    /// As above, for ECUs that use the 0x7b init commands.
    recover_new,
    /// Flash mode is active: the ECU answers 0x7e flash commands.
    flash,
    /// Security access granted: the ECU answers flash read requests.
    secure,

    pub fn describe(s: State) []const u8 {
        return switch (s) {
            .off => "off (no K-line echo: ignition off or adapter not connected)",
            .unknown => "unknown (K-line alive, ECU not answering)",
            .ok => "ok (normal operation)",
            .recover_old => "recovery (flash incomplete, 0x7d init)",
            .recover_new => "recovery (flash incomplete, 0x7b init)",
            .flash => "flash mode",
            .secure => "secure (flash readable)",
        };
    }
};

/// Meaning of the status byte of `7e 01 01 00`, as named in the HondaECU
/// source before it moved to eculib.
pub fn writeStatusName(status: u8) []const u8 {
    return switch (status) {
        0x00 => "idle",
        0x0f => "write good",
        0x10 => "init (0x7d)",
        0x20 => "init (0x7b)",
        0x30 => "erase",
        0x40 => "write",
        else => "error",
    };
}

pub const Stats = struct {
    commands: u32 = 0,
    retries: u32 = 0,
    no_echo: u32 = 0,
    echo_mismatch: u32 = 0,
    checksum_errors: u32 = 0,
    unneeded_retries: u32 = 0,
};

pub const Stage = enum { init_write, init_recover, erase_wait, erasing, writing, finalizing, reading };
pub const Prompt = enum { ignition_off, ignition_on };

pub const Event = union(enum) {
    stage: Stage,
    /// The user has to do something before the operation continues.
    prompt: Prompt,
    /// Seconds left of the fixed wait before erasing.
    countdown: u32,
    /// Number of erase polls so far.
    erase_poll: u32,
    write_progress: struct { done: usize, total: usize, block: usize },
    read_progress: struct { done: usize, total: ?u32 },
    /// Something worth telling the user that does not stop the operation.
    note: []const u8,
};

pub const Reporter = struct {
    ptr: ?*anyopaque = null,
    func: ?*const fn (ptr: ?*anyopaque, ev: Event) void = null,

    pub const none: Reporter = .{};

    fn emit(r: Reporter, ev: Event) void {
        if (r.func) |f| f(r.ptr, ev);
    }
};

pub const Dtc = struct {
    current: bool,
    major: u8,
    minor: u8,
};

pub const DtcList = struct {
    items: [66]Dtc = undefined,
    len: usize = 0,

    pub fn slice(l: *const DtcList) []const Dtc {
        return l.items[0..l.len];
    }
};

pub const Options = struct {
    /// Extra attempts after the first one; null means `Ecu.retries`.
    retries: ?u8 = null,
    /// Pause after a successful exchange.
    delay_ms: u32 = 0,
};

pub const Ecu = struct {
    t: Transport,
    /// How long to wait for the ECU to start answering a request.
    timeout_ms: u32 = 200,
    /// Extra attempts for commands that do not say otherwise. 3 is
    /// HondaECU's shipped setting (eculib 1.0.9 itself defaulted to 1).
    /// Repeating a flash write block is safe: an ECU that already has the
    /// block answers with the address it expects next.
    retries: u8 = 3,
    /// How long to wait for the echo of the K-line presence probe.
    kline_timeout_ms: u32 = 100,
    /// A K-line is a single wire, so everything sent is also received.
    /// Turn off only for adapters that suppress their own echo.
    expect_echo: bool = true,
    /// Print every frame to stderr.
    trace: bool = false,
    stats: Stats = .{},
    /// ECM ID from the last successful state detection.
    ecm_id: ?[5]u8 = null,
    /// Status byte from the last `7e 01 01 00`.
    write_status: ?u8 = null,

    // -- framing ----------------------------------------------------------

    /// Send one request and wait for its response. Returns null when the ECU
    /// did not give a valid answer within the allowed attempts.
    pub fn command(e: *Ecu, mtype: []const u8, data: []const u8, opts: Options) Error!?Frame {
        const req = try protocol.build(mtype, data);
        const retries = opts.retries orelse e.retries;
        var attempt: u32 = 0;
        while (attempt <= retries) : (attempt += 1) {
            e.stats.commands += 1;
            if (attempt > 0) e.stats.retries += 1;
            e.traceFrame('>', attempt, req.bytes());
            const resp = (try e.exchange(&req)) orelse continue;
            if (!protocol.sumIsZero(resp.bytes())) {
                e.stats.checksum_errors += 1;
                e.traceFrame('!', attempt, resp.bytes());
                continue;
            }
            e.traceFrame('<', attempt, resp.bytes());
            // As in eculib: an answer of the wrong type is not retried.
            if (!protocol.responseTypeMatches(req.mtype(), resp.mtype())) return null;
            if (opts.delay_ms > 0) e.t.sleepMs(opts.delay_ms);
            return resp;
        }
        return null;
    }

    fn exchange(e: *Ecu, req: *const Frame) Error!?Frame {
        const t = e.t;
        try t.flushInput();
        try t.write(req.bytes());

        // At 10400 baud a byte takes about 1 ms on the wire.
        const wire_ms: u64 = 50 + 2 * @as(u64, req.len);
        if (e.expect_echo) {
            var echo: [protocol.max_frame]u8 = undefined;
            const n = try t.readExact(echo[0..req.len], t.nowMs() + wire_ms);
            if (n != req.len) {
                e.stats.no_echo += 1;
                return null;
            }
            if (!std.mem.eql(u8, echo[0..n], req.bytes())) {
                // Something else was driving the line while we were sending.
                e.stats.echo_mismatch += 1;
                return null;
            }
        }

        var f: Frame = .{ .type_len = req.type_len };
        const header = req.type_len + 1;
        const header_deadline = t.nowMs() + e.timeout_ms + (if (e.expect_echo) 0 else wire_ms);
        if (try t.readExact(f.buf[0..header], header_deadline) != header) return null;
        const total: usize = f.buf[header - 1];
        if (total < header + 1) return null;
        const body = f.buf[header..total];
        if (try t.readExact(body, t.nowMs() + 50 + 2 * @as(u64, body.len)) != body.len) return null;
        f.len = total;
        return f;
    }

    fn traceFrame(e: *Ecu, dir: u8, attempt: u32, bytes: []const u8) void {
        if (!e.trace) return;
        std.debug.print("{d:>8} {d} {c}", .{ e.t.nowMs(), attempt, dir });
        for (bytes) |b| std.debug.print(" {x:0>2}", .{b});
        std.debug.print("\n", .{});
    }

    // -- link ---------------------------------------------------------------

    /// Is the K-line powered? Sends one 0xff (on the wire: just a start bit)
    /// and looks for its echo. With the ignition off the line is held low
    /// and nothing sensible comes back.
    pub fn klinePresent(e: *Ecu) Error!bool {
        if (!e.expect_echo) return true;
        try e.t.flushInput();
        try e.t.write(&.{0xff});
        var b: [1]u8 = undefined;
        const n = try e.t.readExact(&b, e.t.nowMs() + e.kline_timeout_ms);
        return n == 1 and b[0] == 0xff;
    }

    /// The wake-up pulse: K-line low for 70 ms, then idle for 130 ms.
    pub fn wakeup(e: *Ecu) Error!void {
        try e.t.setBreak(true);
        e.t.sleepMs(70);
        try e.t.setBreak(false);
        try e.t.flushInput();
        e.t.sleepMs(130);
    }

    pub fn ping(e: *Ecu) Error!bool {
        return (try e.command(&.{0xfe}, &.{0x72}, .{ .retries = 0 })) != null;
    }

    /// Port of eculib 1.0.9 `detect_ecu_state`. Note that it is not passive:
    /// it may send the wake-up pulse and the first init command.
    pub fn detectState(e: *Ecu) Error!State {
        if (!try e.klinePresent()) return .off;
        var t0 = try e.command(&.{0x72}, &.{ 0x71, 0x00 }, .{ .retries = 0 });
        if (t0 == null) {
            try e.wakeup();
            _ = try e.ping();
            t0 = try e.command(&.{0x72}, &.{ 0x71, 0x00 }, .{ .retries = 0 });
        }
        if (t0) |f| {
            const d = f.data();
            if (d.len >= 7) {
                e.ecm_id = d[2..7].*;
                if (d[5] != 0 or d[6] != 0) return .ok;
            }
        }
        if ((try e.command(&.{0x7d}, &.{ 0x01, 0x01, 0x00 }, .{ .retries = 0 })) != null) return .recover_old;
        if ((try e.command(&.{0x7b}, &.{ 0x00, 0x01, 0x01 }, .{ .retries = 0 })) != null) return .recover_new;
        if (try e.command(&.{0x7e}, &.{ 0x01, 0x01, 0x00 }, .{ .retries = 0 })) |ws| {
            const d = ws.data();
            e.write_status = if (d.len >= 2) d[1] else null;
            return .flash;
        }
        if (try e.probeRead()) return .secure;
        return .unknown;
    }

    // -- information --------------------------------------------------------

    /// Number of times the ECU has been flashed. In HondaECU's idle loop
    /// this is byte 4 of the answer to `7d 01 01 03`.
    pub fn flashCount(e: *Ecu) Error!?u8 {
        const f = (try e.command(&.{0x7d}, &.{ 0x01, 0x01, 0x03 }, .{})) orelse return null;
        const d = f.data();
        return if (d.len >= 5) d[4] else null;
    }

    pub fn readTable(e: *Ecu, table: u8) Error!?Frame {
        return e.command(&.{0x72}, &.{ 0x71, table }, .{});
    }

    /// Current (0x74) and stored (0x73) trouble codes. Returns null if the
    /// ECU stopped answering part-way.
    pub fn readDtcs(e: *Ecu) Error!?DtcList {
        var list: DtcList = .{};
        for ([_]u8{ 0x74, 0x73 }) |kind| {
            var i: u8 = 1;
            while (i < 0x0c) : (i += 1) {
                const f = (try e.command(&.{0x72}, &.{ kind, i }, .{})) orelse return null;
                const d = f.data();
                for ([_]usize{ 3, 5, 7 }) |j| {
                    if (d.len > j + 1 and d[j] != 0) {
                        list.items[list.len] = .{ .current = kind == 0x74, .major = d[j], .minor = d[j + 1] };
                        list.len += 1;
                    }
                }
            }
        }
        return list;
    }

    pub fn clearDtcs(e: *Ecu) Error!bool {
        const f = (try e.command(&.{0x72}, &.{ 0x60, 0x03 }, .{})) orelse return false;
        const d = f.data();
        return d.len >= 2 and d[1] == 0x00;
    }

    // -- reading flash ------------------------------------------------------

    /// Send the two halves of the security password. HondaECU does this
    /// immediately after the ECU powers up.
    pub fn sendPassword(e: *Ecu, password: *const [14]u8) Error!bool {
        const p1 = try e.command(&.{0x27}, &([_]u8{0xe0} ++ password[0..7].*), .{});
        const p2 = try e.command(&.{0x27}, &([_]u8{0xe0} ++ password[7..14].*), .{});
        return p1 != null and p2 != null;
    }

    /// Read up to 12 bytes of flash. Only works with security access.
    pub fn readFlashBlock(e: *Ecu, location: u32, size: u8, opts: Options) Error!?Frame {
        if (size == 0 or size > 12) return null;
        const loc = protocol.formatRead(location);
        const f = (try e.command(&.{ 0x82, 0x82, 0x00 }, &.{ loc[0], loc[1], loc[2], size }, opts)) orelse return null;
        if (f.lengthByte() != @as(usize, size) + 5) return null;
        return f;
    }

    pub fn probeRead(e: *Ecu) Error!bool {
        return (try e.command(&.{ 0x82, 0x82, 0x00 }, &.{ 0x00, 0x00, 0x00, 0x08 }, .{ .retries = 0 })) != null;
    }

    pub const SecureError = Error || error{
        /// The ignition was not switched in time.
        IgnitionTimeout,
        /// The ECU did not come up in normal operation after power-on.
        EcuNotReady,
        /// The password was sent but flash reads are still refused.
        SecurityAccessDenied,
    };

    /// Get security access so that flash can be read.
    ///
    /// HondaECU sends the password at the moment the ECU goes from off to
    /// on, so this asks (through `reporter`) for the ignition to be cycled.
    /// `wait_ms` bounds each of the two waits.
    pub fn enterSecure(e: *Ecu, password: *const [14]u8, reporter: Reporter, wait_ms: u64) SecureError!void {
        if (try e.klinePresent() and try e.probeRead()) return;

        reporter.emit(.{ .prompt = .ignition_off });
        try e.waitForKline(false, wait_ms);
        reporter.emit(.{ .prompt = .ignition_on });
        try e.waitForKline(true, wait_ms);

        return e.unlockAfterPowerOn(password);
    }

    /// The part of `enterSecure` that follows power-on: wake the ECU, send
    /// the password, and check that reads now work. Call this directly when
    /// the ignition cannot be sensed through the K-line.
    pub fn unlockAfterPowerOn(e: *Ecu, password: *const [14]u8) SecureError!void {
        if (try e.detectState() != .ok) return error.EcuNotReady;
        if (!try e.sendPassword(password)) return error.SecurityAccessDenied;
        if (!try e.probeRead()) return error.SecurityAccessDenied;
    }

    /// Wait until the K-line has been steadily present/absent for three
    /// probes in a row.
    fn waitForKline(e: *Ecu, present: bool, wait_ms: u64) SecureError!void {
        const deadline = e.t.nowMs() + wait_ms;
        var streak: u32 = 0;
        while (streak < 3) {
            if (e.t.nowMs() > deadline) return error.IgnitionTimeout;
            streak = if (try e.klinePresent() == present) streak + 1 else 0;
            e.t.sleepMs(20);
        }
    }

    pub const ReadError = Error || std.mem.Allocator.Error || error{
        /// The K-line went dead while reading.
        Interrupted,
    };

    /// Port of HondaECU `read_flash`: read 12 bytes at a time from `start`;
    /// when a block fails, shrink the block size, and stop once even a
    /// single byte cannot be read (that is how the end of flash is found
    /// when `max_len` is null). Data is appended to `out` as it arrives, so
    /// a partial read survives an error.
    pub fn readFlash(
        e: *Ecu,
        gpa: std.mem.Allocator,
        out: *std.ArrayList(u8),
        start: u32,
        max_len: ?u32,
        reporter: Reporter,
    ) ReadError!void {
        reporter.emit(.{ .stage = .reading });
        var size: u8 = 12;
        var location = start;
        // No Honda ECU of this family has more flash than this; stops an
        // ECU that never refuses from being read forever.
        const hard_cap = 0x100000;
        while (out.items.len < hard_cap) {
            if (max_len) |m| {
                if (out.items.len >= m) break;
                size = @intCast(@min(size, m - out.items.len));
            }
            if (try e.readFlashBlock(location, size, .{})) |f| {
                try out.appendSlice(gpa, f.data());
                location += size;
                reporter.emit(.{ .read_progress = .{ .done = out.items.len, .total = max_len } });
            } else {
                size -= 1;
                if (size < 1) break;
            }
        }
        if (!try e.klinePresent()) return error.Interrupted;
    }

    // -- writing flash ------------------------------------------------------

    fn initWrite(e: *Ecu) Error!void {
        const o: Options = .{ .delay_ms = 500 };
        _ = try e.command(&.{0x7d}, &.{ 0x01, 0x01, 0x01 }, o);
        _ = try e.command(&.{0x7d}, &.{ 0x01, 0x01, 0x02 }, o);
        _ = try e.command(&.{0x7d}, &.{ 0x01, 0x01, 0x03 }, o);
        _ = try e.command(&.{0x7d}, &.{ 0x01, 0x02, 0x50, 0x47, 0x4d }, o); // "PGM"
        _ = try e.command(&.{0x7d}, &.{ 0x01, 0x03, 0x2d, 0x46, 0x49 }, o); // "-FI"
    }

    fn initRecover(e: *Ecu) Error!void {
        const o: Options = .{ .delay_ms = 500 };
        _ = try e.command(&.{0x7b}, &.{ 0x00, 0x01, 0x01 }, o);
        _ = try e.command(&.{0x7b}, &.{ 0x00, 0x01, 0x02 }, o);
        _ = try e.command(&.{0x7b}, &.{ 0x00, 0x01, 0x03 }, o);
        _ = try e.command(&.{0x7b}, &.{ 0x00, 0x02, 0x76, 0x03, 0x17 }, o);
        _ = try e.command(&.{0x7b}, &.{ 0x00, 0x03, 0x75, 0x05, 0x13 }, o);
        _ = try e.command(&.{0x72}, &.{ 0x00, 0xf1 }, .{});
        e.t.sleepMs(1000);
        _ = try e.command(&.{0x27}, &.{ 0x00, 0x01, 0x00 }, .{});
    }

    fn writeStatus(e: *Ecu) Error!?u8 {
        const f = (try e.command(&.{0x7e}, &.{ 0x01, 0x01, 0x00 }, .{})) orelse return null;
        const d = f.data();
        e.write_status = if (d.len >= 2) d[1] else null;
        return e.write_status;
    }

    fn eraseCommands(e: *Ecu) Error!void {
        const o: Options = .{ .delay_ms = 500 };
        _ = try e.command(&.{0x7e}, &.{ 0x01, 0x02 }, o);
        _ = try e.command(&.{0x7e}, &.{ 0x01, 0x03, 0x00, 0x00 }, o);
        _ = try e.command(&.{0x7e}, &.{ 0x01, 0x0b, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff }, o);
        _ = try e.command(&.{0x7e}, &.{ 0x01, 0x0e, 0x01, 0x90 }, o);
        _ = try e.command(&.{0x7e}, &.{ 0x01, 0x01, 0x01 }, o);
        _ = try e.command(&.{0x7e}, &.{ 0x01, 0x04, 0xff }, o);
    }

    fn postWrite(e: *Ecu) Error!bool {
        const o: Options = .{ .delay_ms = 500 };
        _ = try e.command(&.{0x7e}, &.{ 0x01, 0x09 }, o);
        _ = try e.command(&.{0x7e}, &.{ 0x01, 0x0a }, o);
        _ = try e.command(&.{0x7e}, &.{ 0x01, 0x0c }, o);
        const f = (try e.command(&.{0x7e}, &.{ 0x01, 0x0d }, o)) orelse return false;
        const d = f.data();
        return d.len >= 2 and d[1] == 0x0f;
    }

    pub const FlashError = Error || error{
        /// Image length is zero or not a multiple of 128 bytes.
        BadImageSize,
        /// Offset is not a multiple of 16 or the image does not fit in the
        /// 16-bit block address space.
        BadOffset,
        /// The ECU is not in a state from which flashing can start.
        EcuNotReady,
        /// The ECU is in flash mode but reports a flash error (status
        /// 0xf0). eculib treats this as a dead end; cycle the ignition.
        EcuFlashError,
        /// The ECU stopped answering while erasing.
        EraseFailed,
        /// The ECU reported an erase block error (status 0xfa).
        EraseBlockError,
        EraseTimeout,
        /// The ECU never accepted the first block, at either block size.
        WriteRejected,
        /// The ECU stopped answering part-way through. Leave the ignition
        /// on and run the write again.
        WriteNoResponse,
        /// The ECU acknowledged a different address than the one sent.
        WriteUnexpectedReply,
        /// Everything was sent but the ECU did not confirm the result.
        VerifyFailed,
    };

    /// Seconds HondaECU waits between entering flash mode and erasing.
    pub const erase_wait_s = 11;
    /// Upper bound on erase polling. The reference polls forever; its
    /// progress bar assumes about 3.5 s.
    const erase_timeout_ms = 60_000;

    /// Erase the ECU and write `image` at flash address `offset`.
    ///
    /// Port of HondaECU `write_helper` / `do_erase` / `write_flash` /
    /// `do_write`, including the state probes it makes between steps.
    /// Can be started from normal operation, from either recovery state, or
    /// from flash mode (to resume after an interrupted attempt).
    pub fn flashImage(e: *Ecu, image: []const u8, offset: u32, reporter: Reporter) FlashError!void {
        if (image.len == 0 or image.len % 128 != 0) return error.BadImageSize;
        if (offset % 16 != 0 or (@as(u64, offset) + image.len) / 16 > 0x10000) return error.BadOffset;

        const state = try e.detectState();
        const init: ?Stage = switch (state) {
            .ok, .recover_old => .init_write,
            .recover_new => .init_recover,
            .flash => if (e.write_status == 0xf0) return error.EcuFlashError else null,
            .off, .unknown, .secure => return error.EcuNotReady,
        };

        if (init) |stage| {
            reporter.emit(.{ .stage = stage });
            _ = try e.detectState();
            if (stage == .init_recover) try e.initRecover() else try e.initWrite();
            _ = try e.detectState();
            e.t.sleepMs(100);
            e.t.sleepMs(100);
        }

        try e.erase(reporter);

        _ = try e.detectState();
        reporter.emit(.{ .stage = .writing });
        try e.writeBlocks(image, offset, reporter);
        reporter.emit(.{ .stage = .finalizing });
        const good = try e.postWrite();
        _ = try e.detectState();
        if (!good) return error.VerifyFailed;
    }

    fn erase(e: *Ecu, reporter: Reporter) FlashError!void {
        _ = try e.detectState();
        reporter.emit(.{ .stage = .erase_wait });
        _ = try e.writeStatus();
        var left: u32 = erase_wait_s;
        while (left > 0) : (left -= 1) {
            reporter.emit(.{ .countdown = left });
            e.t.sleepMs(1000);
        }
        reporter.emit(.{ .stage = .erasing });
        // Inferred: eculib 1.0.37's do_erase() returned a success flag whose
        // source is lost. The poll below decides instead, which is at least
        // as strict: an ECU that is not erasing does not answer it with 0.
        try e.eraseCommands();
        e.t.sleepMs(2000);

        const started = e.t.nowMs();
        var polls: u32 = 0;
        while (true) {
            reporter.emit(.{ .erase_poll = polls });
            e.t.sleepMs(100);
            polls += 1;
            const f = (try e.command(&.{0x7e}, &.{ 0x01, 0x05 }, .{})) orelse return error.EraseFailed;
            const d = f.data();
            if (d.len >= 2) {
                if (d[1] == 0x00) break;
                if (d[1] == 0xfa) return error.EraseBlockError;
            }
            if (e.t.nowMs() - started > erase_timeout_ms) return error.EraseTimeout;
        }
        _ = try e.writeStatus();
        _ = try e.detectState();
    }

    fn writeBlocks(e: *Ecu, image: []const u8, offset: u32, reporter: Reporter) FlashError!void {
        const offseti: usize = offset / 16;
        var writesize: usize = 128;
        var z: usize = 8;
        var maxi: usize = image.len / writesize;
        var i: usize = 0;
        while (i < maxi) {
            const start16: u16 = @intCast(offseti + z * i);
            const next: usize = offseti + z * (i + 1);
            // The last block carries 0 as its "next" address. `next` can be
            // 0x10000 for an image ending at the top of the address space,
            // where the Python would have failed to pack it; it is only
            // ever sent for non-final blocks.
            const next16: u16 = if (i + 1 == maxi) 0 else @intCast(next);
            const frame = try protocol.buildWriteBlock(start16, image[i * writesize ..][0..writesize], next16);
            if (try e.command(&.{0x7e}, frame.data(), .{})) |resp| {
                const ilen = resp.lengthByte();
                if (ilen == 7) {
                    // The ECU already had this block (our retry was not
                    // needed) and reports the address it expects next.
                    const d = resp.data();
                    if (std.mem.readInt(u16, d[2..4], .big) != @as(u16, @truncate(next)))
                        return error.WriteUnexpectedReply;
                    e.stats.unneeded_retries += 1;
                } else if (ilen != 5) {
                    // The reference carries on here as well.
                    reporter.emit(.{ .note = "unexpected reply length to a write block" });
                }
            } else {
                if (i != 0) return error.WriteNoResponse;
                if (writesize != 128) return error.WriteRejected;
                // Some ECUs only take 64-byte blocks.
                writesize = 64;
                z = 4;
                maxi = image.len / writesize;
                reporter.emit(.{ .note = "128-byte blocks not accepted, trying 64-byte blocks" });
                continue;
            }
            i += 1;
            reporter.emit(.{ .write_progress = .{ .done = i * writesize, .total = image.len, .block = writesize } });
            if (i % 2 == 0 and writesize == 64) {
                _ = try e.command(&.{0x7e}, &.{ 0x01, 0x07 }, .{});
                e.t.sleepMs(200);
            }
        }
    }
};
