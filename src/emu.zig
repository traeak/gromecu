//! `gromecu emu`: the simulated ECU behind a pseudo-terminal (Linux only).
//!
//! This lets the real serial code path (termios2, break, poll) be exercised
//! end to end without hardware:
//!
//!     terminal 1:  gromecu emu --model K26-911
//!     terminal 2:  gromecu --port /dev/pts/N info
//!
//! Pressing Enter in terminal 1 toggles the "ignition".

const std = @import("std");
const builtin = @import("builtin");
const serial = @import("serial.zig");
const sim_mod = @import("sim.zig");

pub const supported = builtin.os.tag == .linux;

pub const Error = error{ PtyUnavailable, EmulatorIo };

fn logFrame(request: []const u8, response: ?[]const u8) void {
    std.debug.print("  >", .{});
    for (request) |b| std.debug.print(" {x:0>2}", .{b});
    if (response) |r| {
        std.debug.print("\n  <", .{});
        for (r) |b| std.debug.print(" {x:0>2}", .{b});
    } else {
        std.debug.print("\n  < (no answer)", .{});
    }
    std.debug.print("\n", .{});
}

/// Runs until the process is killed. `announce` is called once with the
/// path of the pty to connect to.
pub fn run(
    io: std.Io,
    sim: *sim_mod.Sim,
    quiet: bool,
    ctx: *anyopaque,
    announce: *const fn (ctx: *anyopaque, path: []const u8) void,
) Error!void {
    if (!supported) return error.PtyUnavailable;
    const posix = std.posix;
    const linux = std.os.linux;

    const master = posix.openat(posix.AT.FDCWD, "/dev/ptmx", .{ .ACCMODE = .RDWR, .NOCTTY = true }, 0) catch
        return error.PtyUnavailable;
    defer _ = linux.close(master);

    var unlock: c_int = 0;
    if (posix.errno(linux.ioctl(master, linux.T.IOCSPTLCK, @intFromPtr(&unlock))) != .SUCCESS) return error.PtyUnavailable;
    var number: c_uint = 0;
    if (posix.errno(linux.ioctl(master, linux.T.IOCGPTN, @intFromPtr(&number))) != .SUCCESS) return error.PtyUnavailable;

    var path_buf: [32]u8 = undefined;
    const path = std.fmt.bufPrint(&path_buf, "/dev/pts/{d}", .{number}) catch unreachable;

    // Until a client has switched the pty to raw mode, keep the line
    // discipline from echoing or line-buffering what we send.
    if (posix.tcgetattr(master)) |tio_in| {
        var tio = tio_in;
        tio.iflag = .{};
        tio.oflag = .{};
        tio.lflag = .{};
        posix.tcsetattr(master, .NOW, tio) catch {};
    } else |_| {}

    sim.require_break = false;
    if (!quiet) sim.on_frame = logFrame;
    announce(ctx, path);

    var watch_stdin = true;
    var buf: [512]u8 = undefined;
    while (true) {
        var fds = [_]posix.pollfd{
            .{ .fd = master, .events = posix.POLL.IN, .revents = 0 },
            .{ .fd = if (watch_stdin) posix.STDIN_FILENO else -1, .events = posix.POLL.IN, .revents = 0 },
        };
        _ = posix.poll(&fds, 1000) catch return error.EmulatorIo;

        if (fds[1].revents & (posix.POLL.IN | posix.POLL.HUP) != 0) {
            const n = posix.read(posix.STDIN_FILENO, &buf) catch 0;
            if (n == 0) {
                watch_stdin = false;
            } else if (sim.power) {
                sim.powerOff();
                std.debug.print("ignition OFF\n", .{});
            } else {
                sim.powerOn();
                std.debug.print("ignition ON\n", .{});
            }
        }

        if (fds[0].revents & posix.POLL.IN != 0) {
            const n = posix.read(master, &buf) catch 0;
            for (buf[0..n]) |b| {
                sim.now_ms = serial.nowMs(io);
                sim.feed(b);
            }
            var out: [1024]u8 = undefined;
            const m = sim.take(&out);
            var off: usize = 0;
            while (off < m) {
                const rc = linux.write(master, out[off..m].ptr, m - off);
                if (posix.errno(rc) != .SUCCESS) break;
                off += rc;
            }
        } else if (fds[0].revents & posix.POLL.HUP != 0) {
            // No client has the pty open.
            serial.sleepMs(io, 100);
        }
    }
}
