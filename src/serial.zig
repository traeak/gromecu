//! Serial port access for a USB K-line adapter.
//!
//! The reference tools talk to an FTDI chip through libftdi/libusb. Here the
//! operating system's own serial driver is used instead, so any adapter that
//! shows up as a serial port works (FTDI, CH340, CP210x, ...) and no driver
//! swapping or libusb is needed:
//!
//!   Linux    /dev/ttyUSB0      termios2 + BOTHER for the 10400 baud rate
//!   macOS    /dev/cu.usbserial IOSSIOSPEED for the 10400 baud rate
//!   Windows  COM3              SetCommState
//!
//! Only the Linux path has been exercised (against a pty); macOS and Windows
//! are compile-checked only.

const std = @import("std");
const builtin = @import("builtin");
const transport = @import("transport.zig");
const Transport = transport.Transport;
const Error = transport.Error;

pub const default_baud = 10400;

pub const OpenError = error{
    PortNotFound,
    PortAccessDenied,
    PortBusy,
    PortConfigFailed,
    NameTooLong,
};

pub const Serial = struct {
    io: std.Io,
    impl: Impl,

    const Impl = switch (builtin.os.tag) {
        .windows => WindowsImpl,
        else => PosixImpl,
    };

    pub fn open(io: std.Io, path: []const u8, baud: u32) OpenError!Serial {
        return .{ .io = io, .impl = try Impl.open(path, baud) };
    }

    pub fn close(s: *Serial) void {
        s.impl.close();
    }

    pub fn asTransport(s: *Serial) Transport {
        return .{ .ptr = s, .vtable = &vtable };
    }

    const vtable: Transport.VTable = .{
        .write = vWrite,
        .read = vRead,
        .setBreak = vSetBreak,
        .flushInput = vFlushInput,
        .sleepMs = vSleepMs,
        .nowMs = vNowMs,
    };

    fn self(ptr: *anyopaque) *Serial {
        return @ptrCast(@alignCast(ptr));
    }

    fn vWrite(ptr: *anyopaque, bytes: []const u8) Error!void {
        return self(ptr).impl.write(bytes);
    }

    fn vRead(ptr: *anyopaque, buf: []u8, timeout_ms: u32) Error!usize {
        return self(ptr).impl.read(buf, timeout_ms);
    }

    fn vSetBreak(ptr: *anyopaque, on: bool) Error!void {
        return self(ptr).impl.setBreak(on);
    }

    fn vFlushInput(ptr: *anyopaque) Error!void {
        return self(ptr).impl.flushInput();
    }

    fn vSleepMs(ptr: *anyopaque, ms: u32) void {
        sleepMs(self(ptr).io, ms);
    }

    fn vNowMs(ptr: *anyopaque) u64 {
        return nowMs(self(ptr).io);
    }
};

pub fn sleepMs(io: std.Io, ms: u32) void {
    io.sleep(.fromMilliseconds(ms), .awake) catch {};
}

pub fn nowMs(io: std.Io) u64 {
    const ts = std.Io.Timestamp.now(io, .awake);
    return @intCast(@max(0, ts.toMilliseconds()));
}

// ---------------------------------------------------------------------------
// POSIX (Linux, macOS, BSD)
// ---------------------------------------------------------------------------

const PosixImpl = struct {
    fd: posix.fd_t,

    const posix = std.posix;
    const system = posix.system;
    const linux = std.os.linux;
    const is_linux = builtin.os.tag == .linux;
    const is_darwin = builtin.os.tag.isDarwin();

    // Darwin ioctl request numbers (sys/ttycom.h, IOKit/serial/ioss.h).
    const darwin_TIOCSBRK: c_int = 0x2000747b;
    const darwin_TIOCCBRK: c_int = 0x2000747a;
    const darwin_TIOCFLUSH: c_int = @bitCast(@as(u32, 0x80047410));
    const darwin_IOSSIOSPEED: c_int = @bitCast(@as(u32, 0x80085402));
    const darwin_FREAD: c_int = 1;

    fn open(path: []const u8, baud: u32) OpenError!PosixImpl {
        // O_NONBLOCK so that opening never waits for a carrier signal that a
        // three-wire K-line adapter will not provide.
        const fd = posix.openat(posix.AT.FDCWD, path, .{
            .ACCMODE = .RDWR,
            .NOCTTY = true,
            .NONBLOCK = true,
        }, 0) catch |err| return switch (err) {
            error.FileNotFound, error.NoDevice, error.NotDir => error.PortNotFound,
            error.AccessDenied, error.PermissionDenied => error.PortAccessDenied,
            error.DeviceBusy => error.PortBusy,
            error.NameTooLong => error.NameTooLong,
            else => error.PortNotFound,
        };
        var p: PosixImpl = .{ .fd = fd };
        errdefer p.close();
        try p.configure(baud);
        return p;
    }

    fn close(p: *PosixImpl) void {
        // The exclusive flag outlives the descriptor on ptys.
        if (is_linux) _ = linux.ioctl(p.fd, linux.T.IOCNXCL, 0);
        _ = system.close(p.fd);
    }

    fn configure(p: *PosixImpl, baud: u32) OpenError!void {
        var tio = posix.tcgetattr(p.fd) catch return error.PortConfigFailed;
        // Raw 8N1, no flow control, receiver on, modem lines ignored.
        tio.iflag = .{};
        tio.oflag = .{};
        tio.lflag = .{};
        tio.cflag = .{};
        tio.cflag.CSIZE = .CS8;
        tio.cflag.CREAD = true;
        tio.cflag.CLOCAL = true;
        if (is_linux) {
            // On Linux the rate lives in cflag, and leaving it at zero would
            // ask for B0, i.e. "hang up" (DTR/RTS dropped). Use B38400 until
            // the real rate is set through termios2 just below.
            const b38400: u32 = 0o17;
            tio.cflag = @bitCast(@as(u32, @bitCast(tio.cflag)) | b38400);
        }
        tio.cc[@intFromEnum(posix.V.MIN)] = 0;
        tio.cc[@intFromEnum(posix.V.TIME)] = 0;
        if (!is_linux) {
            // A standard placeholder rate; Darwin gets the real one below and
            // the BSDs accept an arbitrary number here.
            const placeholder: posix.speed_t = if (is_darwin) .B9600 else @enumFromInt(baud);
            tio.ispeed = placeholder;
            tio.ospeed = placeholder;
        }
        posix.tcsetattr(p.fd, .NOW, tio) catch return error.PortConfigFailed;

        if (is_linux) {
            try p.linuxSetBaud(baud);
            p.linuxLowLatency();
            // Keep ModemManager and friends from opening the port mid-flash.
            _ = linux.ioctl(p.fd, linux.T.IOCEXCL, 0);
        } else if (is_darwin) {
            var speed: c_ulong = baud;
            if (std.c.ioctl(p.fd, darwin_IOSSIOSPEED, &speed) != 0) return error.PortConfigFailed;
        }
        p.flushInput() catch {};
    }

    /// 10400 baud is not one of the classic Bxxx constants, so it has to be
    /// requested through termios2 with BOTHER.
    fn linuxSetBaud(p: *PosixImpl, baud: u32) OpenError!void {
        const is_ppc = builtin.cpu.arch.isPowerPC();
        const cbaud: u32 = if (is_ppc) 0o377 else 0o010017;
        const bother: u32 = if (is_ppc) 0o037 else 0o010000;

        var t2: linux.termios2 = undefined;
        if (posix.errno(linux.ioctl(p.fd, linux.T.CGETS2, @intFromPtr(&t2))) != .SUCCESS)
            return error.PortConfigFailed;
        var cflag: u32 = @bitCast(t2.cflag);
        cflag = (cflag & ~cbaud) | bother;
        t2.cflag = @bitCast(cflag);
        // speed_t is an enum of the classic Bxxx codes in Zig's bindings; with
        // BOTHER these fields hold the rate itself, so write them as integers.
        speedField(&t2.ispeed).* = baud;
        speedField(&t2.ospeed).* = baud;
        if (posix.errno(linux.ioctl(p.fd, linux.T.CSETS2, @intFromPtr(&t2))) != .SUCCESS)
            return error.PortConfigFailed;

        // Some drivers silently pick the nearest rate they support.
        if (posix.errno(linux.ioctl(p.fd, linux.T.CGETS2, @intFromPtr(&t2))) == .SUCCESS) {
            const got: u32 = speedField(&t2.ospeed).*;
            const diff = if (got > baud) got - baud else baud - got;
            if (diff * 50 > baud) return error.PortConfigFailed; // off by more than 2%
        }
    }

    fn speedField(field: *linux.speed_t) *u32 {
        comptime std.debug.assert(@sizeOf(linux.speed_t) == @sizeOf(u32));
        return @ptrCast(field);
    }

    const SerialStruct = extern struct {
        type: c_int,
        line: c_int,
        port: c_uint,
        irq: c_int,
        flags: c_int,
        xmit_fifo_size: c_int,
        custom_divisor: c_int,
        baud_base: c_int,
        close_delay: c_ushort,
        io_type: u8,
        reserved_char: [1]u8,
        hub6: c_int,
        closing_wait: c_ushort,
        closing_wait2: c_ushort,
        iomem_base: ?*u8,
        iomem_reg_shift: c_ushort,
        port_high: c_uint,
        iomap_base: c_ulong,
    };

    /// FTDI adapters buffer received bytes for 16 ms by default, which makes
    /// every request/response round trip slow. ASYNC_LOW_LATENCY drops that
    /// to 1 ms. Best effort: other drivers reject it, which is fine.
    fn linuxLowLatency(p: *PosixImpl) void {
        const TIOCSSERIAL = 0x541F;
        const ASYNC_LOW_LATENCY = 1 << 13;
        var ss: SerialStruct = undefined;
        if (posix.errno(linux.ioctl(p.fd, linux.T.IOCGSERIAL, @intFromPtr(&ss))) != .SUCCESS) return;
        ss.flags |= ASYNC_LOW_LATENCY;
        _ = linux.ioctl(p.fd, TIOCSSERIAL, @intFromPtr(&ss));
    }

    fn write(p: *PosixImpl, bytes: []const u8) Error!void {
        var off: usize = 0;
        while (off < bytes.len) {
            const rc = system.write(p.fd, bytes.ptr + off, bytes.len - off);
            switch (posix.errno(rc)) {
                .SUCCESS => off += @intCast(rc),
                .INTR => continue,
                .AGAIN => {
                    var fds = [_]posix.pollfd{.{ .fd = p.fd, .events = posix.POLL.OUT, .revents = 0 }};
                    _ = posix.poll(&fds, 1000) catch return error.TransportIo;
                },
                else => return error.TransportIo,
            }
        }
    }

    fn read(p: *PosixImpl, buf: []u8, timeout_ms: u32) Error!usize {
        if (buf.len == 0) return 0;
        var fds = [_]posix.pollfd{.{ .fd = p.fd, .events = posix.POLL.IN, .revents = 0 }};
        const ready = posix.poll(&fds, @intCast(@min(timeout_ms, std.math.maxInt(i32)))) catch
            return error.TransportIo;
        if (ready == 0) return 0;
        if (fds[0].revents & (posix.POLL.ERR | posix.POLL.NVAL) != 0) return error.TransportIo;
        if (fds[0].revents & posix.POLL.IN == 0) {
            // Hang-up with nothing left to read: the adapter is gone.
            if (fds[0].revents & posix.POLL.HUP != 0) return error.TransportIo;
            return 0;
        }
        return posix.read(p.fd, buf) catch |err| switch (err) {
            error.WouldBlock => 0,
            else => error.TransportIo,
        };
    }

    fn setBreak(p: *PosixImpl, on: bool) Error!void {
        if (is_linux) {
            const req: u32 = if (on) linux.T.IOCSBRK else linux.T.IOCCBRK;
            if (posix.errno(linux.ioctl(p.fd, req, 0)) != .SUCCESS) return error.TransportIo;
        } else {
            const req: c_int = if (is_darwin)
                (if (on) darwin_TIOCSBRK else darwin_TIOCCBRK)
            else
                @compileError("K-line break is not implemented for this OS");
            if (std.c.ioctl(p.fd, req, @as(c_int, 0)) != 0) return error.TransportIo;
        }
    }

    fn flushInput(p: *PosixImpl) Error!void {
        if (is_linux) {
            const TCIFLUSH = 0;
            if (posix.errno(linux.ioctl(p.fd, linux.T.CFLSH, TCIFLUSH)) != .SUCCESS) return error.TransportIo;
        } else if (is_darwin) {
            var what: c_int = darwin_FREAD;
            if (std.c.ioctl(p.fd, darwin_TIOCFLUSH, &what) != 0) return error.TransportIo;
        } else {
            var scratch: [256]u8 = undefined;
            while ((try p.read(&scratch, 0)) != 0) {}
        }
    }
};

// ---------------------------------------------------------------------------
// Windows
// ---------------------------------------------------------------------------

const WindowsImpl = struct {
    handle: HANDLE,
    read_timeout_ms: u32 = std.math.maxInt(u32),

    const HANDLE = std.os.windows.HANDLE;
    const DWORD = u32;
    const WINBOOL = c_int;

    const DCB = extern struct {
        DCBlength: DWORD,
        BaudRate: DWORD,
        flags: DWORD,
        wReserved: u16,
        XonLim: u16,
        XoffLim: u16,
        ByteSize: u8,
        Parity: u8,
        StopBits: u8,
        XonChar: u8,
        XoffChar: u8,
        ErrorChar: u8,
        EofChar: u8,
        EvtChar: u8,
        wReserved1: u16,
    };

    const COMMTIMEOUTS = extern struct {
        ReadIntervalTimeout: DWORD,
        ReadTotalTimeoutMultiplier: DWORD,
        ReadTotalTimeoutConstant: DWORD,
        WriteTotalTimeoutMultiplier: DWORD,
        WriteTotalTimeoutConstant: DWORD,
    };

    extern "kernel32" fn CreateFileW(
        name: [*:0]const u16,
        access: DWORD,
        share: DWORD,
        security: ?*anyopaque,
        disposition: DWORD,
        flags: DWORD,
        template: ?HANDLE,
    ) callconv(.winapi) HANDLE;
    extern "kernel32" fn CloseHandle(h: HANDLE) callconv(.winapi) WINBOOL;
    extern "kernel32" fn GetCommState(h: HANDLE, dcb: *DCB) callconv(.winapi) WINBOOL;
    extern "kernel32" fn SetCommState(h: HANDLE, dcb: *const DCB) callconv(.winapi) WINBOOL;
    extern "kernel32" fn SetCommTimeouts(h: HANDLE, t: *const COMMTIMEOUTS) callconv(.winapi) WINBOOL;
    extern "kernel32" fn SetCommBreak(h: HANDLE) callconv(.winapi) WINBOOL;
    extern "kernel32" fn ClearCommBreak(h: HANDLE) callconv(.winapi) WINBOOL;
    extern "kernel32" fn PurgeComm(h: HANDLE, flags: DWORD) callconv(.winapi) WINBOOL;
    extern "kernel32" fn ReadFile(h: HANDLE, buf: [*]u8, len: DWORD, got: *DWORD, ov: ?*anyopaque) callconv(.winapi) WINBOOL;
    extern "kernel32" fn WriteFile(h: HANDLE, buf: [*]const u8, len: DWORD, put: *DWORD, ov: ?*anyopaque) callconv(.winapi) WINBOOL;
    extern "kernel32" fn GetLastError() callconv(.winapi) DWORD;

    const GENERIC_READ: DWORD = 0x80000000;
    const GENERIC_WRITE: DWORD = 0x40000000;
    const OPEN_EXISTING: DWORD = 3;
    const PURGE_RXCLEAR: DWORD = 0x0008;
    const ERROR_FILE_NOT_FOUND: DWORD = 2;
    const ERROR_ACCESS_DENIED: DWORD = 5;

    fn open(path: []const u8, baud: u32) OpenError!WindowsImpl {
        // "COM12" must be opened as "\\.\COM12".
        var name8: [80]u8 = undefined;
        const full = if (std.mem.startsWith(u8, path, "\\\\"))
            path
        else
            std.fmt.bufPrint(&name8, "\\\\.\\{s}", .{path}) catch return error.NameTooLong;
        var name16: [96:0]u16 = undefined;
        if (full.len >= name16.len) return error.NameTooLong;
        const n = std.unicode.utf8ToUtf16Le(&name16, full) catch return error.PortNotFound;
        name16[n] = 0;

        const h = CreateFileW(&name16, GENERIC_READ | GENERIC_WRITE, 0, null, OPEN_EXISTING, 0, null);
        if (h == std.os.windows.INVALID_HANDLE_VALUE) return switch (GetLastError()) {
            ERROR_FILE_NOT_FOUND => error.PortNotFound,
            ERROR_ACCESS_DENIED => error.PortBusy,
            else => error.PortNotFound,
        };
        var w: WindowsImpl = .{ .handle = h };
        errdefer w.close();

        var dcb: DCB = std.mem.zeroes(DCB);
        dcb.DCBlength = @sizeOf(DCB);
        if (GetCommState(h, &dcb) == 0) return error.PortConfigFailed;
        dcb.BaudRate = baud;
        // fBinary, DTR and RTS held asserted (as a POSIX open does), no flow control.
        dcb.flags = 0x0001 | (1 << 4) | (1 << 12);
        dcb.ByteSize = 8;
        dcb.Parity = 0;
        dcb.StopBits = 0;
        if (SetCommState(h, &dcb) == 0) return error.PortConfigFailed;
        w.setReadTimeout(0) catch return error.PortConfigFailed;
        _ = PurgeComm(h, PURGE_RXCLEAR);
        return w;
    }

    fn close(w: *WindowsImpl) void {
        _ = CloseHandle(w.handle);
    }

    /// Makes ReadFile return as soon as any data is available, or after
    /// `ms` milliseconds with nothing.
    fn setReadTimeout(w: *WindowsImpl, ms: u32) Error!void {
        if (w.read_timeout_ms == ms) return;
        const max = std.math.maxInt(DWORD);
        const t: COMMTIMEOUTS = if (ms == 0) .{
            .ReadIntervalTimeout = max,
            .ReadTotalTimeoutMultiplier = 0,
            .ReadTotalTimeoutConstant = 0,
            .WriteTotalTimeoutMultiplier = 0,
            .WriteTotalTimeoutConstant = 1000,
        } else .{
            .ReadIntervalTimeout = max,
            .ReadTotalTimeoutMultiplier = max,
            .ReadTotalTimeoutConstant = @min(ms, max - 1),
            .WriteTotalTimeoutMultiplier = 0,
            .WriteTotalTimeoutConstant = 1000,
        };
        if (SetCommTimeouts(w.handle, &t) == 0) return error.TransportIo;
        w.read_timeout_ms = ms;
    }

    fn write(w: *WindowsImpl, bytes: []const u8) Error!void {
        var off: usize = 0;
        while (off < bytes.len) {
            var put: DWORD = 0;
            if (WriteFile(w.handle, bytes.ptr + off, @intCast(bytes.len - off), &put, null) == 0 or put == 0)
                return error.TransportIo;
            off += put;
        }
    }

    fn read(w: *WindowsImpl, buf: []u8, timeout_ms: u32) Error!usize {
        if (buf.len == 0) return 0;
        try w.setReadTimeout(timeout_ms);
        var got: DWORD = 0;
        if (ReadFile(w.handle, buf.ptr, @intCast(@min(buf.len, 4096)), &got, null) == 0)
            return error.TransportIo;
        return got;
    }

    fn setBreak(w: *WindowsImpl, on: bool) Error!void {
        const ok = if (on) SetCommBreak(w.handle) else ClearCommBreak(w.handle);
        if (ok == 0) return error.TransportIo;
    }

    fn flushInput(w: *WindowsImpl) Error!void {
        if (PurgeComm(w.handle, PURGE_RXCLEAR) == 0) return error.TransportIo;
    }
};
