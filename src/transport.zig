//! Byte transport abstraction for a half-duplex K-line.
//!
//! Everything above this layer (framing, ECU operations) only talks to a
//! `Transport`, so the same code drives a real serial adapter, the in-process
//! simulated ECU used by the tests, and the pty emulator.
//!
//! Time is part of the interface on purpose: the flash procedure contains
//! many fixed delays, and the simulator runs them on a virtual clock.

pub const Error = error{
    /// The underlying device reported an I/O error (often: adapter unplugged).
    TransportIo,
};

pub const Transport = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        write: *const fn (ptr: *anyopaque, bytes: []const u8) Error!void,
        /// Wait up to `timeout_ms` for input. Returns the number of bytes
        /// stored in `buf`; 0 means the timeout expired.
        read: *const fn (ptr: *anyopaque, buf: []u8, timeout_ms: u32) Error!usize,
        /// Hold the TX line in the break (low) state, or release it.
        setBreak: *const fn (ptr: *anyopaque, on: bool) Error!void,
        /// Throw away everything received so far.
        flushInput: *const fn (ptr: *anyopaque) Error!void,
        sleepMs: *const fn (ptr: *anyopaque, ms: u32) void,
        /// Monotonic milliseconds.
        nowMs: *const fn (ptr: *anyopaque) u64,
    };

    pub fn write(t: Transport, bytes: []const u8) Error!void {
        return t.vtable.write(t.ptr, bytes);
    }

    pub fn read(t: Transport, buf: []u8, timeout_ms: u32) Error!usize {
        return t.vtable.read(t.ptr, buf, timeout_ms);
    }

    pub fn setBreak(t: Transport, on: bool) Error!void {
        return t.vtable.setBreak(t.ptr, on);
    }

    pub fn flushInput(t: Transport) Error!void {
        return t.vtable.flushInput(t.ptr);
    }

    pub fn sleepMs(t: Transport, ms: u32) void {
        t.vtable.sleepMs(t.ptr, ms);
    }

    pub fn nowMs(t: Transport) u64 {
        return t.vtable.nowMs(t.ptr);
    }

    /// Read exactly `buf.len` bytes, giving up at the absolute time
    /// `deadline_ms`. Returns how many bytes were actually read.
    pub fn readExact(t: Transport, buf: []u8, deadline_ms: u64) Error!usize {
        var got: usize = 0;
        while (got < buf.len) {
            const now = t.nowMs();
            if (now >= deadline_ms) break;
            const n = try t.read(buf[got..], @intCast(@min(deadline_ms - now, 60_000)));
            got += n;
        }
        return got;
    }
};
