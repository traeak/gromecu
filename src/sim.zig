//! A simulated ECU, for testing the tool without a motorcycle.
//!
//! It speaks the request side of the protocol exactly as the reference
//! tools send it. The *responses* are a different matter: apart from the few
//! frames ECUemu captured from a real ECU, their contents are modelled on
//! what the reference code reads out of them (which byte it tests against
//! which value). So the simulator proves that this tool is consistent with
//! the reference's expectations, not that it matches a real ECU.

const std = @import("std");
const protocol = @import("protocol.zig");
const models = @import("models.zig");
const transport = @import("transport.zig");
const Transport = transport.Transport;

pub const Sim = struct {
    model: *const models.Model,
    /// Flash contents, `model.rom_size` bytes.
    flash: []u8,

    // -- behaviour switches ---------------------------------------------
    /// Need a wake-up pulse before answering. The pty emulator cannot see a
    /// break, so there the 0xfe ping wakes the ECU instead.
    require_break: bool = true,
    /// False models an ECU that only takes 64-byte write blocks.
    accept_128: bool = true,
    /// Lose power after accepting this many write blocks.
    die_after_blocks: ?usize = null,
    /// Lose the acknowledgement of this write block once, forcing a retry.
    drop_ack_of_block: ?usize = null,
    /// Stored trouble code to report, as major/minor.
    stored_dtc: ?[2]u8 = .{ 7, 1 },
    password: [14]u8 = "HelloHowAreYou".*,
    erase_ms: u64 = 2500,
    /// False models an adapter that suppresses the echo of its own bytes.
    echo: bool = true,

    // -- state ------------------------------------------------------------
    power: bool = true,
    awake: bool = false,
    mode: enum { normal, flash, secure } = .normal,
    /// Flash does not hold a complete image: the ECU boots into recovery.
    blank: bool = false,
    init_step: u8 = 0,
    password_half: bool = false,
    write_status: u8 = 0x00,
    erase_done_at: ?u64 = null,
    erased: bool = false,
    expected16: u32 = 0,
    blocks_written: usize = 0,
    /// "Next address" field of the most recent write block.
    last_next16: u16 = 0xffff,
    flash_count: u8 = 3,
    now_ms: u64 = 0,

    rx: [300]u8 = undefined,
    rx_len: usize = 0,
    rx_last_ms: u64 = 0,
    tx: [1024]u8 = undefined,
    tx_len: usize = 0,

    /// Frames seen, for the emulator's log.
    on_frame: ?*const fn (request: []const u8, response: ?[]const u8) void = null,

    pub fn init(model: *const models.Model, flash: []u8) Sim {
        std.debug.assert(flash.len == model.rom_size);
        return .{ .model = model, .flash = flash };
    }

    pub fn powerOff(s: *Sim) void {
        s.power = false;
        s.awake = false;
        s.mode = .normal;
        s.init_step = 0;
        s.password_half = false;
        s.erase_done_at = null;
        s.erased = false;
        s.write_status = 0;
        s.rx_len = 0;
        s.tx_len = 0;
    }

    pub fn powerOn(s: *Sim) void {
        s.power = true;
    }

    /// The wake-up pulse was seen on the line.
    pub fn wake(s: *Sim) void {
        if (s.power) s.awake = true;
    }

    /// One byte arrives on the K-line.
    pub fn feed(s: *Sim, byte: u8) void {
        if (!s.power) return; // dead line: not even an echo
        if (s.echo) s.push(&.{byte}); // single-wire bus: the sender hears itself

        // A stalled partial frame is abandoned.
        if (s.rx_len > 0 and s.now_ms -| s.rx_last_ms > 100) s.rx_len = 0;
        s.rx_last_ms = s.now_ms;

        if (s.rx_len == 0 and byte == 0xff) return; // K-line presence probe
        if (s.rx_len == s.rx.len) s.rx_len = 0;
        s.rx[s.rx_len] = byte;
        s.rx_len += 1;

        const type_len = protocol.requestTypeLen(s.rx[0]);
        if (s.rx_len <= type_len) return;
        const total: usize = s.rx[type_len];
        if (total < type_len + 2) {
            s.rx_len = 0;
            return;
        }
        if (s.rx_len < total) return;

        var req: protocol.Frame = .{ .len = total, .type_len = type_len };
        @memcpy(req.buf[0..total], s.rx[0..total]);
        s.rx_len = 0;
        if (!protocol.sumIsZero(req.bytes())) return;

        var resp: protocol.Frame = .{};
        const answered = s.handle(&req, &resp);
        if (s.on_frame) |f| f(req.bytes(), if (answered) resp.bytes() else null);
        if (answered) s.push(resp.bytes());
    }

    /// Take bytes the ECU (and the echo) put on the line.
    pub fn take(s: *Sim, buf: []u8) usize {
        const n = @min(buf.len, s.tx_len);
        @memcpy(buf[0..n], s.tx[0..n]);
        std.mem.copyForwards(u8, s.tx[0 .. s.tx_len - n], s.tx[n..s.tx_len]);
        s.tx_len -= n;
        return n;
    }

    fn push(s: *Sim, bytes: []const u8) void {
        const n = @min(bytes.len, s.tx.len - s.tx_len);
        @memcpy(s.tx[s.tx_len..][0..n], bytes[0..n]);
        s.tx_len += n;
    }

    fn reply(req: *const protocol.Frame, resp: *protocol.Frame, data: []const u8) bool {
        const rtype: [3]u8 = switch (req.type_len) {
            1 => .{ req.buf[0] & 0x0f, 0, 0 },
            else => .{ req.buf[0] | 0x10, req.buf[1] | 0x10, req.buf[2] },
        };
        resp.* = protocol.build(rtype[0..req.type_len], data) catch return false;
        return true;
    }

    fn handle(s: *Sim, req: *const protocol.Frame, resp: *protocol.Frame) bool {
        const d = req.data();
        const kind = req.buf[0];

        if (kind == 0xfe) {
            // ECUemu's capture shows no answer to the wake-up message.
            if (!s.require_break) s.awake = true;
            return false;
        }
        if (!s.awake) return false;

        switch (kind) {
            0x72 => return s.handleDiag(req, resp, d),
            0x7d => return s.handleInit(req, resp, d),
            0x7e => return s.handleFlash(req, resp, d),
            0x27 => return s.handleSecurity(req, resp, d),
            0x82 => return s.handleRead(req, resp, d),
            else => return false,
        }
    }

    fn handleDiag(s: *Sim, req: *const protocol.Frame, resp: *protocol.Frame, d: []const u8) bool {
        if (s.mode == .flash or d.len < 2) return false;
        switch (d[0]) {
            0x71 => switch (d[1]) {
                0x00 => {
                    var id = s.model.ecm_id;
                    if (s.blank) {
                        id[3] = 0;
                        id[4] = 0;
                    }
                    return reply(req, resp, &([_]u8{ 0x71, 0x00 } ++ id ++ [_]u8{ 0, 0, 0, 0 }));
                },
                // Captured from a real ECU by ECUemu.
                0x11 => return reply(req, resp, &.{ 0x71, 0x11, 0x00, 0x00, 0x1C, 0x00, 0xC3, 0x2E, 0xC3, 0x2E, 0x90, 0x62, 0xFF, 0xFF, 0x77, 0x00, 0x00, 0x00, 0x80, 0x69, 0x1D, 0x14 }),
                0xd1 => return reply(req, resp, &.{ 0x71, 0xD1, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00 }),
                // An empty table: just the request echoed back.
                else => return reply(req, resp, &.{ 0x71, d[1] }),
            },
            0x73, 0x74 => {
                var out = [_]u8{ d[0], d[1], 0, 0, 0, 0, 0, 0, 0 };
                if (d[0] == 0x73 and d[1] == 1) if (s.stored_dtc) |c| {
                    out[3] = c[0];
                    out[4] = c[1];
                };
                return reply(req, resp, &out);
            },
            0x60 => {
                s.stored_dtc = null;
                return reply(req, resp, &.{ 0x60, 0x00 });
            },
            // `72 05 00 f0 99` -> `02 04 00 fa`, captured by ECUemu.
            0x00 => return reply(req, resp, &.{0x00}),
            else => return false,
        }
    }

    const init_sequence = [5][]const u8{
        &.{ 0x01, 0x01, 0x01 },
        &.{ 0x01, 0x01, 0x02 },
        &.{ 0x01, 0x01, 0x03 },
        &.{ 0x01, 0x02, 0x50, 0x47, 0x4d },
        &.{ 0x01, 0x03, 0x2d, 0x46, 0x49 },
    };

    fn handleInit(s: *Sim, req: *const protocol.Frame, resp: *protocol.Frame, d: []const u8) bool {
        if (s.mode != .normal or d.len < 3) return false;
        if (s.init_step < init_sequence.len and std.mem.eql(u8, d, init_sequence[s.init_step])) {
            s.init_step += 1;
            if (s.init_step == init_sequence.len) {
                s.mode = .flash;
                s.write_status = 0x10;
                s.erased = false;
            }
        } else {
            s.init_step = 0;
        }
        // HondaECU reads the flash counter from byte 4 of this answer.
        return reply(req, resp, &.{ d[0], d[1], d[2], 0x00, s.flash_count });
    }

    fn handleFlash(s: *Sim, req: *const protocol.Frame, resp: *protocol.Frame, d: []const u8) bool {
        if (s.mode != .flash or d.len < 2 or d[0] != 0x01) return false;
        switch (d[1]) {
            0x01 => return reply(req, resp, &.{ 0x01, s.write_status }),
            0x02, 0x03, 0x0b, 0x0e, 0x07, 0x09, 0x0a, 0x0c => return reply(req, resp, &.{ 0x01, 0x00 }),
            0x04 => {
                s.erase_done_at = s.now_ms + s.erase_ms;
                s.write_status = 0x30;
                s.blank = true;
                @memset(s.flash, 0xff);
                return reply(req, resp, &.{ 0x01, 0x00 });
            },
            0x05 => {
                const done_at = s.erase_done_at orelse return reply(req, resp, &.{ 0x01, 0x01 });
                if (s.now_ms < done_at) return reply(req, resp, &.{ 0x01, 0x01 });
                s.erased = true;
                s.write_status = 0x40;
                s.expected16 = s.model.write_offset / 16;
                s.blocks_written = 0;
                return reply(req, resp, &.{ 0x01, 0x00 });
            },
            0x06 => return s.handleWriteBlock(req, resp, d),
            0x0d => {
                const end16 = (s.model.write_offset + s.model.rom_size) / 16;
                const good = s.erased and s.expected16 == end16 and protocol.sumIsZero(s.flash);
                if (good) {
                    s.blank = false;
                    s.flash_count +%= 1;
                    s.write_status = 0x0f;
                }
                return reply(req, resp, &.{ 0x01, if (good) 0x0f else 0xf0 });
            },
            else => return false,
        }
    }

    fn handleWriteBlock(s: *Sim, req: *const protocol.Frame, resp: *protocol.Frame, d: []const u8) bool {
        // 01 06 <start:2> <data:64|128> <next:2> <c1> <c2>
        if (!s.erased or d.len < 8) return false;
        const n = d.len - 8;
        if (n != 64 and n != 128) return false;
        if (n == 128 and !s.accept_128) return false;
        const body = d[2 .. 6 + n];
        if (d[6 + n] != protocol.checksum8(body) or d[7 + n] != protocol.checksumHonda(body)) return false;

        const start16: u32 = std.mem.readInt(u16, d[2..4], .big);
        const blocks16: u32 = @intCast(n / 16);
        if (start16 + blocks16 == s.expected16 and s.blocks_written > 0) {
            // A repeat of the block just written: say what is expected next.
            return reply(req, resp, &.{ 0x01, 0x06, @truncate(s.expected16 >> 8), @truncate(s.expected16) });
        }
        if (start16 != s.expected16) return false;
        const at = start16 * 16 - s.model.write_offset;
        if (at + n > s.flash.len) return false;
        @memcpy(s.flash[at..][0..n], d[4..][0..n]);
        s.last_next16 = std.mem.readInt(u16, d[4 + n ..][0..2], .big);
        s.expected16 += blocks16;
        const index = s.blocks_written;
        s.blocks_written += 1;

        if (s.die_after_blocks) |limit| if (s.blocks_written >= limit) {
            s.die_after_blocks = null;
            s.powerOff();
            return false;
        };
        if (s.drop_ack_of_block) |b| if (b == index) {
            s.drop_ack_of_block = null;
            return false;
        };
        return reply(req, resp, &.{ 0x01, 0x00 });
    }

    fn handleSecurity(s: *Sim, req: *const protocol.Frame, resp: *protocol.Frame, d: []const u8) bool {
        if (d.len == 8 and d[0] == 0xe0 and s.mode == .normal and !s.blank) {
            if (!s.password_half) {
                s.password_half = std.mem.eql(u8, d[1..8], s.password[0..7]);
            } else {
                if (std.mem.eql(u8, d[1..8], s.password[7..14])) s.mode = .secure;
                s.password_half = false;
            }
        }
        return reply(req, resp, &.{0x00});
    }

    fn handleRead(s: *Sim, req: *const protocol.Frame, resp: *protocol.Frame, d: []const u8) bool {
        if (s.mode != .secure or d.len != 4) return false;
        const location = @as(usize, d[0]) << 16 | @as(usize, d[2]) << 8 | d[1];
        const size: usize = d[3];
        if (size == 0 or size > 12 or location + size > s.flash.len) return false;
        return reply(req, resp, s.flash[location..][0..size]);
    }
};

/// Connects a `Sim` to the `Transport` interface on a virtual clock, so the
/// many fixed delays of the flash procedure take no real time.
pub const SimTransport = struct {
    sim: *Sim,
    clock_ms: u64 = 1000,
    break_since: ?u64 = null,
    /// Stand-in for a hand on the ignition key: after this many K-line
    /// probes the power goes off, and after as many again it comes back.
    cycle_power_after_probes: ?u32 = null,
    probes: u32 = 0,

    pub fn asTransport(st: *SimTransport) Transport {
        return .{ .ptr = st, .vtable = &vtable };
    }

    const vtable: Transport.VTable = .{
        .write = vWrite,
        .read = vRead,
        .setBreak = vSetBreak,
        .flushInput = vFlushInput,
        .sleepMs = vSleepMs,
        .nowMs = vNowMs,
    };

    fn self(ptr: *anyopaque) *SimTransport {
        return @ptrCast(@alignCast(ptr));
    }

    fn vWrite(ptr: *anyopaque, bytes: []const u8) transport.Error!void {
        const st = self(ptr);
        if (bytes.len == 1 and bytes[0] == 0xff) st.countProbe();
        for (bytes) |b| {
            st.clock_ms += 1;
            st.sim.now_ms = st.clock_ms;
            st.sim.feed(b);
        }
    }

    fn countProbe(st: *SimTransport) void {
        const n = st.cycle_power_after_probes orelse return;
        st.probes += 1;
        if (st.probes == n) st.sim.powerOff();
        if (st.probes == 2 * n) {
            st.sim.powerOn();
            st.cycle_power_after_probes = null;
        }
    }

    fn vRead(ptr: *anyopaque, buf: []u8, timeout_ms: u32) transport.Error!usize {
        const st = self(ptr);
        const n = st.sim.take(buf);
        if (n == 0) st.clock_ms += @max(timeout_ms, 1);
        return n;
    }

    fn vSetBreak(ptr: *anyopaque, on: bool) transport.Error!void {
        const st = self(ptr);
        if (on) {
            st.break_since = st.clock_ms;
        } else if (st.break_since) |since| {
            if (st.clock_ms - since >= 50) st.sim.wake();
            st.break_since = null;
        }
    }

    fn vFlushInput(ptr: *anyopaque) transport.Error!void {
        self(ptr).sim.tx_len = 0;
    }

    fn vSleepMs(ptr: *anyopaque, ms: u32) void {
        self(ptr).clock_ms += ms;
    }

    fn vNowMs(ptr: *anyopaque) u64 {
        return self(ptr).clock_ms;
    }
};

/// A valid stand-in image for a model: pseudo-random bytes with the ECM ID
/// in place and a zero 8-bit sum. Used when no stock image is at hand.
pub fn fillSyntheticImage(model: *const models.Model, image: []u8, seed: u64) void {
    var prng = std.Random.DefaultPrng.init(seed);
    prng.random().bytes(image);
    @memcpy(image[model.ecm_id_addr..][0..5], &model.ecm_id);
    image[0] = 0;
    image[0] = protocol.checksumHonda(image);
}
