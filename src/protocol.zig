//! Honda/Keihin K-line message framing.
//!
//! A message is:  <type: 1..3 bytes> <length: 1 byte> <data...> <checksum>
//! where `length` counts every byte of the message including itself and the
//! checksum, and the checksum makes the 8-bit sum of the message zero.
//!
//! Ported from eculib (`format_message`, `checksum8bitHonda`, `checksum8bit`,
//! `format_read`) by Ryan M. Hope, GPL-3.

const std = @import("std");

pub const max_frame = 255;

/// Two's-complement checksum: appending it makes the 8-bit sum zero.
pub fn checksumHonda(data: []const u8) u8 {
    var sum: u8 = 0;
    for (data) |b| sum +%= b;
    return 0 -% sum;
}

/// True when the 8-bit sum of `data` is zero, i.e. a frame or ROM image with
/// its checksum byte in place is consistent.
pub fn sumIsZero(data: []const u8) bool {
    return checksumHonda(data) == 0;
}

/// The second checksum used inside flash write blocks:
/// `0xff - ((sum - 1) >> 8)`, as in eculib's `checksum8bit`.
pub fn checksum8(data: []const u8) u8 {
    var sum: i32 = 0;
    for (data) |b| sum += b;
    // Arithmetic shift, so an all-zero block gives 0x100, which the Python
    // original could not even encode. It is truncated to a byte here.
    return @truncate(@as(u32, @bitCast(0xff - ((sum - 1) >> 8))));
}

/// Address bytes of a flash read request, in the order the ECU expects:
/// bits 16-23, bits 0-7, bits 8-15.
pub fn formatRead(location: u32) [3]u8 {
    return .{
        @truncate(location >> 16),
        @truncate(location),
        @truncate(location >> 8),
    };
}

pub const Frame = struct {
    buf: [max_frame]u8 = undefined,
    len: usize = 0,
    type_len: usize = 0,

    pub fn bytes(f: *const Frame) []const u8 {
        return f.buf[0..f.len];
    }

    pub fn mtype(f: *const Frame) []const u8 {
        return f.buf[0..f.type_len];
    }

    /// The value of the length byte.
    pub fn lengthByte(f: *const Frame) u8 {
        return f.buf[f.type_len];
    }

    /// Payload between the length byte and the checksum. This is what the
    /// Python code calls `info[2]`.
    pub fn data(f: *const Frame) []const u8 {
        return f.buf[f.type_len + 1 .. f.len - 1];
    }
};

pub const BuildError = error{FrameTooLong};

pub fn build(mtype: []const u8, data: []const u8) BuildError!Frame {
    const total = mtype.len + 1 + data.len + 1;
    if (mtype.len == 0 or mtype.len > 3 or total > max_frame) return error.FrameTooLong;
    var f: Frame = .{ .len = total, .type_len = mtype.len };
    @memcpy(f.buf[0..mtype.len], mtype);
    f.buf[mtype.len] = @intCast(total);
    @memcpy(f.buf[mtype.len + 1 ..][0..data.len], data);
    f.buf[total - 1] = checksumHonda(f.buf[0 .. total - 1]);
    return f;
}

/// Does a response type belong to a request of type `req`?
///   1-byte types: the response carries the low nibble (0x72 -> 0x02)
///   2-byte types: echoed unchanged
///   3-byte types: first two bytes have bit 4 set (0x82 0x82 -> 0x92 0x92)
pub fn responseTypeMatches(req: []const u8, resp: []const u8) bool {
    if (req.len != resp.len) return false;
    return switch (req.len) {
        1 => resp[0] == (req[0] & 0x0f),
        2 => std.mem.eql(u8, req, resp),
        3 => resp[0] == (req[0] | 0x10) and resp[1] == (req[1] | 0x10),
        else => false,
    };
}

/// Number of type bytes of a *request* that starts with `first`.
/// Only the request types used by this tool are known.
pub fn requestTypeLen(first: u8) usize {
    return if (first == 0x82) 3 else 1;
}

/// Build one flash write block (`7e .. 01 06 <start> <data> <next> c1 c2`).
/// Addresses are in units of 16 bytes; `next` is 0 for the final block.
pub fn buildWriteBlock(start16: u16, block: []const u8, next16: u16) BuildError!Frame {
    var payload: [2 + 2 + 128 + 2 + 2]u8 = undefined;
    if (block.len > 128) return error.FrameTooLong;
    payload[0] = 0x01;
    payload[1] = 0x06;
    std.mem.writeInt(u16, payload[2..4], start16, .big);
    @memcpy(payload[4..][0..block.len], block);
    std.mem.writeInt(u16, payload[4 + block.len ..][0..2], next16, .big);
    const body = payload[2 .. 6 + block.len];
    payload[6 + block.len] = checksum8(body);
    payload[7 + block.len] = checksumHonda(body);
    return build(&.{0x7e}, payload[0 .. 8 + block.len]);
}

// ---------------------------------------------------------------------------
// Tests. The expected bytes were produced by running the reference Python
// (eculib 1.0.9) or come from ECUemu's captures of a real ECU.
// ---------------------------------------------------------------------------

const testing = std.testing;

fn expectFrame(expected: []const u8, f: Frame) !void {
    try testing.expectEqualSlices(u8, expected, f.bytes());
}

test "request frames match eculib" {
    try expectFrame(&.{ 0xfe, 0x04, 0x72, 0x8c }, try build(&.{0xfe}, &.{0x72}));
    try expectFrame(&.{ 0x72, 0x05, 0x71, 0x00, 0x18 }, try build(&.{0x72}, &.{ 0x71, 0x00 }));
    try expectFrame(&.{ 0x7d, 0x08, 0x01, 0x02, 0x50, 0x47, 0x4d, 0x94 }, try build(&.{0x7d}, &.{ 0x01, 0x02, 0x50, 0x47, 0x4d }));
    try expectFrame(
        &.{ 0x27, 0x0b, 0xe0, 0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x48, 0x6f, 0x43 },
        try build(&.{0x27}, &([_]u8{0xe0} ++ "HelloHo".*)),
    );
    try expectFrame(
        &.{ 0x27, 0x0b, 0xe0, 0x77, 0x41, 0x72, 0x65, 0x59, 0x6f, 0x75, 0x22 },
        try build(&.{0x27}, &([_]u8{0xe0} ++ "wAreYou".*)),
    );
}

test "request frames match ECUemu captures" {
    try expectFrame(&.{ 0x72, 0x05, 0x71, 0x11, 0x07 }, try build(&.{0x72}, &.{ 0x71, 0x11 }));
    try expectFrame(&.{ 0x72, 0x05, 0x71, 0xd1, 0x47 }, try build(&.{0x72}, &.{ 0x71, 0xd1 }));
    try expectFrame(&.{ 0x72, 0x05, 0x00, 0xf0, 0x99 }, try build(&.{0x72}, &.{ 0x00, 0xf0 }));
}

test "real ECU responses have a zero sum" {
    const table_11 = [_]u8{ 0x02, 0x19, 0x71, 0x11, 0x00, 0x00, 0x1C, 0x00, 0xC3, 0x2E, 0xC3, 0x2E, 0x90, 0x62, 0xFF, 0xFF, 0x77, 0x00, 0x00, 0x00, 0x80, 0x69, 0x1D, 0x14, 0xE4 };
    const table_d1 = [_]u8{ 0x02, 0x0B, 0x71, 0xD1, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2E };
    const init_ack = [_]u8{ 0x02, 0x04, 0x00, 0xFA };
    try testing.expect(sumIsZero(&table_11));
    try testing.expect(sumIsZero(&table_d1));
    try testing.expect(sumIsZero(&init_ack));
    try testing.expectEqual(@as(usize, table_11[1]), table_11.len);
    try testing.expect(responseTypeMatches(&.{0x72}, table_11[0..1]));
}

test "flash read request" {
    try testing.expectEqualSlices(u8, &.{ 0x01, 0x45, 0x23 }, &formatRead(0x012345));
    const loc = formatRead(0x012345);
    try expectFrame(
        &.{ 0x82, 0x82, 0x00, 0x09, 0x01, 0x45, 0x23, 0x0c, 0x7e },
        try build(&.{ 0x82, 0x82, 0x00 }, &.{ loc[0], loc[1], loc[2], 12 }),
    );
    try testing.expect(responseTypeMatches(&.{ 0x82, 0x82, 0x00 }, &.{ 0x92, 0x92, 0x00 }));
    try testing.expect(!responseTypeMatches(&.{ 0x82, 0x82, 0x00 }, &.{ 0x82, 0x82, 0x00 }));
}

test "checksum8 matches eculib" {
    try testing.expectEqual(@as(u8, 126), checksum8(&@as([130]u8, @splat(0xff))));
    try testing.expectEqual(@as(u8, 255), checksum8(&.{1}));
    try testing.expectEqual(@as(u8, 190), checksum8(&@as([132]u8, @splat(0x80))));
    var ramp: [132]u8 = undefined;
    for (&ramp, 0..) |*b, i| b.* = @intCast(i);
    try testing.expectEqual(@as(u8, 222), checksum8(&ramp));
    // Python yields 256 here; see the comment in checksum8.
    try testing.expectEqual(@as(u8, 0), checksum8(&@as([10]u8, @splat(0))));
}

test "write block matches eculib" {
    var data: [64]u8 = undefined;
    for (&data, 0..) |*b, i| b.* = @intCast(i);
    const f = try buildWriteBlock(0x0800, &data, 0x0804);
    try testing.expectEqual(@as(usize, 75), f.len);
    try testing.expectEqualSlices(u8, &.{ 0x7e, 0x4b, 0x01, 0x06, 0x08, 0x00, 0x00, 0x01 }, f.bytes()[0..8]);
    try testing.expectEqualSlices(u8, &.{ 0x3e, 0x3f, 0x08, 0x04, 0xf8, 0x0c, 0x38 }, f.bytes()[68..75]);
}

test "frame too long is rejected" {
    const big: [253]u8 = @splat(0);
    try testing.expectError(error.FrameTooLong, build(&.{0x7e}, &big));
}
