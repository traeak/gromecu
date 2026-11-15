//! Known ECUs. The ECU is identified by the 5-byte ECM ID it reports, never
//! by model year: the same bike can carry different ECUs by market.
//!
//! Data from HondaECU's `ecmids.py` and README, cross-checked against the
//! stock images in HondaECU/bins.

const std = @import("std");
const protocol = @import("protocol.zig");

pub const Confidence = enum {
    /// Fully described by HondaECU's ECM ID table.
    reference,
    /// Pieced together from HondaECU's README and stock image; HondaECU's
    /// own table only lists the ID for this ECU.
    partial,
};

pub const Model = struct {
    ecm_id: [5]u8,
    name: []const u8,
    years: []const u8,
    part_number: []const u8,
    /// Size of a full image in bytes.
    rom_size: u32,
    /// Flash address the image is written to ("Start Offset" in HondaECU).
    write_offset: u32,
    /// Where the ECM ID is stored inside the image.
    ecm_id_addr: u32,
    /// Where the Keihin calibration string is stored inside the image.
    keihin_addr: u32,
    confidence: Confidence,

    /// Short name accepted on the command line, e.g. "K26-911".
    pub fn short(m: *const Model) []const u8 {
        return m.part_number["38770-".len..];
    }
};

pub const all = [_]Model{
    .{
        .ecm_id = .{ 0x01, 0x01, 0x35, 0x05, 0x01 },
        .name = "MSX125 / Grom",
        .years = "2013-2015",
        .part_number = "38770-K26-911",
        .rom_size = 0xC000,
        .write_offset = 0x4000,
        .ecm_id_addr = 0x97CD,
        .keihin_addr = 0x7601,
        .confidence = .reference,
    },
    .{
        .ecm_id = .{ 0x01, 0x01, 0x35, 0x01, 0x01 },
        .name = "MSX125 / Grom",
        .years = "2013-2015",
        .part_number = "38770-K26-931",
        .rom_size = 0xC000,
        .write_offset = 0x4000,
        .ecm_id_addr = 0x97CD,
        .keihin_addr = 0x7601,
        .confidence = .reference,
    },
    .{
        .ecm_id = .{ 0x01, 0x02, 0x13, 0x05, 0x01 },
        .name = "MSX125 / Grom",
        .years = "2016-2019",
        .part_number = "38770-K26-B13",
        .rom_size = 0x10000,
        .write_offset = 0x8000,
        .ecm_id_addr = 0x23B8,
        .keihin_addr = 0x1,
        .confidence = .reference,
    },
    .{
        .ecm_id = .{ 0x01, 0x02, 0x57, 0x05, 0x01 },
        .name = "MSX125 / Grom",
        .years = "2016-2019",
        .part_number = "38770-K26-C31",
        .rom_size = 0x10000,
        .write_offset = 0x8000,
        .ecm_id_addr = 0x260C,
        .keihin_addr = 0x1,
        .confidence = .reference,
    },
    .{
        // ecmids.py lists only the ID and part number. The start offset is
        // from HondaECU's README; size, ECM ID address and Keihin address
        // were read off the stock image (the README says 92 kb, the image
        // is 96 kb).
        .ecm_id = .{ 0x01, 0x02, 0xCA, 0x05, 0x01 },
        .name = "Z125M / Monkey",
        .years = "2018-2019",
        .part_number = "38770-K0F-A01",
        .rom_size = 0x18000,
        .write_offset = 0x8000,
        .ecm_id_addr = 0x3233,
        .keihin_addr = 0x1,
        .confidence = .partial,
    },
};

pub fn byEcmId(id: []const u8) ?*const Model {
    for (&all) |*m| {
        if (std.mem.eql(u8, &m.ecm_id, id)) return m;
    }
    return null;
}

/// Models of the same ECU family (first three ID bytes): same hardware,
/// different market or calibration revision.
pub fn sameFamily(a: []const u8, b: []const u8) bool {
    return a.len >= 3 and b.len >= 3 and std.mem.eql(u8, a[0..3], b[0..3]);
}

pub fn byName(name: []const u8) ?*const Model {
    for (&all) |*m| {
        if (std.ascii.eqlIgnoreCase(name, m.short()) or std.ascii.eqlIgnoreCase(name, m.part_number))
            return m;
    }
    return null;
}

/// Work out which ECU an image file belongs to from its size and the ECM ID
/// embedded in it.
pub fn identifyImage(image: []const u8) ?*const Model {
    for (&all) |*m| {
        if (image.len != m.rom_size) continue;
        if (std.mem.eql(u8, image[m.ecm_id_addr..][0..5], &m.ecm_id)) return m;
    }
    return null;
}

/// The 7-character Keihin calibration string, if it looks like one.
pub fn keihinString(m: *const Model, image: []const u8) ?[]const u8 {
    if (image.len < m.keihin_addr + 7) return null;
    const s = image[m.keihin_addr..][0..7];
    for (s) |c| if (!std.ascii.isAlphanumeric(c)) return null;
    return s;
}

pub const dtc_descriptions = [_]struct { code: [2]u8, text: []const u8 }{
    .{ .code = .{ 1, 1 }, .text = "MAP sensor circuit low voltage" },
    .{ .code = .{ 1, 2 }, .text = "MAP sensor circuit high voltage" },
    .{ .code = .{ 2, 1 }, .text = "MAP sensor performance problem" },
    .{ .code = .{ 7, 1 }, .text = "ECT sensor circuit low voltage" },
    .{ .code = .{ 7, 2 }, .text = "ECT sensor circuit high voltage" },
    .{ .code = .{ 8, 1 }, .text = "TP sensor circuit low voltage" },
    .{ .code = .{ 8, 2 }, .text = "TP sensor circuit high voltage" },
    .{ .code = .{ 9, 1 }, .text = "IAT sensor circuit low voltage" },
    .{ .code = .{ 9, 2 }, .text = "IAT sensor circuit high voltage" },
    .{ .code = .{ 11, 1 }, .text = "VS sensor no signal" },
    .{ .code = .{ 12, 1 }, .text = "No.1 primary injector circuit malfunction" },
    .{ .code = .{ 13, 1 }, .text = "No.2 primary injector circuit malfunction" },
    .{ .code = .{ 14, 1 }, .text = "No.3 primary injector circuit malfunction" },
    .{ .code = .{ 15, 1 }, .text = "No.4 primary injector circuit malfunction" },
    .{ .code = .{ 16, 1 }, .text = "No.1 secondary injector circuit malfunction" },
    .{ .code = .{ 17, 1 }, .text = "No.2 secondary injector circuit malfunction" },
    .{ .code = .{ 18, 1 }, .text = "CMP sensor no signal" },
    .{ .code = .{ 19, 1 }, .text = "CKP sensor no signal" },
    .{ .code = .{ 21, 1 }, .text = "O2 sensor malfunction" },
    .{ .code = .{ 23, 1 }, .text = "O2 sensor heater malfunction" },
    .{ .code = .{ 25, 2 }, .text = "Knock sensor circuit malfunction" },
    .{ .code = .{ 25, 3 }, .text = "Knock sensor circuit malfunction" },
    .{ .code = .{ 29, 1 }, .text = "IACV circuit malfunction" },
    .{ .code = .{ 33, 2 }, .text = "ECM EEPROM malfunction" },
    .{ .code = .{ 34, 1 }, .text = "ECV POT low voltage malfunction" },
    .{ .code = .{ 34, 2 }, .text = "ECV POT high voltage malfunction" },
    .{ .code = .{ 35, 1 }, .text = "EGCA malfunction" },
    .{ .code = .{ 48, 1 }, .text = "No.3 secondary injector circuit malfunction" },
    .{ .code = .{ 49, 1 }, .text = "No.4 secondary injector circuit malfunction" },
    .{ .code = .{ 51, 1 }, .text = "HESD linear solenoid malfunction" },
    .{ .code = .{ 54, 1 }, .text = "Bank angle sensor circuit low voltage" },
    .{ .code = .{ 54, 2 }, .text = "Bank angle sensor circuit high voltage" },
    .{ .code = .{ 56, 1 }, .text = "Knock sensor IC malfunction" },
    .{ .code = .{ 86, 1 }, .text = "Serial communication malfunction" },
};

pub fn dtcDescription(major: u8, minor: u8) ?[]const u8 {
    for (dtc_descriptions) |d| {
        if (d.code[0] == major and d.code[1] == minor) return d.text;
    }
    return null;
}

test "model table is self-consistent" {
    for (&all) |*m| {
        try std.testing.expect(m.rom_size % 128 == 0);
        try std.testing.expect(m.write_offset % 16 == 0);
        // Block addresses are 16-bit counts of 16 bytes.
        try std.testing.expect((m.write_offset + m.rom_size) / 16 <= 0x10000);
        try std.testing.expect(m.ecm_id_addr + 5 <= m.rom_size);
        try std.testing.expect(byEcmId(&m.ecm_id) == m);
        try std.testing.expect(byName(m.short()) == m);
    }
    try std.testing.expect(byName("k26-911") != null);
    try std.testing.expect(byName("K0F-A01").?.rom_size == 0x18000);
    try std.testing.expect(sameFamily(&all[0].ecm_id, &all[1].ecm_id));
    try std.testing.expect(!sameFamily(&all[0].ecm_id, &all[2].ecm_id));
}
