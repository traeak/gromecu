//! End-to-end tests of the ECU operations against the simulated ECU.
//! See the caveat at the top of sim.zig about what these do and do not prove.

const std = @import("std");
const testing = std.testing;
const ecu_mod = @import("ecu.zig");
const Ecu = ecu_mod.Ecu;
const models = @import("models.zig");
const protocol = @import("protocol.zig");
const sim_mod = @import("sim.zig");

const Rig = struct {
    model: *const models.Model,
    stock: []u8,
    flash: []u8,
    sim: sim_mod.Sim,
    st: sim_mod.SimTransport,
    ecu: Ecu,

    /// Heap-allocated so the internal pointers stay valid.
    fn create(name: []const u8) !*Rig {
        const gpa = testing.allocator;
        const model = models.byName(name).?;
        const rig = try gpa.create(Rig);
        errdefer gpa.destroy(rig);
        const stock = try gpa.alloc(u8, model.rom_size);
        errdefer gpa.free(stock);
        sim_mod.fillSyntheticImage(model, stock, 1);
        const flash = try gpa.dupe(u8, stock);
        rig.* = .{
            .model = model,
            .stock = stock,
            .flash = flash,
            .sim = .init(model, flash),
            .st = undefined,
            .ecu = undefined,
        };
        rig.st = .{ .sim = &rig.sim };
        rig.ecu = .{ .t = rig.st.asTransport() };
        return rig;
    }

    fn destroy(rig: *Rig) void {
        const gpa = testing.allocator;
        gpa.free(rig.stock);
        gpa.free(rig.flash);
        gpa.destroy(rig);
    }

    fn tune(rig: *Rig) ![]u8 {
        const image = try testing.allocator.alloc(u8, rig.model.rom_size);
        sim_mod.fillSyntheticImage(rig.model, image, 2);
        return image;
    }
};

test "synthetic images are valid and distinct" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    const image = try rig.tune();
    defer testing.allocator.free(image);
    try testing.expect(protocol.sumIsZero(rig.stock));
    try testing.expect(protocol.sumIsZero(image));
    try testing.expect(!std.mem.eql(u8, rig.stock, image));
    try testing.expect(models.identifyImage(image) == rig.model);
}

test "detect state, id, flash count and trouble codes" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    const e = &rig.ecu;

    try testing.expectEqual(ecu_mod.State.ok, try e.detectState());
    try testing.expectEqualSlices(u8, &rig.model.ecm_id, &e.ecm_id.?);
    try testing.expectEqual(@as(?u8, 3), try e.flashCount());

    const dtcs = (try e.readDtcs()).?;
    try testing.expectEqual(@as(usize, 1), dtcs.len);
    try testing.expectEqual(ecu_mod.Dtc{ .current = false, .major = 7, .minor = 1 }, dtcs.items[0]);
    try testing.expect(try e.clearDtcs());
    try testing.expectEqual(@as(usize, 0), (try e.readDtcs()).?.len);

    const table = (try e.readTable(0x11)).?;
    try testing.expectEqual(@as(usize, 22), table.data().len);
}

test "ignition off is reported as off" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    rig.sim.powerOff();
    try testing.expectEqual(ecu_mod.State.off, try rig.ecu.detectState());
}

test "read the whole flash after security access" {
    for ([_][]const u8{ "K26-911", "K0F-A01" }) |name| {
        const rig = try Rig.create(name);
        defer rig.destroy();
        const e = &rig.ecu;
        rig.st.cycle_power_after_probes = 5;

        // Without access, reads are refused.
        try testing.expectEqual(ecu_mod.State.ok, try e.detectState());
        try testing.expect(!try e.probeRead());

        try e.enterSecure(&ecu_mod.default_password, .none, 10_000);

        var out: std.ArrayList(u8) = .empty;
        defer out.deinit(testing.allocator);
        try e.readFlash(testing.allocator, &out, 0, null, .none);
        try testing.expectEqualSlices(u8, rig.stock, out.items);
    }
}

test "partial read with start and length" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    rig.st.cycle_power_after_probes = 5;
    try rig.ecu.enterSecure(&ecu_mod.default_password, .none, 10_000);
    var out: std.ArrayList(u8) = .empty;
    defer out.deinit(testing.allocator);
    try rig.ecu.readFlash(testing.allocator, &out, 0x100, 31, .none);
    try testing.expectEqualSlices(u8, rig.stock[0x100..][0..31], out.items);
}

test "wrong password does not give access" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    rig.st.cycle_power_after_probes = 5;
    try testing.expectError(
        error.SecurityAccessDenied,
        rig.ecu.enterSecure("NotThePassword", .none, 10_000),
    );
}

test "no ignition cycle times out" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    try testing.expectError(
        error.IgnitionTimeout,
        rig.ecu.enterSecure(&ecu_mod.default_password, .none, 2_000),
    );
}

test "flash every known model" {
    for (&models.all) |*m| {
        const rig = try Rig.create(m.short());
        defer rig.destroy();
        const image = try rig.tune();
        defer testing.allocator.free(image);

        try rig.ecu.flashImage(image, m.write_offset, .none);

        try testing.expectEqualSlices(u8, image, rig.flash);
        try testing.expect(!rig.sim.blank);
        try testing.expectEqual(@as(u8, 4), rig.sim.flash_count);
        try testing.expectEqual(image.len / 128, rig.sim.blocks_written);
        // The final block is marked by a zero "next" address.
        try testing.expectEqual(@as(u16, 0), rig.sim.last_next16);
        try testing.expectEqual(@as(u32, 0), rig.ecu.stats.unneeded_retries);

        // After a power cycle the ECU runs normally again.
        rig.sim.powerOff();
        rig.sim.powerOn();
        try testing.expectEqual(ecu_mod.State.ok, try rig.ecu.detectState());
    }
}

test "falls back to 64-byte blocks" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    rig.sim.accept_128 = false;
    const image = try rig.tune();
    defer testing.allocator.free(image);
    try rig.ecu.flashImage(image, rig.model.write_offset, .none);
    try testing.expectEqualSlices(u8, image, rig.flash);
    try testing.expectEqual(image.len / 64, rig.sim.blocks_written);
}

test "a lost acknowledgement is recovered by the retry" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    rig.sim.drop_ack_of_block = 17;
    const image = try rig.tune();
    defer testing.allocator.free(image);
    try rig.ecu.flashImage(image, rig.model.write_offset, .none);
    try testing.expectEqualSlices(u8, image, rig.flash);
    try testing.expectEqual(@as(u32, 1), rig.ecu.stats.unneeded_retries);
}

test "interrupted flash leaves a recoverable ECU" {
    const rig = try Rig.create("K0F-A01");
    defer rig.destroy();
    rig.sim.die_after_blocks = 40;
    const image = try rig.tune();
    defer testing.allocator.free(image);

    try testing.expectError(error.WriteNoResponse, rig.ecu.flashImage(image, rig.model.write_offset, .none));
    try testing.expect(rig.sim.blank);

    rig.sim.powerOn();
    try testing.expectEqual(ecu_mod.State.recover_old, try rig.ecu.detectState());
    try rig.ecu.flashImage(image, rig.model.write_offset, .none);
    try testing.expectEqualSlices(u8, image, rig.flash);
    try testing.expect(!rig.sim.blank);
}

test "resume when the ECU is already in flash mode" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    rig.sim.awake = true;
    rig.sim.mode = .flash;
    rig.sim.write_status = 0x10;
    const image = try rig.tune();
    defer testing.allocator.free(image);
    try testing.expectEqual(ecu_mod.State.flash, try rig.ecu.detectState());
    try rig.ecu.flashImage(image, rig.model.write_offset, .none);
    try testing.expectEqualSlices(u8, image, rig.flash);
}

test "an image with a bad checksum is refused by the ECU check" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    const image = try rig.tune();
    defer testing.allocator.free(image);
    image[0x1234] +%= 1;
    try testing.expectError(error.VerifyFailed, rig.ecu.flashImage(image, rig.model.write_offset, .none));
}

test "flash error status stops before erasing" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    rig.sim.awake = true;
    rig.sim.mode = .flash;
    rig.sim.write_status = 0xf0;
    const image = try rig.tune();
    defer testing.allocator.free(image);
    try testing.expectError(error.EcuFlashError, rig.ecu.flashImage(image, rig.model.write_offset, .none));
    try testing.expectEqualSlices(u8, rig.stock, rig.flash);
}

test "unlock without sensing the ignition" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    rig.ecu.expect_echo = false; // an adapter that hides its own echo
    rig.sim.echo = false;
    try rig.ecu.unlockAfterPowerOn(&ecu_mod.default_password);
    var out: std.ArrayList(u8) = .empty;
    defer out.deinit(testing.allocator);
    try rig.ecu.readFlash(testing.allocator, &out, 0, 64, .none);
    try testing.expectEqualSlices(u8, rig.stock[0..64], out.items);
}

test "bad sizes and offsets are refused before anything is sent" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    const image = try rig.tune();
    defer testing.allocator.free(image);
    try testing.expectError(error.BadImageSize, rig.ecu.flashImage(image[0..100], 0x4000, .none));
    try testing.expectError(error.BadImageSize, rig.ecu.flashImage(image[0..0], 0x4000, .none));
    try testing.expectError(error.BadOffset, rig.ecu.flashImage(image, 0x4001, .none));
    try testing.expectError(error.BadOffset, rig.ecu.flashImage(image, 0xFF0000, .none));
    try testing.expectEqual(@as(u32, 0), rig.ecu.stats.commands);
    try testing.expectEqualSlices(u8, rig.stock, rig.flash);
}

test "flashing is refused with the ignition off" {
    const rig = try Rig.create("K26-911");
    defer rig.destroy();
    rig.sim.powerOff();
    const image = try rig.tune();
    defer testing.allocator.free(image);
    try testing.expectError(error.EcuNotReady, rig.ecu.flashImage(image, 0x4000, .none));
}
