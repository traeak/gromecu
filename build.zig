const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const root = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
        // Linux uses raw system calls and Windows uses kernel32 directly;
        // the other POSIX systems go through libc for termios and ioctl.
        .link_libc = switch (target.result.os.tag) {
            .linux, .windows => null,
            else => true,
        },
    });

    const exe = b.addExecutable(.{ .name = "gromecu", .root_module = root });
    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    run_cmd.addPassthruArgs();
    b.step("run", "Run gromecu").dependOn(&run_cmd.step);

    const tests = b.addTest(.{ .root_module = root });
    b.step("test", "Run the unit and simulator tests").dependOn(&b.addRunArtifact(tests).step);
}
