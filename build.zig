const std = @import("std");
const microzig = @import("microzig");

const MicroBuild = microzig.MicroBuild(.{
    .rp2xxx = true,
});

pub fn build(b: *std.Build) void {
    const mz_dep = b.dependency("microzig", .{});
    const mb = MicroBuild.init(b, mz_dep) orelse return;

    // --- building the firmware ---
    const firmware = mb.add_firmware(.{
        .name = "callandor",
        .target = mb.ports.rp2xxx.boards.raspberrypi.pico,
        .optimize = .ReleaseSafe,
        .root_source_file = b.path("src/main.zig"),
    });

    // --- create binaries ---
    mb.install_firmware(firmware, .{}); // uf2
    mb.install_firmware(firmware, .{ .format = .elf });
}
