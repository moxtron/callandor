const std = @import("std");
const microzig = @import("microzig");

const MicroBuild = microzig.MicroBuild(.{
    .rp2xxx = true,
});

pub fn build(b: *std.Build) void {
    const mz_dep = b.dependency("microzig", .{});
    const mb = MicroBuild.init(b, mz_dep) orelse return;

    // --- build options ---
    // - calibrate -
    const calibrate = b.option(bool, "calibrate", "Run ESC throttle range calibration on boot") orelse false;
    const options = b.addOptions();
    options.addOption(bool, "calibrate", calibrate);

    // --- building the firmware ---
    const firmware = mb.add_firmware(.{
        .name = "callandor",
        .target = mb.ports.rp2xxx.boards.raspberrypi.pico,
        .optimize = .ReleaseSafe,
        .root_source_file = b.path("src/main.zig"),
    });

    // --- apply options ---
    firmware.add_app_import("build_options", options.createModule(), .{});

    // --- create binaries ---
    mb.install_firmware(firmware, .{}); // uf2
    mb.install_firmware(firmware, .{ .format = .elf });
}
