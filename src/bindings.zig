const microzig = @import("microzig");

const rpi = microzig.hal;
const time = rpi.time;
const i2c = rpi.i2c;
const std = @import("std");

export fn c_atan2f(y: f32, x: f32) callconv(.c) f32 {
    return std.math.atan2(y, x);
}
/// sleep_ms replacement for pico sdk
export fn sleep_ms(us: u32) callconv(.c) void {
    time.sleep_ms(us);
}

// --- MPU6050 I2C ---

/// stores the i2c instance Zig sets up. C code reads it from here.
var imu_i2c = i2c.instance.I2C1;

/// Called from `main.zig` after setup_i2c_imu() to bind the instance.
/// Accepts `@enumFromInt(imu_i2c)`
pub fn imu_i2c_set_instance(instance: i2c.I2C) void {
    imu_i2c = instance;
}

/// Plain write, used for all config register writes in mpu6050_init().
/// Returns bytes written, or -1 on error.
export fn mpu_i2c_write(addr: u8, data: [*]const u8, len: usize) callconv(.c) i32 {
    const address: i2c.Address = @enumFromInt(addr);
    imu_i2c.write_blocking(address, data[0..len], null) catch return -1;
    return @intCast(len);
}

/// Write-then-read in a single repeated-start transaction.
/// Used by mpu6050_read() to send the register pointer, then burst-read 14 bytes.
/// Returns bytes read, or -1 on error.
export fn mpu_i2c_write_then_read(
    addr: u8,
    write_data: [*]const u8,
    write_len: usize,
    read_data: [*]u8,
    read_len: usize,
) callconv(.c) i32 {
    const address: i2c.Address = @enumFromInt(addr);
    imu_i2c.write_then_read_blocking(address, write_data[0..write_len], read_data[0..read_len], null) catch return -1;
    return @intCast(read_len);
}
export fn printNum(x: i32) callconv(.c) void {
    std.log.info("{d}", .{x});
}
export fn print(str: [*:0]u8) callconv(.c) void {
    std.log.info("{s}", .{str});
}
