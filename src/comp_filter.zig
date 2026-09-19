const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const mpu = @import("mpu6050.zig");
const gyro_scale: f64 = 500.0 / 32768.0;
const accel_scale: f64 = 4.0 / 32768.0;
const rad_to_deg: f64 = 180.0 / std.math.pi;
const Bias_Values_Buf = struct {
    accel_x: i32,
    accel_y: i32,
    accel_z: i32,
    gyro_x: i32,
    gyro_y: i32,
    gyro_z: i32,
};
pub const compFilterObj = struct {
    mpu_data: mpu.MpuData,
    bias_values: mpu.MpuData,
    pitch_angle: f64,
    roll_angle: f64,
    last_filtered_time: u64,

    fn getTimeUs() u64 {
        return time.get_time_since_boot().to_us();
    }
    pub fn calibrateBias(self: *compFilterObj) bool {
        const calibration_start = getTimeUs();
        var bias_values_buf: Bias_Values_Buf = std.mem.zeroes(Bias_Values_Buf);
        var last_sample_time = calibration_start;
        var counter: u16 = 0;
        var now: u64 = undefined;
        while (true) {
            now = getTimeUs();
            if (now - last_sample_time >= 1000) {
                if (mpu.mpu6050_read(&self.mpu_data)) {
                    last_sample_time = now;
                    counter += 1;
                    bias_values_buf.accel_x += @as(i32, @intCast(self.mpu_data.accel_x));
                    bias_values_buf.accel_y += @as(i32, @intCast(self.mpu_data.accel_y));
                    bias_values_buf.accel_z += @as(i32, @intCast(self.mpu_data.accel_z));
                    bias_values_buf.gyro_x += @as(i32, @intCast(self.mpu_data.gyro_x));
                    bias_values_buf.gyro_y += @as(i32, @intCast(self.mpu_data.gyro_y));
                    bias_values_buf.gyro_z += @as(i32, @intCast(self.mpu_data.gyro_z));
                } else {
                    return false;
                }
            }
            if ((now - calibration_start) > 1000000) {
                break;
            }
        }
        // return this for init
        if (counter == 0) return false;

        self.bias_values.accel_x = @as(i16, @intCast(@divTrunc(bias_values_buf.accel_x, counter)));
        self.bias_values.accel_y = @as(i16, @intCast(@divTrunc(bias_values_buf.accel_y, counter)));
        self.bias_values.accel_z = @as(i16, @intCast(@divTrunc(bias_values_buf.accel_z, counter)));
        self.bias_values.gyro_x = @as(i16, @intCast(@divTrunc(bias_values_buf.gyro_x, counter)));
        self.bias_values.gyro_y = @as(i16, @intCast(@divTrunc(bias_values_buf.gyro_y, counter)));
        self.bias_values.gyro_z = @as(i16, @intCast(@divTrunc(bias_values_buf.gyro_z, counter)));
        return true;
    }

    pub fn filter(self: *compFilterObj) bool {
        const now = getTimeUs();
        const dt: f64 = @as(f64, (@floatFromInt((now - self.last_filtered_time)))) / 1000000;
        const corrected_ax = @as(f64, @floatFromInt(self.mpu_data.accel_x - self.bias_values.accel_x));
        const corrected_ay = @as(f64, @floatFromInt(self.mpu_data.accel_y - self.bias_values.accel_y));
        const corrected_gx = @as(f64, @floatFromInt(self.mpu_data.gyro_x - self.bias_values.gyro_x));
        const corrected_gy = @as(f64, @floatFromInt(self.mpu_data.gyro_y - self.bias_values.gyro_y));
        const corrected_gz = @as(f64, @floatFromInt(self.mpu_data.gyro_z - self.bias_values.gyro_z));
        const accel_z_offset: i16 = 8192 - self.bias_values.accel_z;
        const corrected_az: i16 = self.mpu_data.accel_z + accel_z_offset;
        const az_g: f64 = @as(f64, @floatFromInt(corrected_az)) / 8192.0;
        self.last_filtered_time = now;
        if (az_g < 0.01 and az_g > -0.01) return false;
        const gyro_roll: f64 = self.roll_angle + (corrected_gx * gyro_scale) * dt;
        const gyro_pitch: f64 = self.pitch_angle + (corrected_gy * gyro_scale) * dt;
        const accel_roll: f64 = std.math.atan2((corrected_ax * accel_scale), az_g) * rad_to_deg;
        const accel_pitch: f64 = std.math.atan2((corrected_ay * accel_scale), az_g) * rad_to_deg;
        self.roll_angle = 0.98 * gyro_roll + 0.02 * accel_roll;
        self.pitch_angle = 0.98 * gyro_pitch + 0.02 * accel_pitch;

        self.mpu_data.accel_x = @as(i16, @intFromFloat(corrected_ax));
        self.mpu_data.accel_y = @as(i16, @intFromFloat(corrected_ay));
        self.mpu_data.accel_z = corrected_az;
        self.mpu_data.gyro_x = @as(i16, @intFromFloat(corrected_gx));
        self.mpu_data.gyro_y = @as(i16, @intFromFloat(corrected_gy));
        self.mpu_data.gyro_z = @as(i16, @intFromFloat(corrected_gz));
        return true;
    }

    // call this function before first filter call and after calibrateBias
    pub fn initLastFilteredTime(self: *compFilterObj) void {
        self.last_filtered_time = getTimeUs();
    }

    pub fn init() compFilterObj {
        return .{
            .bias_values = std.mem.zeroes(mpu.MpuData),
            .last_filtered_time = 0,
            .mpu_data = std.mem.zeroes(mpu.MpuData),
            .pitch_angle = 0,
            .roll_angle = 0,
        };
    }
};
