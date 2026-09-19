const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const mpu = @import("mpu6050.zig");


pub const Mpu_data = struct {
    accel_x: i16,
    accel_y: i16,
    accel_z: i16,
    gyro_x: i16,
    gyro_y: i16,
    gyro_z: i16,
};
const Bias_Values = struct {
    accel_x: i32,
    accel_y: i32,
    accel_z: i32,
    gyro_x: i32,
    gyro_y: i32,
    gyro_z: i32,
};
pub const compFilterObj = struct {
    mpu_data: Mpu_data,
    bias_values: Bias_Values,
    pitch_angle: f64,
    roll_angle: f64,

    fn getTimeUs() u32 {
        return time.get_time_since_boot().to_us();
    }
    pub fn calibrateBias(self: *compFilterObj) bool {
        const calibration_start = getTimeUs();
        var last_sample_time = calibration_start;
        var counter: u16 = 0;

        while(true){
            var now : u32 = getTimeUs();
            if(now - last_sample_time >= 1000)
        }
    }
};
