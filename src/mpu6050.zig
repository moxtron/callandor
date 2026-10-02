pub const MpuData = extern struct {
    accel_x: i32,
    accel_y: i32,
    accel_z: i32,
    gyro_x: i32,
    gyro_y: i32,
    gyro_z: i32,
};

pub extern fn mpu6050_init() bool;
pub extern fn mpu6050_read(data: *MpuData) bool;
pub extern fn calibrateBias(data: *MpuData) bool;
pub extern fn filter(data: *MpuData, bias_values: MpuData, pitch_angle: *f32, roll_angle: *f32) bool;
