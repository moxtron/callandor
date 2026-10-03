const std = @import("std");
const microzig = @import("microzig");
const servo = @import("servo.zig");
const pwmlib = @import("pwm.zig");
const esc = @import("esc.zig");
const crsf = @import("crsf.zig");
const mixer = @import("mixer.zig");
const debug = @import("debug.zig");
const mpu = @import("mpu6050.zig");
const mpu_real = @import("comp_filter.zig");
const pid = @import("pid.zig");
comptime {
    _ = @import("bindings.zig");
}
// --- Compile-time build options ---
const build_options = @import("build_options");
const calibrate_mode = build_options.calibrate;
const debug_mode = build_options.debug; // before 57.5KiB
const ch_roll: u8 = @intFromEnum(mixer.RcChannelIndex(.ailerons));
const ch_pitch: u8 = @intFromEnum(mixer.RcChannelIndex(.elevator));
const gyro_scale: f32 = 500.0 / 32768.0;

// --- Aliases ---
const rpi = microzig.hal;
const time = rpi.time;
const uart = rpi.uart;

const Servo = servo.Servo;
const ServoConfig = servo.ServoConfig;
const ServoGroup = servo.ServoGroup;

// --- Configurations ---
pub const microzig_options: microzig.Options = .{
    .interrupts = .{ .PWM_IRQ_WRAP = .{ .c = pwmlib.handler } },
    .logFn = uart.log,
};

/// Compile-time pin assignment for UART and PWM peripherals.
const pin_config = rpi.pins.GlobalConfiguration{
    .GPIO0 = .{ .name = "uart0_tx", .function = .UART0_TX },
    .GPIO1 = .{ .name = "uart0_rx", .function = .UART0_RX },
    .GPIO8 = .{
        .name = "elrs_tx", // connects to RX on ELRS receiver
        .function = .UART1_TX,
    },
    .GPIO9 = .{
        .name = "elrs_rx", // connects to TX on ELRS receiver
        .function = .UART1_RX,
    },
    .GPIO16 = .{
        .name = "aileron_left",
        .direction = .out,
        .function = .PWM0_A,
    },
    .GPIO17 = .{
        .name = "aileron_right",
        .direction = .out,
        .function = .PWM0_B,
    },
    .GPIO18 = .{
        .name = "elevator",
        .direction = .out,
        .function = .PWM1_A,
    },
    .GPIO19 = .{
        .name = "rudder",
        .direction = .out,
        .function = .PWM1_B,
    },
    .GPIO22 = .{
        .name = "esc",
        .direction = .out,
        .function = .PWM3_A,
    },
};

// --- Hardware Initialization ---

/// Configures UART1 at 420_000 baud for ELRS/CRSF receiver communication.
fn setup_uart_crsf() uart.UART {
    const uart1 = uart.instance.UART1;
    uart1.apply(.{
        .baud_rate = 420_000,
        .clock_config = rpi.clock_config,
    });
    return uart1;
}

pub fn main() void {

    // --- Setup ---
    const pins = pin_config.apply();

    // initialize ESC
    var motor = esc.Esc.init(pins.esc, .{}, calibrate_mode);
    // initialize servos
    const aileron_left = Servo.init(pins.aileron_left, .{ .servo_type = .aileron });
    const aileron_right = Servo.init(pins.aileron_right, .{ .servo_type = .aileron });
    const elevator = Servo.init(pins.elevator, .{ .servo_type = .elevator });
    const rudder = Servo.init(pins.rudder, .{ .servo_type = .rudder });

    // initialize interrupts
    pwmlib.init(pin_config);

    // arm / calibrate ESC
    if (calibrate_mode) motor.calibrate() else motor.arm();

    // debug mode setup
    if (comptime debug_mode) debug.setup();
    var debug_state = if (comptime debug_mode) debug.State{};

    // uart & crsf setup
    //setup_uart_logging(); // logging

    const uart_crsf = setup_uart_crsf();
    var fsm = crsf.CrsfFsm{};

    // filled with safe values until there is real data available
    var channels: [16]u16 = @splat(172);
    var link_stats: crsf.LinkStats = undefined;

    var failsafe: bool = false; // failsafe flag
    // initialize IMU
    if (!mpu.mpu6050_init()) {
        std.log.err("MPU6050 initialization failed...", .{});
    }
    // container for IMU data
    var CompFilterObj: mpu_real.compFilterObj = mpu_real.compFilterObj.init();
    // calibrateBias() takes one second!!!
    if (CompFilterObj.calibrateBias()) {} else {
        std.log.err("BIAS calibration failed...", .{});
    }
    CompFilterObj.initLastFilteredTime();

    // PID init
    const roll_angle = pid.AngleLoop{ .kp_angle = 4.0, .max_angle = 45.0, .max_rate = 120.0 };
    var roll_rate = pid.RatePid{ .kp = 0.003, .ki = 0.003, .kd = 0.0, .i_limit = 0.2, .out_limit = 0.6, .d_alpha = 0.2 };

    const pitch_angle = pid.AngleLoop{ .kp_angle = 4.0, .max_angle = 25.0, .max_rate = 60.0 };
    var pitch_rate = pid.RatePid{ .kp = 0.003, .ki = 0.003, .kd = 0.0, .i_limit = 0.2, .out_limit = 0.6, .d_alpha = 0.2 };

    const stick_cal = pid.StickCal{};

    var before = time.get_time_since_boot();
    var last_rc_frame = before;

    // debug counters
    // var uart_errors: usize = 0;
    // var rc_frames: usize = 0;
    // var ls_frames: usize = 0;

    // --- MAIN CONTROL LOOP ---
    while (true) {
        // drain all available bytes into the FSM
        drain: while (true) {
            const byte = uart_crsf.read_word() catch {
                // log the error, clear it and keep going
                //std.log.warn("UART1_RX Error: {}", .{err});
                uart_crsf.clear_errors();
                if (comptime debug_mode) debug_state.countUartError();
                continue :drain;
            } orelse break :drain; // FIFO empty -> done draining
            fsm.feed(byte);
        }

        // --- CRSF consumers ---
        // only poll each type of CRSF frame once per main loop iteration.

        if (fsm.takeRcChannels()) |ch| {
            if (comptime debug_mode) debug_state.countControls(ch);
            channels = ch;
            last_rc_frame = time.get_time_since_boot();
        }
        if (fsm.takeLinkStats()) |ls| {
            if (comptime debug_mode) debug_state.countLinkStats(ls);
            link_stats = ls;
        }

        // TODO: PID
        const mpu_log_now = time.get_time_since_boot().to_us();
        const now = time.get_time_since_boot();
        const dt_dur = now.diff(before);
        before = now;
        const dt: f32 = std.math.clamp(@as(f32, @floatFromInt(dt_dur.to_us())) / 1_000_000.0, 0.0001, 0.02);
        if (!mpu.mpu6050_read(&CompFilterObj.mpu_data)) {
            _ = 0;
        }

        if (!CompFilterObj.filter()) {
            _ = 0;
        }
        // Before real flight have to check if reads and filters worked
        // const imu_ok = mpu.mpu6050_read(&CompFilterObj.mpu_data) and CompFilterObj.filter();
        if ((mpu_log_now - CompFilterObj.last_filtered_time) > 1000000) {
            std.log.info("ax:{d} ay:{d} az:{d} gx:{d} gy:{d} gz:{d}\n", .{
                CompFilterObj.mpu_data.accel_x, CompFilterObj.mpu_data.accel_y, CompFilterObj.mpu_data.accel_z,
                CompFilterObj.mpu_data.gyro_x,  CompFilterObj.mpu_data.gyro_y,  CompFilterObj.mpu_data.gyro_z,
            });
            std.log.info("ax:{d} ay:{d} az:{d} gx:{d} gy:{d} gz:{d} BIAS:{d}, PITCH_ANGLE: {d}, ROLL_ANGLE: {d}\n", .{
                CompFilterObj.mpu_data.accel_x,   CompFilterObj.mpu_data.accel_y, CompFilterObj.mpu_data.accel_z,
                CompFilterObj.mpu_data.gyro_x,    CompFilterObj.mpu_data.gyro_y,  CompFilterObj.mpu_data.gyro_z,
                CompFilterObj.bias_values.gyro_z, CompFilterObj.pitch_angle,      CompFilterObj.roll_angle,
            });
        }
        // PID loop
        var ctrl = channels;

        const gyro_roll = @as(f32, @floatFromInt(CompFilterObj.mpu_data.gyro_x)) * gyro_scale;
        const gyro_pitch = @as(f32, @floatFromInt(CompFilterObj.mpu_data.gyro_y)) * gyro_scale;

        const stick_roll = stick_cal.normalize(channels[ch_roll]);
        const stick_pitch = stick_cal.normalize(channels[ch_pitch]);

        const roll_sp = roll_angle.rateSetPoint(stick_roll, CompFilterObj.roll_angle);
        const roll_out = roll_rate.update(roll_sp, gyro_roll, dt, true);

        const pitch_sp = pitch_angle.rateSetPoint(stick_pitch, CompFilterObj.pitch_angle);
        const pitch_out = pitch_rate.update(pitch_sp, gyro_pitch, dt, true);

        ctrl[ch_roll] = stick_cal.denormalize(roll_out);
        ctrl[ch_pitch] = stick_cal.denormalize(pitch_out);

        // TODO: When failsafe = active turn off PID, channel for PID active off,
        // handling imu errors, integrate bool in update needs handling
        // --- Mixer ---

        failsafe = now.diff(last_rc_frame).to_us() > 500_000; // sets to true if the last rc frame was received more than 0.5s ago
        mixer.mix(ctrl, motor, .{ aileron_left, aileron_right, elevator, rudder }, failsafe);

        if (comptime debug_mode) debug_state.ticker(now, failsafe);
    }
}
mpu.mpu6050_read(&CompFilterObj.mpu_data)) {
            _ = 0;
