const std = @import("std");
const microzig = @import("microzig");
const servo = @import("servo.zig");
const pwmlib = @import("pwm.zig");
const esc = @import("esc.zig");
const crsf = @import("crsf.zig");
const mpu = @import("mpu6050.zig");
comptime { _ = @import("bindings.zig"); }
// const mpu = @cImport({
//     @cInclude("mpu6050.h");
// });


const build_options = @import("build_options");
const calibrate_mode = build_options.calibrate;


const rpi = microzig.hal;
const time = rpi.time;
const uart = rpi.uart;
const i2c = rpi.i2c;
const sleep = time.sleep_ms;
const log = std.log;

const Servo = servo.Servo;
const ServoConfig = servo.ServoConfig;
const ServoGroup = servo.ServoGroup;

pub const microzig_options: microzig.Options = .{
    .interrupts = .{
        .PWM_IRQ_WRAP = .{ .c = pwmlib.handler }
    },
    .logFn = uart.log,
};

/// Compile-time pin assignment for UART and PWM peripherals.
const pin_config = rpi.pins.GlobalConfiguration{
    .GPIO0 = .{
        .name = "uart0_tx",
        .function = .UART0_TX
    },
    .GPIO1 = .{
        .name = "uart0_rx",
        .function = .UART0_RX
    },
    .GPIO8 = .{
        .name = "crsf_rx",
        .function = .UART1_TX
    },
    .GPIO9 = .{
        .name = "crsf_tx",
        .function = .UART1_RX
    },
    .GPIO14 = .{
        .name = "i2c_sda",
        .function = .I2C1_SDA,
        //.pull = .up,
    },
    .GPIO15 = .{
        .name = "i2c_scl",
        .function = .I2C1_SCL,
        //.pull = .up,
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

/// Configures UART0 at 115200 baud and registers it as the 'std.log' backend. Debug use only.
fn setup_uart_logging() void {
    const uart0 = uart.instance.UART0;
    uart0.apply(.{
        .baud_rate = 115200,
        .clock_config = rpi.clock_config,
    });
    uart.init_logger(uart0);

    std.log.info("UART successfully set up!", .{});
}
/// Configures UART1 at 420_000 baud for ELRS/CRSF receiver communication.
fn setup_uart_crsf() uart.UART {
    const uart1 = uart.instance.UART1;
    uart1.apply(.{
        .baud_rate = 420_000,
        .clock_config = rpi.clock_config,
    });
    return uart1;
}
/// Configures the I2C1 interface for the MPU6050 and returns the I2C instance
fn setup_i2c_imu() void {
    const instance = i2c.instance.I2C1;
    instance.apply(.{
        .clock_config = rpi.clock_config,
        .baud_rate = 400_000,
        .repeated_start = true,
    });
}

pub fn main() void {
    // setting up the PWM pins
    _ = pin_config.apply();

    // uart & crsf setup
    setup_uart_logging(); // logging
    const crsf_uart = setup_uart_crsf();
    var fsm = crsf.CrsfFsm{};
    // setup I2C
    setup_i2c_imu();

    // initialize IMU
    if (!mpu.mpu6050_init()) {
        std.log.err("MPU6050 initialization failed...", .{});
    }
    // container for IMU data
    var imu_data: mpu.MpuData = undefined;

    while (true) {
        // drain all available bytes into the FSM
        while (true) {
            const received = crsf_uart.read_word() catch blk: {
                // log the error, clear it and keep going
                //std.log.warn("UART1_RX Error: {}", .{err});
                crsf_uart.clear_errors();
                break :blk null;
            };
            const byte = received orelse break;
            fsm.feed(byte);
        }

        // consume one decoded frame per loop
        switch (fsm.takeFrame()) {
            .none => {},

            .rc_channels => |ch| {
                std.log.info(
                    "CH: {d} {d} {d} {d} | {d} {d} {d} {d}",
                    .{ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7]}
                );
            },
            .link_stats => |ls| {
                std.log.info("LQ: {d}%  RSSI1: -{d}dBm  SNR: {d}dB", .{
                    ls.uplink_link_quality,
                    ls.uplink_rssi_1,
                    ls.uplink_snr,
                });
            },

        }
        if (mpu.mpu6050_read(&imu_data)) {
            std.log.info(
                "ax:{d} ay:{d} az:{d} gx:{d} gy:{d} gz:{d}", .{
                    imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                    imu_data.gyro_x,  imu_data.gyro_y,  imu_data.gyro_z,
                    });
        }
    }
}
