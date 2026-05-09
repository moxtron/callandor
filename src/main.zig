const std = @import("std");
const microzig = @import("microzig");
const servo = @import("servo.zig");
const pwmlib = @import("pwm.zig");
const esc = @import("esc.zig");

const build_options = @import("build_options");
const calibrate_mode = build_options.calibrate;


const rpi = microzig.hal;
const time = rpi.time;
const uart = rpi.uart;
const sleep = time.sleep_ms;

const Servo = servo.Servo;
const ServoConfig = servo.ServoConfig;
const ServoGroup = servo.ServoGroup;

pub const microzig_options: microzig.Options = .{
    .interrupts = .{
        .PWM_IRQ_WRAP = .{ .c = pwmlib.handler }
    },
    .logFn = uart.log,
};

/// Compile-time pin configuration
/// DO NOT CHANGE! (except for a really, really good reason)
const pin_config = rpi.pins.GlobalConfiguration{
    .GPIO0 = .{
        .name = "uart0_tx",
        .function = .UART0_TX
    },
    .GPIO4 = .{
        .name = "crsf_rx",
        .function = .UART0_RX
    },
    .GPIO5 = .{
        .name = "crsf_tx",
        .function = .UART1_TX
    },
    .GPIO1 = .{
        .name = "uart0_rx",
        .function = .UART1_RX
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

/// only to be used for debugging
fn setup_uart_logging() void {
    const uart0 = uart.instance.UART0;
    uart0.apply(.{
        .baud_rate = 115200,
        .clock_config = rpi.clock_config,
    });
    uart.init_logger(uart0);

    std.log.info("UART successfully set up!", .{});
}
fn setup_uart_crsf() uart.UART {
    const uart1 = uart.instance.UART1;
    uart1.apply(.{
        .baud_rate = 420_000,
        .clock_config = rpi.clock_config,
    });
    return uart1;
}

pub fn main() void {
    // setting up the PWM pins
    _ = pin_config.apply();

    // --------------------
    // # --- UART setup ---
    // --------------------
    setup_uart_logging(); // logging
    const crsf_uart = setup_uart_crsf(); // crsf
    _ = crsf_uart;
    while (true) {

    }
}
