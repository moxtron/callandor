const std = @import("std");
const microzig = @import("microzig");
const servo = @import("servo.zig");
const pwmlib = @import("pwm.zig");



const rpi = microzig.hal;
const time = rpi.time;
const uart = rpi.uart;

const Servo = servo.Servo;
const ServoConfig = servo.ServoConfig;

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
    .GPIO1 = .{
        .name = "uart0_rx",
        .function = .UART0_RX
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
fn setup_uart0() void {
    const uart0 = uart.instance.UART0;
    uart0.apply(.{
        .baud_rate = 115200,
        .clock_config = rpi.clock_config,
    });
    uart.init_logger(uart0);

    std.log.info("UART successfully set up!", .{});
}



pub fn main() void {
    setup_uart0();
    std.log.info("main() starting...",.{});
    // setting up the PWM pins
    const pins = pin_config.apply();
    const aileron_left = Servo.init(pins.aileron_left, .{});
    pwmlib.initFromPinConfig(pin_config);

    while (true) {
        aileron_left.setPulse(1000);
        time.sleep_ms(333);
        aileron_left.center();
        time.sleep_ms(333);
        aileron_left.setPulse(2000);
        time.sleep_ms(333);
        std.log.info("Fired: {}", .{pwmlib.fire_counter});
    }
}
