const std = @import("std");
const microzig = @import("microzig");
const servo = @import("servo.zig");
const pwmlib = @import("pwm.zig");



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

    const aileron_left  = Servo.init(pins.aileron_left, .{});
    const aileron_right = Servo.init(pins.aileron_right, .{});
    const elevator      = Servo.init(pins.elevator, .{});
    const rudder        = Servo.init(pins.rudder, .{});
    const esc           = Servo.init(pins.esc, .{});

    const front = ServoGroup(2).init(.{
        aileron_left,
        aileron_right,
    });
    const back = ServoGroup(2).init(.{
        elevator,
        rudder,
    });

    pwmlib.initFromPinConfig(pin_config);

    const level = ServoConfig{};
    while (true) {

        front.setPulse(level.min_us);
        sleep(500);
        front.setPulse(level.max_us);
        sleep(500);
        front.center();
        sleep(2000);

        back.setPulse(level.min_us);
        sleep(500);
        back.setPulse(level.max_us);
        sleep(500);
        back.center();
        sleep(2000);

        esc.setPulse(level.min_us);
        sleep(500);
        esc.setPulse(level.max_us);
        sleep(500);
        esc.center();
        sleep(2000);
    }
}
