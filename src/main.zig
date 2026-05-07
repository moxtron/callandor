const std = @import("std");
const microzig = @import("microzig");
const servo = @import("servo.zig");

const rpi = microzig.hal;
const time = rpi.time;

const Servo = servo.Servo;
const ServoConfig = servo.ServoConfig;

// Compile-time pin configuration
// DO NOT CHANGE! (except for a really, really good reason)
const pin_config = rpi.pins.GlobalConfiguration{
    .GPIO22 = .{
        .name = "esc",
        .direction = .out,
        .function = .PWM3_A,
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
};

pub fn main() void {
    const pins = pin_config.apply();
    const aileron_left = Servo.init(pins.aileron_left, .{});
    while (true) {
        aileron_left.setPulse(1000);
        time.sleep_ms(500);
        aileron_left.center();
        time.sleep_ms(500);
        aileron_left.setPulse(2000);
        time.sleep_ms(500);
        aileron_left.center();
        time.sleep_ms(500);
    }
}
