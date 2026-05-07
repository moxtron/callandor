const std = @import("std");
const microzig = @import("microzig");

const rpi = microzig.hal;
const time = rpi.time;

// Compile-time pin configuration
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
    _ = pins;

    while (true) {
    }
}
