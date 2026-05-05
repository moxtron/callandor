const std = @import("std");
const microzig = @import("microzig");

const rpi = microzig.hal;
const time = rpi.time;

// Compile-time pin configuration
const pin_config = rpi.pins.GlobalConfiguration{
    .GPIO25 = .{
        .name = "led",
        .direction = .out,
    },
};

pub fn main() void {
    const pins = pin_config.apply();

    while (true) {
        pins.led.toggle();
        time.sleep_ms(250);
    }
}
