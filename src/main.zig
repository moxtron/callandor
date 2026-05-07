const std = @import("std");
const microzig = @import("microzig");
const servo = @import("servo.zig");
const pwmlib = @import("pwm.zig");



const rpi = microzig.hal;
const time = rpi.time;

const Servo = servo.Servo;
const ServoConfig = servo.ServoConfig;

pub const microzig_options: microzig.Options = .{
    .interrupts = .{
        .PWM_IRQ_WRAP = .{ .c = pwmlib.handler }
    },
};

fn irq_setup() void {
    // tell the ISR which slices it manages
    pwmlib.registerSlice(0); // aileron_left & aileron_right
    pwmlib.registerSlice(1); // rudder & elevator
    pwmlib.registerSlice(3); // ESC

    // enable PWM wrap interrupt for each managed slice
    pwmlib.enableSliceIrq(0);
    pwmlib.enableSliceIrq(1);
    pwmlib.enableSliceIrq(3);

    // unmask PWM_IRQ_WRAP to go live
    pwmlib.enableCpuIrq();

}

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

    irq_setup();

    while (true) {
        var i: u16 = 1000;
        while (i < 2000) : (i += 1) {
            aileron_left.setPulse(i);
            time.sleep_ms(9);
        }
    }
}
