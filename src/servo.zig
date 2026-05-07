const std = @import("std");
const microzig = @import("microzig");
const pwmlib = @import("pwm.zig");      // `pwmlib` to avoid confusion with microzigs `pwm`

const rpi = microzig.hal;
const pwm = rpi.pwm;

const Pwm = pwm.Pwm;
const Channel = pwm.Channel;

/// servo settings
pub const ServoConfig = struct {
    min_us:    u16 = 1000,
    center_us: u16 = 1500,
    max_us:    u16 = 2000,
    reversed: bool = false, // mainly meant for the second aileron
};

// TODO: add ISR interrupts to only overwrite the PWM pulse width if the cycle is over
/// creates a Servo struct with bindings to a specific PWM channel
pub const Servo = struct {
    pwm: pwm.Pwm,
    config: ServoConfig,
    slice: u3,
    channel: pwm.Channel,


    // sets up the PWM slice and centers the servo
    pub fn init(pwm_struct: pwm.Pwm, config: ServoConfig) Servo {
        // setup the pwm slice
        const slice = pwm_struct.slice();
        slice.set_clk_div(pwmlib.div, pwmlib.frac);
        slice.set_wrap(pwmlib.wrap);
        slice.enable();
        // direct write for initial position
        pwm_struct.set_level(config.center_us);

        // prime the ISR buffer so it never applies a stale zero
        pwmlib.setLevel(pwm_struct.channel, @as(u3, @truncate(pwm_struct.slice_number)), config.center_us);
        return .{
            .pwm = pwm_struct,
            .config = config,
            .slice = @truncate(pwm_struct.slice_number),
            .channel = pwm_struct.channel,
        };

    }
    pub fn setPulse(self: *const Servo, us: u16) void {
        // `clamp` assures the value is in the safe range
        const level: u16 = switch (self.config.reversed) {
            true  => 3000 - std.math.clamp(us, self.config.min_us, self.config.max_us),
            false => std.math.clamp(us, self.config.min_us, self.config.max_us),
        };
        pwmlib.setLevel(self.channel, self.slice, level);

    }
    pub fn center(self: *const Servo) void {
        pwmlib.setLevel(self.config.center_us);
    }
    // maybe later add a setNormalized(f: f32[-1..1]) for mixing
};
