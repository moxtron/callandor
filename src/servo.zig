const std = @import("std");
const microzig = @import("microzig");
const pwmlib = @import("pwm.zig");      // `pwmlib` to avoid confusion with microzigs `pwm`

const rpi = microzig.hal;
const pwm = rpi.pwm;

const Pwm = pwm.Pwm;
const Channel = pwm.Channel;
const @"pwmlib.SliceIndex" = pwmlib.SliceIndex;

/// Settings for servo motor. Defines its range, the center point and if it is reversed.
pub const ServoConfig = struct {
    min_us:    u16 = 550,
    center_us: u16 = 1500,
    max_us:    u16 = 2450,
    reversed: bool = false, // mainly meant for the second aileron
};


/// Generic container for N servos. Broadcasts 'setPulse' and 'center' commands to all members.
pub fn ServoGroup(comptime N: usize) type {
    return struct {
        servos: [N]Servo,
        count: usize = 0,

        const Self = @This();

        /// Wraps a fixed-size array of pre-initialized `Servo` instances into a group.
        pub fn init(servos: [N]Servo) Self {
            return .{
                .servos = servos,
            };
        }

        /// Sends the same pulse width to every servo in the group.
        pub fn setPulse(self: Self, us: u16) void {
            for (self.servos) |servo| {
                servo.setPulse(us);
            }
        }
        /// Moves every servo in the group to its configured center position.
        pub fn center(self: Self) void {
            for (self.servos) |servo| {
                servo.center();
            }
        }
    };
}

/// Single servo bound to a PWM channel. Clamps all pulse widths to 'config' range and applies optional reversal.
pub const Servo = struct {
    pwm: pwm.Pwm,
    config: ServoConfig,
    slice: pwmlib.SliceIndex,
    channel: pwm.Channel,


    // --- Lifecycle ---

    /// Configures the PWM slice for this servo and centers it to 'config.center_us'.
    pub fn init(pwm_struct: pwm.Pwm, config: ServoConfig) Servo {
        // setup the pwm slice
        const slice = pwm_struct.slice();
        slice.set_clk_div(pwmlib.clk.div, pwmlib.clk.frac);
        slice.set_wrap(pwmlib.clk.wrap);
        slice.enable();
        // direct write for initial position
        pwm_struct.set_level(config.center_us);

        // prime the ISR buffer so it never applies a stale zero
        pwmlib.setLevel(pwm_struct.channel, @as(pwmlib.SliceIndex, @truncate(pwm_struct.slice_number)), config.center_us);
        return .{
            .pwm = pwm_struct,
            .config = config,
            .slice = @truncate(pwm_struct.slice_number),
            .channel = pwm_struct.channel,
        };

    }
    // --- Control ---

    /// Outputs a pulse of 'us' microseconds, clamped to [min_us, max_us]. Applies reversal if configured.
    pub fn setPulse(self: *const Servo, us: u16) void {
        // `clamp` assures the value is in the safe range
        const level: u16 = switch (self.config.reversed) {
            true  => (self.config.max_us + self.config.min_us) - std.math.clamp(us, self.config.min_us, self.config.max_us),
            false => std.math.clamp(us, self.config.min_us, self.config.max_us),
        };
        pwmlib.setLevel(self.channel, self.slice, level);

    }
    /// Centers the servo to the middle position, as specified in `self.config`
    pub fn center(self: *const Servo) void {
        pwmlib.setLevel(self.channel, self.slice, self.config.center_us);
    }
    // maybe later add a setNormalized(f: f32[-1..1]) for mixing
};
