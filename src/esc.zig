const std = @import("std");
const microzig = @import("microzig");
const pwmlib = @import("pwm.zig");

const rpi = microzig.hal;
const pwm = rpi.pwm;

/// Comptime config for the ESC controller
pub const EscConfig = struct {
    min_us: u16 = 1100, // Ro-Control V2 lower limit
    max_us: u16 = 1940, // Ro-Control V2 lower limit
};

/// State of the ESC controller
pub const EscState = enum {
    uninitialized,
    arming,
    armed,
    disarmed,
};

const Esc = struct {
    pwm: pwm.Pwm,
    config: EscConfig,
    slice: pwmlib.SliceIndex,
    channel: pwm.Channel,
    state: EscState,

    /// This has to be the very first function call in `main()`.
    /// Immediately outputs minimum throttle so the ESC doesn't receive a floating signal.
    pub fn init(raw_pwm: pwm.Pwm, config: EscConfig) void {
        // configure the PWM slice
        const slice = raw_pwm.slice();
        slice.set_clk_div(pwmlib.clk.div, pwmlib.clk.frac);
        slice.set_wrap(pwmlib.clk.wrap);
        slice.enable();

        // direct write with min. throttle. wins the race against ESC startup.
        raw_pwm.set_level(config.min_us);

        // prime the ISR buffer so the first wrap iterrupt isn't filled with zero
        pwmlib.setLevel(
            raw_pwm.channel,
            @as(pwmlib.SliceIndex, @truncate(raw_pwm.slice_number)),
            config.min_us
        );

        return .{
            .pwm = raw_pwm,
            .config = config,
            .slice = @truncate(raw_pwm.slice_number),
            .channel = raw_pwm.channel,
            .state = .arming,
        };
    }

    /// Blocking arming sequence. Call once at boot, right after `init()`.
    /// Waits for the ESC startup + cell count + ready beeps. (~5s total)
    pub fn arm(self: *Esc) void {
        // `init()` should have already set the minimum level in PWM, but we are defensive here.
        writeLevel(self.config, self.config.min_us);
        rpi.time.sleep_ms(5000);
        self.state = .armed;
    }

    /// One-time throttle range calibration. Only flash this once when setting up a new ESC.
    /// After a long confirmation beep reflash regular firmware.
    /// TODO: add `zig build -Dcalibrate` build option
    pub fn calibrate(self: *Esc) void {
        // max. throttle needs to be on the wire right after power on.
        // `init()` was called right before, so here we override it.
        writeLevel(self.config, self.config.max_us);
        rpi.time.sleep_ms(3000); // ESC powers on, beeps "B- B-" at ~2s

        // drop to min. within 5s of the "B- B-"
        writeLevel(self.config, self.config.min_us);
        rpi.time.sleep_ms(5000); // cell-count beeps + long confirmation beep

        // set into normal armed state
        self.state = .armed;
    }

    /// Set throttle in microseconds. Clamped to [min_us ... max_us]
    /// Only works in `armed` state.
    pub fn setThrottle(self: Esc, us: u16) void {
        if (self.state != .armed) return;
        writeLevel(self.config, us);
    }

    /// Cut throttle and mark as disarmed. Requires rearming to use again.
    /// Safety feature
    pub fn disarm(self: *Esc) void {
        writeLevel(self.config, self.config.min_us);
        self.state = .disarmed;
    }

    // --- Utility Functions ---

    /// Write raw microsecond values directly to the PWM buffer, no state checks.
    /// This is the only place touching pwmlib.setLevel for the ESC.
    fn writeLevel(self: Esc, us: u16) void {
        const level = std.math.clamp(us, self.config.min_us, self.config.max_us);
        pwmlib.setLevel(self.config.channel, self.config.slice, level);
    }

    pub fn isArmed(self: Esc) bool {
        return self.state == .armed;
    }

    pub fn getState(self: Esc) EscState {
        return self.state;
    }
};
