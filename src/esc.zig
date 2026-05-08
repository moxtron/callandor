const std = @import("std");
const microzig = @import("microzig");
const pwmlib = @import("pwm.zig");

const rpi = microzig.hal;

/// Comptime config for the ESC controller
pub const EscConfig = struct {
    slice: pwmlib.SliceIndex,
    channel: rpi.pwm.Channel,
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

var state: EscState = .uninitialized;

// --- Private ---

/// Write raw microsecond values directly to the PWM buffer, no state checks.
/// This is the only place touching pwmlib.setLevel for the ESC.
fn writeLevel(comptime config: EscConfig, us: u16) void {
    const level = std.math.clamp(us, config.min_us, config.max_us);
    pwmlib.setLevel(config.channel, config.slice, level);
}

// --- Public API ---

/// This has to be the very first function call in `main()`.
/// Immediately outputs minimum throttle so the ESC doesn't receive a floating signal.
pub fn init(comptime config: EscConfig) void {
    writeLevel(config.channel, config.slice, config.min_us);
    state = .arming;
}

/// Blocking arming sequence. Call once at boot, right after `init()`.
/// Waits for the ESC startup + cell count + ready beeps. (~5s total)
pub fn arm(comptime config: EscConfig) void {
    // `init()` should have already set the minimum level in PWM, but we are defensive here.
    writeLevel(config, config.min_us);
    rpi.time.sleep_ms(5000);
    state = .armed;
}

/// One-time throttle range calibration. Only flash this once when setting up a new ESC.
/// After a long confirmation beep reflash regular firmware.
/// TODO: add `zig build -Dcalibrate` build option
pub fn calibrate(comptime config: EscConfig) void {
    // max. throttle needs to be on the wire right after power on.
    // `init()` was called right before, so here we override it.
    writeLevel(config, config.max_us);
    rpi.time.sleep_ms(3000); // ESC powers on, beeps "B- B-" at ~2s

    // drop to min. within 5s of the "B- B-"
    writeLevel(config, config.min_us);
    rpi.time.sleep_ms(5000); // cell-count beeps + long confirmation beep

    // set into normal armed state
    state = .armed;
}

/// Set throttle in microseconds. Clamped to [min_us ... max_us]
/// Only works in `armed` state.
pub fn setThrottle(comptime config: EscConfig, us: u16) void {
    if (state != .armed) return;
    writeLevel(config, us);
}

/// Cut throttle and mark as disarmed. Requires rearming to use again.
/// Safety feature
pub fn disarm(comptime config: EscConfig) void {
    writeLevel(config, config.min_us);
    state = .disarmed;
}

pub fn isArmed() bool {
    return state == .armed;
}

pub fn getState() EscState {
    return state;
}
