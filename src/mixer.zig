const std = @import("std");
const servo = @import("servo.zig");
const esc = @import("esc.zig");

/// CRSF values
pub const crsf_min: u16 = 172;
pub const crsf_mid: u16 = 992;
pub const crsf_max: u16 = 1811;

/// Used to make the RC channel access more convenient.
pub const PilotControls = struct {
    ailerons: u16,
    elevator: u16,
    throttle: u16,
    rudder: u16,
    aux1: u16,
    aux2: u16,
    aux3: u16,
    aux4: u16,
    aux5: u16,
    aux6: u16,
    aux7: u16,
    aux8: u16,
    aux9: u16,
    aux10: u16,
    aux11: u16,
    aux12: u16,
};
/// Control intents corresponding index in CRSF RC Channels
pub const RcChannelIndex = enum(u4) {
    ailerons = 0,
    elevator = 1,
    throttle = 2,
    rudder = 3,
    aux1 = 4,
    aux2 = 5,
    aux3 = 6,
    aux4 = 7,
    aux5 = 8,
    aux6 = 9,
    aux7 = 10,
    aux8 = 11,
    aux9 = 12,
    aux10 = 13,
    aux11 = 14,
    aux12 = 15,
};

/// Helper function to generate a `PilotControls` struct from the raw CRSF RC channels
pub fn genPilotControlsFromChannels(channels: [16]u16) PilotControls {
    return .{
        .ailerons = channels[0],
        .elevator = channels[1],
        .throttle = channels[2],
        .rudder = channels[3],
        .aux1 = channels[4],
        .aux2 = channels[5],
        .aux3 = channels[6],
        .aux4 = channels[7],
        .aux5 = channels[8],
        .aux6 = channels[9],
        .aux7 = channels[10],
        .aux8 = channels[11],
        .aux9 = channels[12],
        .aux10 = channels[13],
        .aux11 = channels[14],
        .aux12 = channels[15],
    };
}

/// Applies RC channel values from `PilotControls` to the ESC & servos.
pub fn mix(channels: [16]u16, motor: esc.Esc, servos: anytype, failsafe: bool) void {
    if (!failsafe) {
        motor.setThrottle(scaleToUs(ch(channels, RcChannelIndex.throttle), motor.config.min_us, motor.config.max_us));
        inline for (servos) |s| {
            switch (s.config.servo_type) {
                .aileron => s.setPulse(scaleToUs(ch(channels, RcChannelIndex.ailerons), s.config.min_us, s.config.max_us)),
                .elevator => s.setPulse(scaleToUs(ch(channels, RcChannelIndex.elevator), s.config.min_us, s.config.max_us)),
                .rudder => s.setPulse(scaleToUs(ch(channels, RcChannelIndex.rudder), s.config.min_us, s.config.max_us)),
                else => unreachable,
            }
        }
    } else { // failsafe
        motor.setThrottle(1100); // cut off power to motor
        inline for (servos) |s| {
            switch (s.config.servo_type) {
                .aileron => s.setPulse(s.config.center_us + 100), // slight roll, so the plane goes in a slight circle
                .elevator => s.setPulse(s.config.center_us + 100), // slight downwards pitch (elevator up) so it descends
                .rudder => s.setPulse(s.config.center_us + 100), // slight yaw in the direction of the roll for circle
                else => unreachable,
            }
        }
    }
}
/// Helper function to get channel value by enum
inline fn ch(channels: [16]u16, c: RcChannelIndex) u16 {
    return channels[@intFromEnum(c)];
}
/// Scales CRSF values to PWM values.
fn scaleToUs(crsf_value: u16, min_us: u16, max_us: u16) u16 {
    const crsf: u32 = @intCast(std.math.clamp(crsf_value, crsf_min, crsf_max));
    // using saturating subtraction to guard against a glitchy signal (crsf < 172)
    const numerator: u32 = (crsf -| crsf_min) * @as(u32, max_us - min_us);
    const result: u32 = numerator / @as(u32, crsf_max - crsf_min) + min_us;
    return @truncate(result);
}
