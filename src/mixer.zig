const servo = @import("servo.zig");
const esc = @import("esc.zig");

/// Used to make the RC channel access more convenient.
pub const PilotControls = struct {
    ailerons: u16,
    elevator: u16,
    throttle: u16,
    rudder:   u16,
    aux1:     u16,
    aux2:     u16,
    aux3:     u16,
    aux4:     u16,
    aux5:     u16,
    aux6:     u16,
    aux7:     u16,
    aux8:     u16,
    aux9:     u16,
    aux10:    u16,
    aux11:    u16,
    aux12:    u16,
};

/// Helper function to generate a `PilotControls` struct from the raw CRSF RC channels
pub fn genPilotControlsFromChannels(channels: [16]u16) PilotControls {
    return .{
        .ailerons = channels[0],
        .elevator = channels[1],
        .throttle = channels[2],
        .rudder   = channels[3],
        .aux1     = channels[4],
        .aux2     = channels[5],
        .aux3     = channels[6],
        .aux4     = channels[7],
        .aux5     = channels[8],
        .aux6     = channels[9],
        .aux7     = channels[10],
        .aux8     = channels[11],
        .aux9     = channels[12],
        .aux10    = channels[13],
        .aux11    = channels[14],
        .aux12    = channels[15],
    };
}

/// Applies RC channel values from `PilotControls` to the ESC & servos.
pub fn mix(pc: PilotControls, motor: esc.Esc , servos: anytype, failsafe: bool) void {
    if(!failsafe) {
        motor.setThrottle(scaleToUs(pc.throttle, motor.config.min_us, motor.config.max_us));
        inline for (servos) |s| {
            switch (s.config.servo_type) {
                .aileron  => s.setPulse(scaleToUs(pc.ailerons, s.config.min_us, s.config.max_us)),
                .elevator => s.setPulse(scaleToUs(pc.elevator, s.config.min_us, s.config.max_us)),
                .rudder   => s.setPulse(scaleToUs(pc.rudder,   s.config.min_us, s.config.max_us)),
                else => unreachable,
            }
        }
    } else {

    }
}
// NOTE: the integer divisions are fine for now, but maybe later we should leverage the hardware SIO divider. it does integer division in 8 cycles instead of the 20-40 cycles.
/// Scales CRSF values to PWM values.
fn scaleToUs(crsf: u16, min_us: u16, max_us: u16) u16 {
    const crsf_min =  172; // minimum for CRSF values
    const crsf_max = 1811; // maximum for CRSF values
    // using saturating subtraction to guard against a glitchy signal (crsf < 172)
    return (crsf -| crsf_min) * (max_us - min_us) / (crsf_max - crsf_min) + min_us;
}
