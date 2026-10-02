const std = @import("std");

pub const RatePid = struct {
    // Umrechnungsfaktor z.b. 0.005 = pro °/s 0.005 also 10°/s = 0.05 = 5% Ausschlag
    // 200°/s = 1 voller Ausschlag
    kp: f32,
    // Maximaler Ausschlag muss begrenzt sein weil sonsts servos übersteuern könnten
    out_limit: f32,
    // Wie schneller i wächst
    ki: f32,
    i_limit: f32,
    // Startwert und integrale aufsummiert
    i_acc: f32 = 0,
    // Umrechnungsfaktor für Ausschlag pro °/s² (0.0001)
    kd: f32,
    // Drehrate vom letzten Aufruf wird benötigt um Änderung auszurechnen
    prev_rate: f32 = 0,
    // Wie viele Messwerte gemittelt werden
    d_alpha: f32,
    // filtered d
    d_filt: f32 = 0,

    pub fn reset(self: *RatePid, rate: f32) void {
        self.i_acc = 0;
        self.prev_rate = rate;
        self.d_filt = 0;
    }

    // rate_sp = Sollrate in °/s rate = Ist-Rate in °/s
    // dt = Zeit seit letztem Aufruf in Sekunden
    // Integrate = switch true in der Luft false am Boden
    pub fn update(self: *RatePid, rate_sp: f32, rate: f32, dt: f32, integrate: bool) f32 {
        // Fehler
        if (dt <= 0) return 0;
        const err = rate_sp - rate;
        // Ist der P-Anteil Ausschlag proportional zum Fehler
        const p = self.kp * err;
        if (integrate) {
            self.i_acc += self.ki * err * dt;
            self.i_acc = std.math.clamp(self.i_acc, -self.i_limit, self.i_limit);
        } else {
            self.i_acc = 0;
        }
        const d_raw = (rate - self.prev_rate) / dt;
        self.prev_rate = rate;
        // Averages 5 last measures when alpha is 0.2
        self.d_filt += self.d_alpha * (d_raw - self.d_filt);
        const d = -self.kd * self.d_filt;
        const out = p + self.i_acc + d;
        return std.math.clamp(out, -self.out_limit, self.out_limit);
    }
};

pub const AngleLoop = struct {
    // Umrechnungsfaktor pro Grad Abstand so und so viel °/s drehen
    kp_angle: f32,
    max_angle: f32,
    max_rate: f32,

    pub fn rateSetPoint(self: AngleLoop, stick: f32, angle: f32) f32 {
        const angle_sp = std.math.clamp(stick, -1, 1) * self.max_angle;
        const angle_err = angle_sp - angle;
        const rate_sp = self.kp_angle * angle_err;
        return std.math.clamp(rate_sp, -self.max_rate, self.max_rate);
    }
};
pub const StickCal = struct {
    min: f32 = 172.0,
    center: f32 = 992.0,
    max: f32 = 1811.0,
    deadband: f32 = 0.03,

    pub fn normalize(self: StickCal, raw: u16) f32 {
        const x: f32 = @floatFromInt(raw);
        var v: f32 = 0;
        if (x > self.center) {
            v = (x - self.center) / (self.max - self.center);
        } else {
            v = (x - self.center) / (self.center - self.min);
        }
        const clamped = std.math.clamp(v, -1, 1);
        const absolute = @abs(clamped);
        if (absolute < self.deadband) return 0;
        return clamped;
    }
    pub fn denormalize(self: StickCal, value: f32) u16 {
        const v = std.math.clamp(value, -1, 1);
        // create const for center
        const center = self.center;
        var out: f32 = 0;
        if (v == 0) {
            out = center;
        } else if (v < 0) {
            out = center + v * (center - self.min);
        } else {
            out = center + v * (self.max - center);
        }
        return @intFromFloat(@round(out));
    }
};
