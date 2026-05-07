pub const div:  u8  = 125;       // 125MHz / 125 -> 1us per tick
pub const wrap: u16 = 19_999;    // 20ms period  -> 50Hz (0 .. 19_999 = 20_000)
pub const frac: u8  = 0;         // fraction is zero, because it's not needed for this project
