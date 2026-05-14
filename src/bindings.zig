const microzig = @import("microzig");

const rpi = microzig.hal;
const time = rpi.time;


export fn sleep_ms(us: u32) callconv(.c) void {
    time.sleep_ms(us);
}
