const std = @import("std");
const microzig = @import("microzig");
const crsf = @import("crsf.zig");
const mixer = @import("mixer.zig");
const build_options = @import("build_options");

pub const enabled: bool = build_options.debug;

const rpi = microzig.hal;
const time = rpi.time;
const uart = rpi.uart;

const Absolute = microzig.drivers.time.Absolute;

// these inline functions will be stripped by the compiler if `enabled` == false

/// Configures UART0 at 115200 baud and registers it as the 'std.log' backend

pub inline fn setup() void {
    if (enabled) {
        const uart0 = uart.instance.UART0;
        uart0.apply(.{
            .baud_rate = 115200,
            .clock_config = rpi.clock_config,
        });
        uart.init_logger(uart0);
        std.log.info("UART successfully set up!", .{});

    }
}

pub const State = struct {
    controls: [16]u16 = @splat(0),
    link_stats: ?crsf.LinkStats = null,
    last_controls: Absolute = @enumFromInt(0),
    last_link_stats: Absolute = @enumFromInt(0),
    before: Absolute = @enumFromInt(0),
    rc_frames:  usize = 0,
    ls_frames:  usize = 0,
    uart_error: usize = 0,


    pub inline fn countControls(self: *State, controls: [16]u16) void {
        if (enabled) {
             self.last_controls = time.get_time_since_boot();
             self.controls = controls;
             self.rc_frames += 1;
        }
    }
    pub inline fn countLinkStats(self: *State, link_stats: crsf.LinkStats) void {
        if (enabled) {
            self.last_link_stats = time.get_time_since_boot();
            self.link_stats = link_stats;
            self.ls_frames += 1;
        }
    }
    pub inline fn countUartError(self: *State) void {
        if (enabled) {
            self.uart_error += 1;
        }
    }

    pub inline fn ticker(self: *State, now: Absolute, failsafe: bool) void {
        if (enabled) {
            // runs every 1s and gives an idea how well the CRSF parser works. expected:   RC: 250, LS: 10, ERR: 0
            if (now.diff(self.before).to_us() > 1_000_000) {
                if (failsafe) {
                    std.log.info("--- FAILSAFE ACTIVE ---", .{});
                }
                std.log.info("RC: {d: >3},  LS: {d: >3},  ERR: {d: >3}", .{ self.rc_frames, self.ls_frames, self.uart_error });
                std.log.info("CH0: {d: >4}, CH1: {d: >4}, CH2: {d: >4}, CH3: {d: >4}", .{ self.controls[0], self.controls[1], self.controls[2], self.controls[3]});
                if (self.link_stats) |ls| {
                    std.log.info("UL: rssi1: {d: >4} | rssi2: {d: >4} | lq: {d: >3}% | snr: {d: >4}dB | ant: {d} | rf: {d} | pwr: {d} || DL: rssi: {d: >4} | lq: {d: >3}% | snr: {d: >4}dB", .{
                        ls.uplink_rssi_1,
                        ls.uplink_rssi_2,
                        ls.uplink_link_quality,
                        ls.uplink_snr,
                        ls.active_antenna,
                        ls.rf_mode,
                        ls.tx_power,
                        ls.downlink_rssi,
                        ls.downlink_link_quality,
                        ls.downlink_snr,
                    });
                } else {
                    std.log.info("LinkStats have not been initialized.", .{});
                }
                std.log.info("", .{});
                self.rc_frames  =  0;
                self.ls_frames  = 0;
                self.uart_error = 0;
                self.before = time.get_time_since_boot();
            }
        }
    }
};
