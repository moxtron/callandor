const std = @import("std");

/// This is a CRSF (Crossfire) protocol implementation in Zig.

// SOURCE: // https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md

// HOW TO USE
// while (uart.readByte()) |byte| fsm.feed(byte);
// // might have to call this one in a loop if adding more frame types to ensure all data is actually consumed
// switch (fsm.takeFrame()) {
//     .none        => {},
//     .rc_channels => |ch| { /* update pilot input */ },
//     .link_stats  => |ls| { /* update telemetry display */ },
// }

const FrameType = enum(u8) {
    gps          = 0x02,
    vario        = 0x07,
    battery      = 0x08,
    barometer    = 0x09,
    heartbeat    = 0x0B,
    link_stats   = 0x14,
    rc_channels  = 0x16,
    rc_subset    = 0x17,
    attitude     = 0x1E,
    flight_mode  = 0x21,
    ping         = 0x28,
    device_info  = 0x29,
    param_entry  = 0x2B,
    command      = 0x32,
    _,
};

// maybe expand the enums later if it turns out to be useful. for now, this is ok.
const LinkStats = struct {
    uplink_rssi_1: u8,          // Uplink RSSI Antenna 1 [dBm * -1]
    uplink_rssi_2: u8,          // Uplink RSSI Antenna 2 [dBm * -1]
    uplink_link_quality: u8,    // Uplink Package success rate / Link quality [%]
    uplink_snr: i8,             // Uplink Signal-to-Noise Ratio (SNR) [dB]
    active_antenna: u8,         // enum {1 = 0, 2}, number of currently best antenna
    rf_mode: u8,                // enum {4fps = 0, 50fps, 150fps}
    tx_power: u8,               // enum {0mW = 0, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW, 250mW, 50mW}
    downlink_rssi: u8,          // Downlink RSSI [dBm * -1]
    downlink_link_quality: u8,  // Downlink Package success rate / Link quality (%)
    downlink_snr: i8            // Downlink Signal-to-Noise Ratio (SNR) [dB]
};

/// This is the union type of the possible results of a decoded CRSF frame.
const FrameResult = union(enum) {
    none,
    rc_channels: [16]u11,
    link_stats: LinkStats,
};

const CrsfState = enum { idle, length, frame_type, payload };


/// This is a finite state machine (FSM) that decodes CRSF frames.
const CrsfFsm = struct {
    state: CrsfState      = .idle,
    buffer: [60]u8        = @splat(0), // max frame size is 64 bytes, 1[device] + 1[length] + 1[type] + 60[payload] + 1[crc]
    length: u8            = 0,
    frame_type: FrameType = undefined,
    index: u8             = 0,  // current buffer position

    // parsed, clean data for consumers
    // consume-on-read to prevent blocking or stale data
    rc_channels: ?[16]u11  = null,
    link_stats: ?LinkStats = null,

    pub fn feed(self: *CrsfFsm, byte: u8) void {
        switch(self.state) {
            .idle => {
                if (byte == 0xC8) {
                    self.state = .length;
                }
            },
            .length => {
                // guards against accessing out-of-bounds buffer memory
                if (byte > self.buffer.len + 2 or byte < 4) {
                    self.reset();
                    return;
                }
                self.length = byte;
                self.state  = .frame_type;
            },
            .frame_type => {
                const frame_type: FrameType = @enumFromInt(byte);
                // the switch here is the best option as the allowed FrameTypes are known at comptime
                // keeps branching at a minimum
                const expected_length: u8 = switch (frame_type) {
                    .rc_channels => 24,
                    .link_stats  => 12,
                    else => 0,
                };
                if (expected_length == 0 or self.length != expected_length) {
                    self.reset();
                } else {
                    self.frame_type = frame_type;
                    self.state = .payload;
                }
            },
            // either puts the current byte into the buffer or handles the buffer if ready (crc ok & length reached)
            .payload => {
                if (self.index < self.length - 2) {
                    self.buffer[self.index] = byte;
                    self.index += 1;
                } else {
                    const checksum = self.crc8();
                    if (byte == checksum) { // cleanup
                        switch(self.frame_type) {
                            .rc_channels => {
                                self.rc_channels = self.decodeRcChannels();
                            },
                            .link_stats => {
                                self.link_stats = self.decodeLinkStats();
                            },
                            else => {
                                // this shouldnt happen normally, but errors occur
                                self.reset();
                            }
                        }
                        self.reset();
                    } else {
                        self.reset();
                    }
                }
            },
        }
    }
    pub fn takeFrame(self: *CrsfFsm) FrameResult {
        // rc_channels takes precedence over telemetry
        if (self.rc_channels) |channels| {
            self.rc_channels = null;
            return .{ .rc_channels = channels };
        }
        if (self.link_stats) |stats| {
            self.link_stats = null;
            return .{ .link_stats = stats };
        }
    }
    fn decodeLinkStats(self: *const CrsfFsm) LinkStats {
        return .{
            .uplink_rssi_1          = self.buffer[0],
            .uplink_rssi_2          = self.buffer[1],
            .uplink_link_quality    = self.buffer[2],
            .uplink_snr             = @bitCast(self.buffer[3]),
            .active_antenna         = self.buffer[4],
            .rf_mode                = self.buffer[5],
            .tx_power               = self.buffer[6],
            .downlink_rssi          = self.buffer[7],
            .downlink_link_quality  = self.buffer[8],
            .downlink_snr           = @bitCast(self.buffer[9]),
        };
    }
    fn decodeRcChannels(self: *const CrsfFsm) [16]u11 {

        var channels: [16]u11 = undefined;
        const payload = self.buffer[0..22]; // for readability

        inline for(0..16) |i| {
            const bit_pos  = i * 11;
            const byte_pos = bit_pos / 8;

            // defines at what offset in byte_pos the channel starts
            // u5, because zig needs a number able to represent all possible shift values for the type ( log2(32) = 5 )
            const bit_shift: u5 = @intCast(bit_pos % 8);

            // needs to be at least 24bits, but rp2040 handles 32bit natively. might as well choose it here
            const b0: u32 = payload[byte_pos];
            const b1: u32 = payload[byte_pos + 1];

            // comptime here evaluates the if-condition during comptime,
            // allowing for the inline for loop to be fully unrolled.
            const word: u32 = if (comptime bit_shift + 11 > 16) blk: { // block syntax FTW
                const b2: u32 = payload[byte_pos + 2];
                break :blk b0 | (b1 << 8) | (b2 << 16); // here we have a total of 24bits
            } else blk: {
                break :blk b0 | (b1 << 8);
            };
            // shifts the word right by the offset and applies the mask for 11 bits
            channels[i] = @truncate((word >> bit_shift) & 0x7FF);
        }
        return channels;
        //self.channels = channels;
    }
    fn crc8(self: *const CrsfFsm) u8 {
        const polynomial: u8 = 0xD5;
        const top_bit: u8 = 1 << 7;
        var crc: u8 = 0;

        crc ^= @intFromEnum(self.frame_type);

        for (0..8) |_| {
            if (crc & top_bit != 0) {
                crc = @truncate((@as(u9, crc) << 1) ^ polynomial);
            } else {
                crc = @truncate(@as(u9, crc) << 1);
            }
        }

        for (self.buffer[0..self.length - 2]) |byte| {
            crc ^= byte;

            for (0..8) |_| {
                if (crc & top_bit != 0) {
                    crc = @truncate((@as(u9, crc) << 1) ^ polynomial);
                } else {
                    crc = @truncate(@as(u9, crc) << 1);
                }
            }
        }
        return crc;
    }
    fn reset(self: *CrsfFsm) void {
        self.state = .idle;
        self.index = 0;
        // the following are not strictly needed, but help a lot when debugging
        self.length = 0;
        self.frame_type = undefined;
    }
};
