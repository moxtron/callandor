//! This is a CRSF (Crossfire) protocol implementation in Zig.
//! SOURCE: // https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md
//!
//! CRSF frame format:
//!  [ device address ] [ length ] [ type ] [ payload ... ] [ CRC8 ]
//! |      1 byte      |  1 byte  |  1 byte | <=60 bytes   | 1 byte |

const std = @import("std");

/// CRSF frame type identifiers as defined in the CRSF specification.
/// Only `rc_channels` and `link_stats` are actively decoded; all others are ignored.
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
    unknown      = 0xFF, // for debugging only
    _,
};

/// Link quality telemetry from the ELRS transmitter. RSSI values are stored as positive integers; multiply by -1 for actual dBm.
pub const LinkStats = struct {
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

/// Result of a decoded CRSF frame. Returns '.none' when no complete frame is available.
pub const FrameResult = union(enum) {
    none,
    rc_channels: [16]u16,
    link_stats: LinkStats,
};

/// Internal FSM states for parsing a CRSF frame byte-by-byte.
const CrsfState = enum { idle, length, frame_type, payload };

/// Byte-by-byte CRSF frame decoder. Feed raw UART bytes via 'feed', then call 'takeFrame' to retrieve decoded frames.
pub const CrsfFsm = struct {
    state: CrsfState      = .idle,
    buffer: [22]u8        = @splat(0), // takes the size of the biggest handled payload
    length: u8            = 0,
    frame_type: FrameType = undefined,
    index: u8             = 0,  // current buffer position

    // parsed, clean data for consumers
    // consume-on-read to prevent blocking or stale data
    rc_channels: ?[16]u16  = null, // NOTE: no struct with named fields, as the use for each channel is not set in stone. Use mixer.PilotControls as a wrapper.
    link_stats: ?LinkStats = null,

    // --- Public API ---

    /// Advances the FSM with one byte from the UART stream. Updates 'rc_channels' or 'link_stats' on a complete, valid frame.
    pub fn feed(self: *CrsfFsm, byte: u8) void {
        switch(self.state) {
            .idle => {
                if (byte == 0xC8) {
                    self.state = .length;
                }
            },
            .length => {
                // guards against accessing out-of-bounds buffer memory
                const MAX_FRAME_SIZE: u8 = self.buffer.len + 2; // biggest handled frame size
                const MIN_FRAME_SIZE: u8 = 4;   // smallest valid frame size in CRSF
                if (byte > MAX_FRAME_SIZE or byte < MIN_FRAME_SIZE) {
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
                            else => self.reset(), // generates a compile error if there are more than 2 frame types in use
                        }
                        self.reset();
                    } else {
                        self.reset();
                    }
                }
            },
        }
    }

    // This new approach of typed "take methods" makes sure the consumer gets the freshest data of EACH type.
    // The historical approach, paired with a `while(true)` loop until `.none` is returned, updates the rc channels very often
    // while waiting for a `link_stats` frame, burning cycles in the process.

    // --- `take` Consumer Functions

    /// Returns an array of the latest RC channels, or `null` if no new "RC channels packed" (0x16) frame has been decoded since the last call.
    /// Calling this function resets the stored value to `null`, so subsequent calls return `null` until a new such frame is decoded.
    pub fn takeRcChannels(self: *CrsfFsm) ?[16]u16 {
        defer self.rc_channels = null;
        return self.rc_channels;
    }
    /// Returns a struct with link statistics or `null` if no new "Link Statistics" (0x14) frame has been decoded since the last function call.
    /// Calling this function resets the stored value to `null`, so subsequent calls return `null` until a new such frame is decoded.
    pub fn takeLinkStats(self: *CrsfFsm) ?LinkStats {
        defer self.link_stats = null;
        return self.link_stats;
    }

    // --- Frame Decoders ---

    /// Parses a 'Link Statistics' payload from 'self.buffer' and returns the decoded struct.
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
    /// Unpacks 16 11bit RC channel values from the packed 'RC Channels' payload in 'self.buffer'.
    fn decodeRcChannels(self: *const CrsfFsm) [16]u16 {

        var channels: [16]u16 = undefined;
        const payload = self.buffer[0 .. self.length - 2]; // for readability

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
            const word: u32 = if (bit_shift + 11 > 16) blk: { // block syntax FTW
                const b2: u32 = payload[byte_pos + 2];
                break :blk b0 | (b1 << 8) | (b2 << 16); // here we have a total of 24bits
            } else blk: {
                break :blk b0 | (b1 << 8);
            };
            // shifts the word right by the offset and applies the mask for 11 bits
            channels[i] = @truncate((word >> bit_shift) & 0x7FF);
        }
        return channels;
    }
    // --- Internal Utilities ---

    /// Computes the CRC-8/DVB-S2 checksum over 'frame_type' and the buffered payload bytes.
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
    /// Resets the state machine to the `.idle` state.
    /// Deletes the entries which are not overwritten in the next round anyway.
    fn reset(self: *CrsfFsm) void {
        self.state = .idle;
        self.index = 0;
        // the following are not strictly needed, but help a lot when debugging
        self.length = 0;
        self.frame_type = .unknown;
    }
};
