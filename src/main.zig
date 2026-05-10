const std = @import("std");
const microzig = @import("microzig");
const servo = @import("servo.zig");
const pwmlib = @import("pwm.zig");
const esc = @import("esc.zig");
const crsf = @import("crsf.zig");

// --- Compile-time build options ---
const build_options = @import("build_options");
const calibrate_mode = build_options.calibrate;

pub const microzig_options: microzig.Options = .{
    .interrupts = .{
        .PWM_IRQ_WRAP = .{ .c = pwmlib.handler }
    },
    .logFn = uart.log,
};

/// Compile-time pin assignment for UART and PWM peripherals.
const pin_config = rpi.pins.GlobalConfiguration{
    .GPIO0 = .{
        .name = "uart0_tx",
        .function = .UART0_TX
    },
    .GPIO1 = .{
        .name = "uart0_rx",
        .function = .UART0_RX
    },
    .GPIO8 = .{
        .name = "elrs_tx",  // connects to RX on ELRS receiver
        .function = .UART1_TX
    },
    .GPIO9 = .{
        .name = "elrs_rx",  // connects to TX on ELRS receiver
        .function = .UART1_RX
    },
    .GPIO16 = .{
        .name = "aileron_left",
        .direction = .out,
        .function = .PWM0_A,
    },
    .GPIO17 = .{
        .name = "aileron_right",
        .direction = .out,
        .function = .PWM0_B,
    },
    .GPIO18 = .{
        .name = "elevator",
        .direction = .out,
        .function = .PWM1_A,
    },
    .GPIO19 = .{
        .name = "rudder",
        .direction = .out,
        .function = .PWM1_B,
    },
    .GPIO22 = .{
        .name = "esc",
        .direction = .out,
        .function = .PWM3_A,
    },
};

// --- Aliases ---
const rpi = microzig.hal;
const time = rpi.time;
const uart = rpi.uart;

const Servo = servo.Servo;
const ServoConfig = servo.ServoConfig;
const ServoGroup = servo.ServoGroup;

// --- Hardware Initialization ---

/// Configures UART0 at 115200 baud and registers it as the 'std.log' backend. Debug use only.
fn setup_uart_logging() void {
    const uart0 = uart.instance.UART0;
    uart0.apply(.{
        .baud_rate = 115200,
        .clock_config = rpi.clock_config,
    });
    uart.init_logger(uart0);

    std.log.info("UART successfully set up!", .{});
}
/// Configures UART1 at 420_000 baud for ELRS/CRSF receiver communication.
fn setup_uart_crsf() uart.UART {
    const uart1 = uart.instance.UART1;
    uart1.apply(.{
        .baud_rate = 420_000,
        .clock_config = rpi.clock_config,
    });
    return uart1;
}

pub fn main() void {

    // # --- Setup ---
    const pins = pin_config.apply();

    // initialize ESC
    var motor = esc.Esc.init(pins.esc, .{}, calibrate_mode);
    // initialize servos
    const aileron_left  = Servo.init(pins.aileron_left,  .{});
    const aileron_right = Servo.init(pins.aileron_right, .{});
    const elevator      = Servo.init(pins.elevator,      .{});
    const rudder        = Servo.init(pins.rudder,        .{});

    // initialize interrupts
    pwmlib.init(pin_config);

    // arm / calibrate ESC
    if (calibrate_mode) motor.calibrate() else motor.arm();

    // uart & crsf setup
    setup_uart_logging(); // logging
    const uart_crsf = setup_uart_crsf();
    var fsm = crsf.CrsfFsm{};

    // group servos for easier testing
    const front = ServoGroup(2).init(.{
        aileron_left,
        aileron_right,
    });
    const rear = ServoGroup(2).init(.{
        elevator,
        rudder,
    });
    const level = ServoConfig{};    // easy access to default servo levels

    // TODO: drive servos & motor from `mixer.zig`
    _ = front;
    _ = rear;
    _ = level;

    // # --- MAIN LOOP ---
    while (true) {
        // drain all available bytes into the FSM
        while (true) {
            const received = uart_crsf.read_word() catch blk: {
                // log the error, clear it and keep going
                //std.log.warn("UART1_RX Error: {}", .{err});
                uart_crsf.clear_errors();
                break :blk null;
            };
            const byte = received orelse break;
            fsm.feed(byte);
        }

        // this while loop only finishes when no more frames are available
        while (true) {
        // consume one decoded frame per loop
            switch (fsm.takeFrame()) {
                .none => break,

                .rc_channels => |ch| {
                    std.log.info(
                        "CH: {d} {d} {d} {d} | {d} {d} {d} {d}",
                        .{ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7]}
                    );
                },
                .link_stats => |ls| {
                    std.log.info("LQ: {d}%  RSSI1: -{d}dBm  SNR: {d}dB", .{
                        ls.uplink_link_quality,
                        ls.uplink_rssi_1,
                        ls.uplink_snr,
                    });
                },

            }
        }
    }
}
