const std = @import("std");
const microzig = @import("microzig");
const servo = @import("servo.zig");
const pwmlib = @import("pwm.zig");
const esc = @import("esc.zig");
const crsf = @import("crsf.zig");
const mixer = @import("mixer.zig");
const debug = @import("debug.zig");
// --- Compile-time build options ---
const build_options = @import("build_options");
const calibrate_mode = build_options.calibrate;
const debug_mode     = build_options.debug; // before 57.5KiB

// --- Aliases ---
const rpi = microzig.hal;
const time = rpi.time;
const uart = rpi.uart;

const Servo = servo.Servo;
const ServoConfig = servo.ServoConfig;
const ServoGroup = servo.ServoGroup;

// --- Configurations ---
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

// --- Hardware Initialization ---


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

    // --- Setup ---
    const pins = pin_config.apply();

    // initialize ESC
    var motor = esc.Esc.init(pins.esc, .{}, calibrate_mode);
    // initialize servos
    const aileron_left  = Servo.init(pins.aileron_left,  .{ .servo_type = .aileron });
    const aileron_right = Servo.init(pins.aileron_right, .{ .servo_type = .aileron });
    const elevator      = Servo.init(pins.elevator,      .{ .servo_type = .elevator });
    const rudder        = Servo.init(pins.rudder,        .{ .servo_type = .rudder });

    // initialize interrupts
    pwmlib.init(pin_config);

    // arm / calibrate ESC
    if (calibrate_mode) motor.calibrate() else motor.arm();

    // debug mode setup
    debug.setup();
    var debug_state = debug.State{};
    // uart & crsf setup
    //setup_uart_logging(); // logging

    const uart_crsf = setup_uart_crsf();
    var fsm = crsf.CrsfFsm{};

    // filled with safe values until there is real data available
    var channels = mixer.PilotControls{
        .ailerons = 992,
        .elevator = 992,
        .throttle = 172,
        .rudder = 992,
        .aux1  = 172, .aux2  = 172, .aux3  = 172, .aux4  = 172,
        .aux5  = 172, .aux6  = 172, .aux7  = 172, .aux8  = 172,
        .aux9  = 172, .aux10 = 172, .aux11 = 172, .aux12 = 172,
    };
    var link_stats: crsf.LinkStats = undefined;

    var failsafe: bool = false; // failsafe flag
    var before = time.get_time_since_boot();
    var last_rc_frame = before;

    // debug counters
    // var uart_errors: usize = 0;
    // var rc_frames: usize = 0;
    // var ls_frames: usize = 0;


    // --- MAIN CONTROL LOOP ---
    while (true) {
        // drain all available bytes into the FSM
        drain: while (true) {
            const byte = uart_crsf.read_word() catch {
                // log the error, clear it and keep going
                //std.log.warn("UART1_RX Error: {}", .{err});
                uart_crsf.clear_errors();
                debug_state.countUartError();
                continue :drain;
            } orelse break :drain; // FIFO empty -> done draining
            fsm.feed(byte);
        }

        // --- CRSF consumers ---
        // only poll each type of CRSF frame once per main loop iteration.

        if (fsm.takeRcChannels()) |ch| {
            debug_state.countControls(ch);
            channels = mixer.genPilotControlsFromChannels(ch);
            last_rc_frame = time.get_time_since_boot();
        }
        if (fsm.takeLinkStats()) |ls| {
            debug_state.countLinkStats(ls);
            link_stats = ls;
        }

        // TODO: PID

        const now = time.get_time_since_boot();
        before = now;

        // --- Mixer ---

        failsafe = now.diff(last_rc_frame).to_us() > 500_000; // sets to true if the last rc frame was received more than 0.5s ago
        mixer.mix(channels, motor, .{ aileron_left, aileron_right, elevator, rudder }, failsafe);

        debug_state.ticker(now, failsafe);

    }
}
