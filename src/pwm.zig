const std = @import("std");
const microzig = @import("microzig");

const rpi = microzig.hal;
const Channel = rpi.pwm.Channel;

/// Clock divisor and wrap settings that produce 1μs ticks and a 50Hz (20ms) wrap period.
pub const clk = struct {
    pub const div:  u8  = 125;       // 125MHz / 125 -> 1us per tick
    pub const wrap: u16 = 19_999;    // 20ms period  -> 50Hz (0 .. 19_999 = 20_000)
    pub const frac: u8  = 0;         // fraction is zero, because it's not needed for this project
};


/// Total number of hardware PWM slices on the microcontroller.
pub const NUM_SLICES: usize = 8;
/// Integer type wide enough to index any PWM slice. Update the bit width if 'NUM_SLICES' ever changes.
pub const SliceIndex = u3; // hardcoded for better ZLS

// NOTE: Use this to make the flight controller compatible with RP2350. Calculates the bit width dynamically.
// pub const SliceIndex = std.meta.Int(.unsigned, std.math.log2(NUM_SLICES));


// --- PWM Register Map ---

const PWM_BASE:     u32 = 0x4005_0000;
const SLICE_STRIDE: u32 = 0x14;  // each slice block is 20 bytes
const CC_OFFSET:    u32 = 0x0C;   // compare/capture register within a slice
const INTR_OFFSET:  u32 = 0xA4;   // raw interrupt status  (write 1 to clear)
const INTE_OFFSET:  u32 = 0xA8;   // interrupt enable      (1 bit per slice)
const INTS_OFFSET:  u32 = 0xB0;   // interrupt status after INTE masking

// --- State ---

/// Per-slice staging area written by the main loop and consumed by the ISR.
/// `active` gates whether the ISR touches the slice at all.
const SliceBuffer =  struct{
    level_a: u16 = 0,
    level_b: u16 = 0,
    active: bool = false,  // false -> entry is skipped
};

/// PWM values are written to this buffer.
/// On interrupt they get later written into the PWM registers
var buffer: [NUM_SLICES]SliceBuffer = [_]SliceBuffer{.{}} ** NUM_SLICES;


// --- Interrupt Handler API ---

/// Register slice to be handled by ISR by adding it to the PWM buffer
pub fn registerSlice(slice_num: SliceIndex) void {
    buffer[slice_num].active = true;
}

/// Schedule a PWM level by storing it in the PWM buffer
pub fn setLevel(ch: Channel, slice: SliceIndex, level: u16) void {
    switch (ch) {
        .a => @atomicStore(u16, &buffer[slice].level_a, level, .monotonic),
        .b => @atomicStore(u16, &buffer[slice].level_b, level, .monotonic),
    }
}

/// Enables the wrap interrupt for one slice by setting its bit in INTE.
/// Must be called after the slice is fully configured (div, wrap, enabled).
pub fn enableSliceIrq(slice_num: SliceIndex) void {
    const INTE = @as(*volatile u32, @ptrFromInt(PWM_BASE + INTE_OFFSET));
    INTE.* |= @as(u32, 1) << slice_num; // NOTE: here the small SliceIndex is needed instead of u32
}

/// Unmasks PWM_IRQ_WRAP in the CPU NVIC. Call once, after all slices are configured and enabled.
pub fn enableCpuIrq() void {
    // --- old version ---
    // NVIC ISER0: writing a 1 to bit N enables IRQ N
    // PWM_IRQ_WRAP = IRQ 4 (ref: RP2040 datasheet Table 2.4)
    // const NVIC_ISER0 = @as(*volatile u32, @ptrFromInt(0xE000_E100));
    // NVIC_ISER0.* = 1 << 4;
    // --- old version ---
    microzig.cpu.interrupt.enable(.PWM_IRQ_WRAP);
}

/// Debug counter. Incremented on every ISR fire; at 50Hz with N active slices it ticks at 50N/s.
pub var fire_counter: usize = 0;

/// ISR / PWM interrupt handler. Fires at 50Hz for each registered slice.
///
/// At the start of each new 20ms cycle the counter is just reset to 0.
/// Updating CC here is glitch-free, as there is no ongoing pulse.
pub fn handler() callconv(.c) void {
    const INTS = @as(*volatile u32, @ptrFromInt(PWM_BASE + INTS_OFFSET));
    const INTR = @as(*volatile u32, @ptrFromInt(PWM_BASE + INTR_OFFSET));

    // find out which slices are wrapped
    var fired: u8 = @truncate(INTS.*);
    // acknowledge all fired interrupts in one write (W1C)
    INTR.* = fired;

    // apply levels from the buffer for every fired slice
    while(fired != 0) {
        const index: SliceIndex = @truncate(@ctz(fired));
        fired &= fired - 1;

        const b = &buffer[index];

        if (!b.active) continue;
        //debug
        fire_counter += 1;

        // atomicLoad might not be necessary here, but it's safer to use.
        const level_a = @atomicLoad(u16, &b.level_a, .monotonic);
        const level_b = @atomicLoad(u16, &b.level_b, .monotonic);

        // CC layout: bits[0:15] = channel a, bits[16:31] = channel b
        // one 32bit write sets both channels atomically.
        const cc_address = PWM_BASE + @as(u32, index) * SLICE_STRIDE + CC_OFFSET;
        const cc = @as(*volatile u32, @ptrFromInt(cc_address));
        cc.* = @as(u32, level_b) << 16 | level_a;
    }
}

/// Sets up the PWM interrupts using the `GlobalConfiguration` at comptime.
/// No overhead during runtime.
pub fn init(comptime pin_config: anytype) void {
    const active_slices = comptime blk: {
        // the BitSet here is used for deduplication
        var slices = std.StaticBitSet(NUM_SLICES).initEmpty();
        for (std.meta.fields(@TypeOf(pin_config))) |field| {
            if (@field(pin_config, field.name)) |config| {
                if (config.function.is_pwm()) {
                    // sets the bit at SLICE_NUM location
                    slices.set(config.function.pwm_slice());
                }
            }
        }
        break :blk slices;
    };
    inline for (0..NUM_SLICES) |i| {
        if (comptime active_slices.isSet(i)) {
            registerSlice(@truncate(i));
            enableSliceIrq(@truncate(i));
        }
    }
    enableCpuIrq();
}
