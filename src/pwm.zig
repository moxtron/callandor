const std = @import("std");
const microzig = @import("microzig");

const rpi = microzig.hal;
const Channel = rpi.pwm.Channel;

pub const div:  u8  = 125;       // 125MHz / 125 -> 1us per tick
pub const wrap: u16 = 19_999;    // 20ms period  -> 50Hz (0 .. 19_999 = 20_000)
pub const frac: u8  = 0;         // fraction is zero, because it's not needed for this project


// RP2040 PWM peripheral constants
const PWM_BASE: u32    = 0x4005_0000;
const SLICE_STRIDE: u32 = 0x14;  // each slice block is 20 bytes
const CC_OFFSET: u32   = 0x0C;   // compare/capture register within a slice
const INTR_OFFSET: u32 = 0xA4;   // raw interrupt status  (write 1 to clear)
const INTE_OFFSET: u32 = 0xA8;   // interrupt enable      (1 bit per slice)
const INTS_OFFSET: u32 = 0xB0;   // interrupt status after INTE masking

// total amount of pwm slices on the RP2040
const NUM_SLICES: usize = 8;

const SliceBuffer =  struct{
    level_a: u16 = 0,
    level_b: u16 = 0,
    active: bool = false,  // false -> entry is skipped
};

// pwm values are written to this buffer
// on interrupt they get later written into the pwm registers
var buffer: [NUM_SLICES]SliceBuffer = [_]SliceBuffer{.{}} ** NUM_SLICES;

// ----------------------------------------
// PUBLIC API
// ----------------------------------------

/// mark slice as handled by ISR
/// NOTE: documentation
pub fn registerSlice(slice_num: u3) void {
    buffer[slice_num].active = true;
}

/// schedule a new level
pub fn setLevel(ch: Channel, slice: u3, level: u16) void {
    switch (ch) {
        .a => @atomicStore(u16, &buffer[slice].level_a, level, .monotonic),
        .b => @atomicStore(u16, &buffer[slice].level_b, level, .monotonic),
    }
}

/// enable PWM wrap interrupt for one slice. Sets its bit in PWM INTE
/// call this after the slice is fully configured (div, wrap, enabled)
pub fn enableSliceIrq(slice_num: u3) void {
    const INTE = @as(*volatile u32, @ptrFromInt(PWM_BASE + INTE_OFFSET));
    INTE.* |= @as(u32, 1) << slice_num; // NOTE: here the u3 is needed instead of u32
}

pub fn enableCpuIrq() void {
    // --- old version ---
    // NVIC ISER0: writing a 1 to bit N enables IRQ N
    // PWM_IRQ_WRAP = IRQ 4 (ref: RP2040 datasheet Table 2.4)
    // const NVIC_ISER0 = @as(*volatile u32, @ptrFromInt(0xE000_E100));
    // NVIC_ISER0.* = 1 << 4;
    // --- old version ---
    microzig.cpu.interrupt.enable(.PWM_IRQ_WRAP);
}

/// PWM interrupt handler. Fires at 50Hz for each registered slice.
///
/// At the start of each new 20ms cycle the counter is just reset to 0.
/// Updating CC here is glitch-free, as there is no ongoing pulse.
pub fn handler() callconv(.c) void {
    const INTS = @as(*volatile u32, @ptrFromInt(PWM_BASE + INTS_OFFSET));
    const INTE = @as(*volatile u32, @ptrFromInt(PWM_BASE + INTR_OFFSET));

    var fired: u8 = @truncate(INTS.*);

    // apply levels from the buffer for every fired slice
    while(fired != 0) {
        const index: u3 = @truncate(@ctz(fired));
        fired &= fired - 1;

        const b = &buffer[index];

        if (!b.active) continue;

        const level_a = @atomicLoad(u16, &b.level_a, .monotonic);
        const level_b = @atomicLoad(u16, &b.level_b, .monotonic);

        // CC layout: bits[0:15] = channel a, bits[16:31] = channel b
        // one 32bit write sets both channels atomically.
        const cc_address = PWM_BASE + @as(u32, index) * SLICE_STRIDE + CC_OFFSET;
        const cc = @as(*volatile u32, @ptrFromInt(cc_address));
        cc.* = @as(u32, level_b) << 16 | level_a;
    }
}
