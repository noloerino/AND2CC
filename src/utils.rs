// The NRF is clocked at 64 MHz, which translates to 64000000 insts/1000 ms
const INST_PER_MS: u32 = 64000;

/// Blocks the program for at least `ms` millisecnods.
/// Because this uses cortex_m::asm::delay under the hood, the actual delay may be longer
/// due to interrupts etc.
pub fn delay_ms(ms: u32) {
    cortex_m::asm::delay(INST_PER_MS * ms);
}
