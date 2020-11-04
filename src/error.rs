//! Error handlers.
//! https://github.com/lab11/buckler/blob/master/software/libraries/better_error/better_error_handling.c

use cortex_m_rt::{exception, ExceptionFrame};
use rtt_target::rprintln;

#[panic_handler]
fn panic(e: &core::panic::PanicInfo) -> ! {
    rprintln!("Unhandled panic; stopping");
    rprintln!("{}", e);
    blink_loop();
}

/// Handles HardFaults. Requires RTT and GPIO to be initialized.
#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    rprintln!("A hard fault occured");
    rprintln!("{:#?}", ef);
    blink_loop();
}

// TODO configure to blink LEDs
fn blink_loop() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
