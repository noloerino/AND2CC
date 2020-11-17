//! Error handlers.
//! https://github.com/lab11/buckler/blob/master/software/libraries/better_error/better_error_handling.c

use cortex_m_rt::{exception, ExceptionFrame};
use rtt_target::rprintln;

#[panic_handler]
fn panic(e: &core::panic::PanicInfo) -> ! {
    rprintln!("Unhandled panic; stopping");
    rprintln!("{}", e);
    dead_loop();
}

/// Handles HardFaults. Requires RTT to be initialized.
#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    rprintln!("A hard fault occured");
    rprintln!("{:#?}", ef);
    dead_loop();
}

fn dead_loop() -> ! {
    cortex_m::interrupt::disable();
    loop {
        cortex_m::asm::bkpt();
    }
}
