#![no_main]
#![no_std]

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use nrf52832_hal as hal;
use nrf52832_hal::gpio::Level;
use nrf52832_hal::delay;
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use rtt_target::{rprintln, rtt_init_print};

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    let p = hal::pac::Peripherals::take().unwrap();
    let port0 = hal::gpio::p0::Parts::new(p.P0);
    let button = port0.p0_28.into_pullup_input();
    let mut led = port0.p0_23.into_push_pull_output(Level::Low);
    let c = hal::pac::CorePeripherals::take().unwrap();
    // let mut delay = delay::Delay::new(c.SYST);
    rprintln!("Blinky button demo starting");
    loop {
        // delay.delay_ms(100u16);
        if button.is_high().unwrap() {
            led.set_low().unwrap();
        } else {
            led.set_high().unwrap();
        }
    }
}

