#![no_main]
#![no_std]

mod kobuki;

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use kobuki::actuator::Actuator;
use kobuki::utilities;
use nrf52832_hal as hal;
use nrf52832_hal::delay;
use nrf52832_hal::gpio::Level;
use panic_rtt_core as _;
use rtt_target::{rprintln, rtt_init_print};

/*
#[panic_handler] // panicking behavior
fn panic(e: &core::panic::PanicInfo) -> ! {
    loop {
        rprintln!("Unhandled panic; stopping");
        rprintln!(
            "{:?} @ {:?}",
            e.payload().downcast_ref::<&str>(),
            e.location().unwrap()
        );
        cortex_m::asm::bkpt();
    }
}
*/

enum DriveState {
    Off,
    TurnCcw,
    Drive,
}

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    let p = hal::pac::Peripherals::take().unwrap();
    let port0 = hal::gpio::p0::Parts::new(p.P0);
    let button = port0.p0_28.into_pullup_input();
    let mut led = port0.p0_23.into_push_pull_output(Level::Low);
    let c = hal::pac::CorePeripherals::take().unwrap();
    let mut delay = delay::Delay::new(c.SYST);
    let mut uart = utilities::init_uart0(
        port0.p0_08.into_floating_input().degrade(),
        port0.p0_06.into_push_pull_output(Level::High).degrade(),
        p.UARTE0,
    );
    let mut actuator = Actuator::new(&mut uart);
    let mut state = DriveState::Off;
    rprintln!("Blinky button/drive demo starting");
    let mut ctr: u32 = 0;
    loop {
        delay.delay_ms(100u16);
        if button.is_low().unwrap() {
            led.set_low().unwrap();
        } else {
            led.set_high().unwrap();
        }
        match state {
            DriveState::Off => {
                if button.is_low().unwrap() {
                    state = DriveState::Drive;
                    ctr = 0;
                    rprintln!("Begin drive");
                }
                actuator.drive_direct(0, 0).unwrap();
            }
            DriveState::Drive => {
                if button.is_low().unwrap() {
                    state = DriveState::Off;
                    rprintln!("Drive off");
                } else if ctr == 10 {
                    state = DriveState::TurnCcw;
                    ctr = 0;
                } else {
                    ctr += 1;
                    rprintln!("drive ctr: {}", ctr);
                    if let Err(e) = actuator.drive_direct(100, 100) {
                        rprintln!("Error attempting drive: {:?}", e);
                    }
                }
            }
            DriveState::TurnCcw => {
                if button.is_low().unwrap() {
                    state = DriveState::Off;
                    rprintln!("Drive off");
                } else if ctr == 10 {
                    state = DriveState::Drive;
                    ctr = 0;
                } else {
                    ctr += 1;
                    rprintln!("turn ctr: {}", ctr);
                    if let Err(e) = actuator.drive_direct(-100, 100) {
                        rprintln!("Error attempting drive: {:?}", e);
                    }
                }
            }
        }
    }
}
