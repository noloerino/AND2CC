//! Utilities for Kobuki functions.
//! https://github.com/noloerino/149-labs/blob/master/software/libraries/kobuki/kobukiUtilities.c

use nrf52832_hal::gpio::{Floating, Input, Output, Pin, PushPull};
use nrf52832_hal::pac::uarte0;
use nrf52832_hal::pac::UARTE0;
use nrf52832_hal::uarte::Pins;
use nrf52832_hal::Uarte;

pub fn init_uart0(
    rxd: Pin<Input<Floating>>,
    txd: Pin<Output<PushPull>>,
    uart: UARTE0,
) -> Uarte<UARTE0> {
    Uarte::new(
        uart,
        Pins {
            // https://github.com/noloerino/149-labs/blob/master/software/boards/buckler_revC/buckler.h
            txd,
            rxd,
            cts: None,
            rts: None,
        },
        uarte0::config::PARITY_A::EXCLUDED,
        uarte0::baudrate::BAUDRATE_A::BAUD115200,
    )
}

pub fn checksum(buf: &[u8]) -> u8 {
    let mut cs: u8 = 0;
    for i in 2..buf.len() {
        cs ^= buf[i];
    }
    cs
}
