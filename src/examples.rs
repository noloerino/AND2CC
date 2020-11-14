//! Minimal examples for buckler features.
//! Import and run each function to test stuff.

#![allow(dead_code)]

use crate::buckler::board::Board;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;

/// Lights all three LEDs if BUTTON0 is pressed.
pub fn blink(b: &mut Board) -> ! {
    loop {
        b.delay.delay_ms(1u8);
        if b.button_0.is_high().unwrap() {
            b.leds.0.set_high().ok();
            b.leds.1.set_high().ok();
            b.leds.2.set_high().ok();
        } else {
            b.leds.0.set_low().ok();
            b.leds.1.set_low().ok();
            b.leds.2.set_low().ok();
        }
    }
}

/// Writes to both display rows.
pub fn display(b: &mut Board) -> ! {
    use core::fmt::Write;
    loop {
        b.delay.delay_ms(1u8);
        b.display.row_0().write_str("this is row 0").ok();
        b.display.row_1().write_str("this is row 1").ok();
    }
}

pub fn pixy(b: &mut Board) -> ! {
    use core::fmt::Write;
    loop {
        b.delay.delay_ms(1u8);
        b.display.row_0().write_fmt(format_args!("{}", b.pixy.frame_height)).ok();
    }
}