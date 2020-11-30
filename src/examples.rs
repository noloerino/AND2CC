//! Minimal examples for buckler features.
//! Import and run each function to test stuff.

#![allow(dead_code)]

use crate::buckler::board::Board;
use crate::pixy2;
use crate::utils::delay_ms;
use core::fmt::Write;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use rtt_target::rprintln;

/// Lights all three LEDs if BUTTON0 is pressed.
pub fn blink(b: &mut Board) -> ! {
    loop {
        delay_ms(1);
        // Button is high when not pressed
        // LEDs are active low
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
    loop {
        delay_ms(1);
        b.display.row_0().write_str("this is row 0").ok();
        b.display.row_1().write_str("this is row 1").ok();
    }
}

/// Writes pixy status to the display and continually prints block data to RTT.
pub fn pixy(b: &mut Board) -> ! {
    b.display
        .row_0()
        .write_fmt(format_args!("hw v: {}", b.pixy.version.hardware))
        .ok();
    b.display
        .row_1()
        .write_fmt(format_args!(
            "fw v: {}.{}",
            b.pixy.version.firmware_major, b.pixy.version.firmware_minor
        ))
        .ok();
    rprintln!(
        "frame height: {}, frame width: {}",
        b.pixy.frame_height,
        b.pixy.frame_width
    );
    loop {
        delay_ms(1);
        if b.pixy.get_blocks(false, pixy2::SigMap::ALL, 10).is_err() {
            continue;
        }
        for i in 0..b.pixy.num_blocks as usize {
            let block = b.pixy.blocks[i];
            rprintln!(
                "sig: {}, x: {}, y: {}, width: {}, height: {}, angle: {}, index: {}, age: {}",
                block.signature,
                block.x,
                block.y,
                block.width,
                block.height,
                block.angle,
                block.index,
                block.age
            );
        }
    }
}

pub fn drive_forward(b: &mut Board) -> ! {
    loop {
        delay_ms(1);
        b.drive_direct(100, 100).ok();
    }
}

pub fn drive_reverse(b: &mut Board) -> ! {
    loop {
        delay_ms(1);
        b.drive_direct(-100, -100).ok();
    }
}

/// Lights LED0 if continuity on the docking pins is detected.
/// Also prints to rtt continually to check how reliable the continuity is
pub fn dock_continuity(b: &mut Board) -> ! {
    loop {
        delay_ms(1);
        if b.is_docked() {
            rprintln!("docked");
            b.leds.0.set_low().ok();
        } else {
            rprintln!("NOT docked");
            b.leds.0.set_high().ok();
        }
    }
}

/// Used to track state for target_block below
#[derive(Debug)]
enum TargetState {
    Scan,
    Drive,
    Done,
}

/// Spins the robot in a circle until signature 1 is detected, at which point the robot will rotate
/// until m_x of the component is at approximately 0.
/// Line 1 of the display will print the state.
/// If signature 1 is detected, line 0 of the display will print its x/y coordinates.
/// When docking is detected, LED0 will be lit. After the initial docking contact, LED1 will
/// continually be lit, while LED0 will change in response to a disconnect.
pub fn target_block(b: &mut Board) -> ! {
    let mut state = TargetState::Scan;
    b.pixy.get_resolution().unwrap();
    let x_mid = b.pixy.frame_width >> 1;
    loop {
        // 1ms causes driving to screw up
        delay_ms(10);
        b.display
            .row_1()
            .write_fmt(format_args!("{:?}", state))
            .ok();
        match state {
            TargetState::Scan => {
                if b.pixy.get_blocks(false, pixy2::SigMap::ALL, 10).is_ok() {
                    let mut sig_1_block: Option<pixy2::Block> = None;
                    // Not very rustic :(
                    for i in 0..b.pixy.num_blocks as usize {
                        let block = b.pixy.blocks[i];
                        if block.signature == 1 {
                            sig_1_block = Some(block);
                            break;
                        }
                    }
                    const X_MID_TOL: u16 = 4;
                    if let Some(block) = sig_1_block {
                        rprintln!("ID'd block {:?}", block);
                        b.display
                            .row_0()
                            .write_fmt(format_args!("goto ({}, {})", block.x, block.y))
                            .ok();
                        if block.x < x_mid + X_MID_TOL || block.x > x_mid - X_MID_TOL {
                            // Proceed to drive
                            rprintln!("we drive");
                            state = TargetState::Drive;
                            b.drive_direct(0, 0).ok();
                            delay_ms(100);
                        } else {
                            // Make some adjustments
                            if block.x > x_mid {
                                // go left
                                b.drive_direct(-60, 60).ok();
                            } else {
                                // go right
                                b.drive_direct(60, -60).ok();
                            }
                        }
                    } else {
                        // Just keep spinning
                        b.display.row_0().write_str("").ok();
                        b.drive_direct(60, -60).ok();
                    }
                } else {
                    // Just keep spinning
                    b.display.row_0().write_str("").ok();
                    b.drive_direct(60, -60).ok();
                }
            }
            TargetState::Drive => {
                // Just go until docked
                b.display.row_0().write_str("zoom zoom").ok();
                if b.is_docked() {
                    rprintln!("done");
                    state = TargetState::Done;
                } else {
                    b.drive_direct(-60, -60).unwrap();
                }
            }
            TargetState::Done => {
                b.display.row_0().write_str("").ok();
                b.drive_direct(0, 0).ok();
                b.leds.1.set_low().ok();
                if b.is_docked() {
                    b.leds.0.set_low().ok();
                } else {
                    b.leds.0.set_high().ok();
                }
            }
        }
    }
}
