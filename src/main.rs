#![no_main]
#![no_std]

mod buckler;
mod error;
mod examples;
mod kobuki;
mod pixy2;

use core::default;
use core::fmt::Write;
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use nrf52832_hal as hal;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};

const DETECT_RECALIBRATE_M: f32 = 1.0;
const DRIVE_SPEED: i16 = 70;

/// Top-level states of the FSM
#[derive(PartialEq)]
enum TopState {
    Off,
    Detect(DetectState),
    Dock,
    Drive,
}

impl default::Default for TopState {
    fn default() -> Self {
        TopState::Off
    }
}

/// Describes states for the detection phase.
#[derive(PartialEq)]
enum DetectState {
    /// Rotating to look for target
    Scan,
    /// Driving towards the target (should be facing backwards)
    Approach {
        last_encoder: u16,
        distance_traveled: f32,
    },
}

#[derive(PartialEq)]
enum DriveDirection {
    Forward,
    Reverse,
}

fn measure_distance(curr_encoder: u16, prev_encoder: u16, direction: DriveDirection) -> f32 {
    const CONVERSION: f32 = 0.0006108;
    // rprintln!("encoder: {} -> {}", prev_encoder, curr_encoder);
    CONVERSION
        * (if direction == DriveDirection::Forward {
            (if curr_encoder >= prev_encoder {
                curr_encoder - prev_encoder
            } else {
                curr_encoder + (u16::MAX - prev_encoder)
            }) as f32
        } else {
            -((if curr_encoder <= prev_encoder {
                prev_encoder - curr_encoder
            } else {
                prev_encoder + (u16::MAX - curr_encoder)
            }) as f32)
        })
}

#[app(device = nrf52832_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        b: buckler::board::Board,
    }

    // https://rtic.rs/0.5/book/en/by-example/app.html#init
    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_init_print!();
        let p: hal::pac::Peripherals = cx.device;
        let c: hal::pac::CorePeripherals = cx.core;
        // Enable pin reset before anything else
        if p.UICR.pselreset[0].read().bits() != 21 || p.UICR.pselreset[1].read().bits() != 21 {
            unsafe {
                p.UICR.pselreset[0].write(|w| w.pin().bits(21));
                p.UICR.pselreset[1].write(|w| w.pin().bits(21));
            }
            hal::pac::SCB::sys_reset();
        }
        let b = buckler::board::Board::new(p, c);
        init::LateResources { b }
    }

    #[idle(resources = [b])]
    fn idle(c: idle::Context) -> ! {
        let b = c.resources.b;
        // main_loop(b);
        // Comment out main_loop and uncomment these to run sanity examples
        // examples::blink(b);
        // examples::display(b);
        // examples::pixy(b);
        // examples::drive_forward(b);
        // examples::drive_reverse(b);
        // examples::dock_continuity(b);
        examples::target_block(b);
    }
};

fn main_loop(b: &mut buckler::board::Board) -> ! {
    use TopState::*;
    let mut top_state = TopState::default();
    loop {
        b.delay.delay_ms(1u8);
        b.poll_sensors().unwrap();
        // Can't just print debug string due to internal state
        b.display
            .row_0()
            .write_str(match top_state {
                Off => "Off",
                Detect(..) => "Detect",
                Dock => "Dock",
                Drive => "Drive",
            })
            .ok();
        let is_button_pressed = b.sensors.is_button_pressed();
        match top_state {
            Off => {
                b.display.row_1().write_str("").ok();
                if is_button_pressed {
                    rprintln!("Beginning detect phase");
                    b.imu.restart_gyro_integration();
                    top_state = TopState::Detect(DetectState::Scan);
                } else {
                    b.actuator().drive_direct(0, 0).ok();
                }
            }
            Detect(detect_state) => {
                // TODO transition to dock when proximity is detected
                if is_button_pressed {
                    top_state = TopState::Off;
                } else {
                    top_state = Detect(detect_state.react(b));
                }
            }
            _ => unimplemented!(),
        }
    }
}

impl DetectState {
    fn react(self, b: &mut buckler::board::Board) -> DetectState {
        use DetectState::*;
        // TODO hook up to pixy2
        // Hack to simulate detection after some number of cycles
        static mut N: u32 = 0;
        let tgt_detected: bool;
        unsafe {
            match self {
                Scan => {
                    N += 1;
                }
                _ => N = 0,
            }
            tgt_detected = N >= 200;
        }
        match self {
            Scan => {
                let angle = fabs(b.imu.read_gyro_integration().unwrap().z_axis);
                if tgt_detected {
                    rprintln!("Moving to approach at angle {}", angle);
                    b.imu.stop_gyro_integration();
                    Approach {
                        last_encoder: b.sensors.left_wheel_encoder,
                        distance_traveled: 0.0,
                    }
                } else {
                    // If this turns out to be flaky, ok() instead of unwrap() and retry
                    b.display
                        .row_1()
                        .write_fmt(format_args!("SCAN: {:.1}", angle))
                        .ok();
                    b.actuator().drive_direct(DRIVE_SPEED, -DRIVE_SPEED).ok();
                    Scan
                }
            }
            Approach {
                last_encoder,
                mut distance_traveled,
            } => {
                b.display
                    .row_1()
                    .write_fmt(format_args!("APPROACH: {:.1}m", distance_traveled))
                    .ok();
                if distance_traveled >= DETECT_RECALIBRATE_M {
                    rprintln!("Reorienting towards target");
                    b.imu.start_gyro_integration();
                    Scan
                } else {
                    // Drive robot backwards until 1m has been traversed, at which point we attempt
                    // to reorient just to be safe
                    b.actuator().drive_direct(-DRIVE_SPEED, -DRIVE_SPEED).ok();
                    let curr_encoder = b.sensors.left_wheel_encoder;
                    distance_traveled += fabs(measure_distance(
                        curr_encoder,
                        last_encoder,
                        DriveDirection::Reverse,
                    ));
                    Approach {
                        last_encoder: curr_encoder,
                        distance_traveled,
                    }
                }
            }
        }
    }
}

/// Apparently f32::abs is part of std, not core.
fn fabs(n: f32) -> f32 {
    if n >= 0.0 {
        n
    } else {
        -n
    }
}
