#![no_main]
#![no_std]

mod buckler;
mod error;
mod kobuki;

use core::default;
use core::fmt::Write;
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use nrf52832_hal as hal;
use rtt_target::{rprintln, rtt_init_print};

#[derive(PartialEq)]
enum DriveState {
    Off,
    Forward {
        last_encoder: u16,
        distance_traveled: f32,
    },
    Reverse {
        last_encoder: u16,
        distance_traveled: f32,
    },
    TurnCcw,
}

impl default::Default for DriveState {
    fn default() -> Self {
        DriveState::Off
    }
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

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    let p = hal::pac::Peripherals::take().unwrap();
    let c = hal::pac::CorePeripherals::take().unwrap();
    let mut b = buckler::board::Board::new(p, c);
    let mut state = DriveState::default();
    rprintln!("[Init] Initialization complete; waiting for first sensor poll from Romi");
    const DRIVE_DIST: f32 = 0.2;
    const REVERSE_DIST: f32 = -0.1;
    const DRIVE_SPEED: i16 = 70;
    // Block until UART connection is made
    b.poll_sensors().unwrap();
    rprintln!("[Init] First sensor poll succeedeed; connected to Romi");
    loop {
        b.delay.delay_ms(1u8);
        b.display
            .row_0()
            .write_str(match state {
                DriveState::Off => "Off",
                DriveState::Forward { .. } => "Forward",
                DriveState::Reverse { .. } => "Reverse",
                DriveState::TurnCcw => "Turn CCW",
            })
            .ok();
        if state != DriveState::TurnCcw {
            b.display.row_1().write_str("").ok();
        }
        b.poll_sensors().unwrap();
        let is_button_pressed = b.sensors.is_button_pressed();
        match state {
            DriveState::Off => {
                if is_button_pressed {
                    state = DriveState::Forward {
                        last_encoder: b.sensors.left_wheel_encoder,
                        distance_traveled: 0.0,
                    };
                } else {
                    b.actuator().drive_direct(0, 0).ok();
                }
            }
            DriveState::Forward {
                last_encoder,
                mut distance_traveled,
            } => {
                if is_button_pressed {
                    state = DriveState::Off;
                } else if b.sensors.is_bump() {
                    rprintln!("Bump! (timestamp {})", b.sensors.timestamp);
                    b.actuator().drive_direct(0, 0).unwrap();
                    // Add slight delay and repoll to let wheel stop
                    b.delay.delay_ms(100u16);
                    b.poll_sensors().unwrap();
                    state = DriveState::Reverse {
                        last_encoder: b.sensors.left_wheel_encoder,
                        distance_traveled: 0.0,
                    };
                } else {
                    let curr_encoder = b.sensors.left_wheel_encoder;
                    distance_traveled +=
                        measure_distance(curr_encoder, last_encoder, DriveDirection::Forward);
                    if distance_traveled >= DRIVE_DIST {
                        state = DriveState::TurnCcw;
                        b.imu.start_gyro_integration();
                    } else {
                        b.actuator().drive_direct(DRIVE_SPEED, DRIVE_SPEED).ok();
                        state = DriveState::Forward {
                            last_encoder: curr_encoder,
                            distance_traveled,
                        }
                    }
                }
            }
            DriveState::Reverse {
                last_encoder,
                mut distance_traveled,
            } => {
                if is_button_pressed {
                    state = DriveState::Off;
                } else {
                    let curr_encoder = b.sensors.left_wheel_encoder;
                    distance_traveled +=
                        measure_distance(curr_encoder, last_encoder, DriveDirection::Reverse);
                    if distance_traveled <= REVERSE_DIST {
                        state = DriveState::Off;
                    } else {
                        b.actuator().drive_direct(-DRIVE_SPEED, -DRIVE_SPEED).ok();
                        state = DriveState::Reverse {
                            last_encoder: curr_encoder,
                            distance_traveled,
                        }
                    }
                }
            }
            DriveState::TurnCcw => {
                let angle = fabs(b.imu.read_gyro_integration().unwrap().z_axis);
                b.display
                    .row_1()
                    .write_fmt(format_args!("angle: {:.1}", angle))
                    .ok();
                if is_button_pressed {
                    state = DriveState::Off;
                    b.imu.stop_gyro_integration();
                } else if angle >= 90.0 {
                    b.actuator().drive_direct(0, 0).ok();
                    // Add slight delay and repoll to let wheel stop
                    b.delay.delay_ms(100u16);
                    b.poll_sensors().ok();
                    state = DriveState::Forward {
                        last_encoder: b.sensors.left_wheel_encoder,
                        distance_traveled: 0.0,
                    };
                    b.imu.stop_gyro_integration();
                } else {
                    b.actuator().drive_direct(DRIVE_SPEED, -DRIVE_SPEED).ok();
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
