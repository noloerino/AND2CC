#![no_main]
#![no_std]

mod error;
mod kobuki;

// use embedded_hal::digital::v2::InputPin;
// use embedded_hal::digital::v2::OutputPin;
use core::default;
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use kobuki::actuator::Actuator;
use kobuki::sensors::{SensorPoller, Sensors};
use kobuki::utilities;
use nrf52832_hal as hal;
use nrf52832_hal::delay;
use nrf52832_hal::gpio::Level;
use rtt_target::{rprintln, rtt_init_print};

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
    let port0 = hal::gpio::p0::Parts::new(p.P0);
    // let button = port0.p0_28.into_pullup_input();
    // let mut led = port0.p0_23.into_push_pull_output(Level::Low);
    let c = hal::pac::CorePeripherals::take().unwrap();
    let mut delay = delay::Delay::new(c.SYST);
    let mut uart = utilities::init_uart0(
        port0.p0_08.into_floating_input().degrade(),
        port0.p0_06.into_push_pull_output(Level::High).degrade(),
        p.UARTE0,
    );
    let mut sensors = Sensors::default();
    let mut state = DriveState::default();
    rprintln!("Initialization complete");
    const DRIVE_DIST: f32 = 0.5;
    const REVERSE_DIST: f32 = -0.1;
    loop {
        delay.delay_ms(1u16);
        SensorPoller::poll(&mut uart, &mut sensors).unwrap();
        let mut actuator = Actuator::new(&mut uart);
        let is_button_pressed = sensors.is_button_pressed();
        match state {
            DriveState::Off => {
                if is_button_pressed {
                    state = DriveState::Forward {
                        last_encoder: sensors.left_wheel_encoder,
                        distance_traveled: 0.0,
                    };
                    rprintln!("Begin drive");
                } else {
                    actuator.drive_direct(0, 0).unwrap();
                }
            }
            DriveState::Forward {
                last_encoder,
                mut distance_traveled,
            } => {
                if is_button_pressed {
                    state = DriveState::Off;
                    rprintln!("Drive off");
                } else if sensors.is_bump() {
                    rprintln!("Bump! (timestamp {})", sensors.timestamp);
                    actuator.drive_direct(0, 0).unwrap();
                    // Add slight delay and repoll to let wheel stop
                    delay.delay_ms(100u16);
                    SensorPoller::poll(&mut uart, &mut sensors).unwrap();
                    state = DriveState::Reverse {
                        last_encoder: sensors.left_wheel_encoder,
                        distance_traveled: 0.0,
                    };
                } else {
                    let curr_encoder = sensors.left_wheel_encoder;
                    distance_traveled +=
                        measure_distance(curr_encoder, last_encoder, DriveDirection::Forward);
                    if distance_traveled >= DRIVE_DIST {
                        state = DriveState::Off;
                        rprintln!("Traveled {}, stopping", distance_traveled);
                    } else {
                        actuator.drive_direct(100, 100).unwrap();
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
                    rprintln!("Drive off");
                } else {
                    let curr_encoder = sensors.left_wheel_encoder;
                    distance_traveled +=
                        measure_distance(curr_encoder, last_encoder, DriveDirection::Reverse);
                    if distance_traveled <= REVERSE_DIST {
                        state = DriveState::Off;
                        rprintln!("Backed up {}, stopping", distance_traveled);
                    } else {
                        actuator.drive_direct(-100, -100).unwrap();
                        state = DriveState::Reverse {
                            last_encoder: curr_encoder,
                            distance_traveled,
                        }
                    }
                }
            } // DriveState::TurnCcw => {
              //     if button.is_low().unwrap() {
              //         state = DriveState::Off;
              //         rprintln!("Drive off");
              //     } else {
              //         rprintln!("turn ctr: {}", ctr);
              //         if let Err(e) = actuator.drive_direct(-100, 100) {
              //             rprintln!("Error attempting drive: {:?}", e);
              //         }
              //     }
              // }
        }
    }
}
