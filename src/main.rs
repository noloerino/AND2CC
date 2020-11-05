#![no_main]
#![no_std]

mod buckler;
mod error;
mod kobuki;

// use embedded_hal::digital::v2::InputPin;
// use embedded_hal::digital::v2::OutputPin;
use buckler::lcd_display::LcdDisplay;
use buckler::lsm9ds1::Imu;
use core::default;
use core::fmt::Write;
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
    let mut spi1 = hal::spim::Spim::new(
        p.SPIM1,
        hal::spim::Pins {
            // https://github.com/lab11/buckler/blob/master/software/boards/buckler_revC/buckler.h
            sck: port0.p0_17.into_push_pull_output(Level::Low).degrade(),
            mosi: Some(port0.p0_15.into_push_pull_output(Level::Low).degrade()),
            miso: Some(port0.p0_16.into_floating_input().degrade()),
        },
        hal::spim::Frequency::M4,
        hal::spim::MODE_2,
        0,
    );
    // Initialize display
    let mut spi_cs = port0.p0_18.into_push_pull_output(Level::Low).degrade();
    let mut display = LcdDisplay::new(&mut spi1, &mut spi_cs, &mut delay).unwrap();
    display.row_0().write_str("Initializing...").unwrap();
    display.row_1().write_str("Blocking on base").unwrap();
    // Initialize IMU
    let twi0 = hal::twim::Twim::new(
        p.TWIM0,
        hal::twim::Pins {
            scl: port0.p0_19.into_floating_input().degrade(),
            sda: port0.p0_20.into_floating_input().degrade(),
        },
        hal::twim::Frequency::K100,
    );
    let mut imu = Imu::new(twi0, p.TIMER1);
    let mut sensors = Sensors::default();
    let mut state = DriveState::default();
    rprintln!("[Init] Initialization complete; waiting for first sensor poll");
    const DRIVE_DIST: f32 = 0.2;
    const REVERSE_DIST: f32 = -0.1;
    const DRIVE_SPEED: i16 = 70;
    // Block until UART connection is made
    SensorPoller::poll(&mut uart, &mut sensors).unwrap();
    rprintln!("[Init] First sensor poll succeedeed; connected to Romi");
    loop {
        delay.delay_ms(1u8);
        display
            .row_0()
            .write_str(match state {
                DriveState::Off => "Off",
                DriveState::Forward { .. } => "Forward",
                DriveState::Reverse { .. } => "Reverse",
                DriveState::TurnCcw => "Turn CCW",
            })
            .unwrap();
        display.row_1().write_str("").unwrap();
        SensorPoller::poll(&mut uart, &mut sensors).unwrap();
        // let accel = imu.read_accel().unwrap();
        // rprintln!("x_accel: {:.2}", accel.x_axis);
        let mut actuator = Actuator::new(&mut uart);
        let is_button_pressed = sensors.is_button_pressed();
        match state {
            DriveState::Off => {
                if is_button_pressed {
                    state = DriveState::Forward {
                        last_encoder: sensors.left_wheel_encoder,
                        distance_traveled: 0.0,
                    };
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
                        state = DriveState::TurnCcw;
                        imu.start_gyro_integration();
                    } else {
                        actuator.drive_direct(DRIVE_SPEED, DRIVE_SPEED).unwrap();
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
                    let curr_encoder = sensors.left_wheel_encoder;
                    distance_traveled +=
                        measure_distance(curr_encoder, last_encoder, DriveDirection::Reverse);
                    if distance_traveled <= REVERSE_DIST {
                        state = DriveState::Off;
                    } else {
                        actuator.drive_direct(-DRIVE_SPEED, -DRIVE_SPEED).unwrap();
                        state = DriveState::Reverse {
                            last_encoder: curr_encoder,
                            distance_traveled,
                        }
                    }
                }
            }
            DriveState::TurnCcw => {
                let angle = fabs(imu.read_gyro_integration().unwrap().z_axis);
                display
                    .row_1()
                    .write_fmt(format_args!("angle: {:.1}", angle))
                    .unwrap();
                if is_button_pressed {
                    state = DriveState::Off;
                    imu.stop_gyro_integration();
                } else if angle >= 90.0 {
                    actuator.drive_direct(0, 0).unwrap();
                    // Add slight delay and repoll to let wheel stop
                    delay.delay_ms(100u16);
                    SensorPoller::poll(&mut uart, &mut sensors).unwrap();
                    state = DriveState::Forward {
                        last_encoder: sensors.left_wheel_encoder,
                        distance_traveled: 0.0,
                    };
                    imu.stop_gyro_integration();
                } else {
                    actuator.drive_direct(DRIVE_SPEED, -DRIVE_SPEED).unwrap();
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
