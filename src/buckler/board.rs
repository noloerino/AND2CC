//! Pin definitions for the Berkeley Buckler, revision C.
//! https://github.com/lab11/buckler/blob/master/software/boards/buckler_revC/buckler.h

use super::{lcd_display::*, lsm9ds1::*};
use crate::kobuki::{actuator::*, sensors::*};
use crate::pixy2::*;
use core::fmt::Write;
use nrf52832_hal::gpio::{Floating, Input, Level, Output, Pin, PullDown, PullUp, PushPull};
use nrf52832_hal::{delay, gpio, pac, spim, twim, uarte};

pub type Leds = (
    Pin<Output<PushPull>>,
    Pin<Output<PushPull>>,
    Pin<Output<PushPull>>,
);

pub struct Pins {
    // TODO add analog accel pins and grove headers
    // TODO IMU interrupt, light interrupt
    pub button_0: Pin<Input<PullUp>>,
    pub switch_0: Pin<Input<PullDown>>,
    pub sensors_twi: twim::Pins,
    // TODO factor out display pins into its own struct
    pub lcd_spi: spim::Pins,
    pub lcd_chip_sel: Pin<Output<PushPull>>,
    // These pins are repurposed from SD card interface
    pub pixy_spi: spim::Pins,
    pub pixy_chip_sel: Pin<Output<PushPull>>,
    pub leds: Leds,
    pub uart_rx: Pin<Input<Floating>>,
    pub uart_tx: Pin<Output<PushPull>>,
}

impl Pins {
    pub fn new(port0: pac::P0) -> Pins {
        let p = gpio::p0::Parts::new(port0);
        Pins {
            button_0: p.p0_28.into_pullup_input().degrade(),
            switch_0: p.p0_22.into_pulldown_input().degrade(),
            sensors_twi: twim::Pins {
                scl: p.p0_19.into_floating_input().degrade(),
                sda: p.p0_20.into_floating_input().degrade(),
            },
            lcd_spi: spim::Pins {
                sck: p.p0_17.into_push_pull_output(Level::Low).degrade(),
                mosi: Some(p.p0_15.into_push_pull_output(Level::Low).degrade()),
                miso: Some(p.p0_16.into_floating_input().degrade()),
            },
            lcd_chip_sel: p.p0_18.into_push_pull_output(Level::Low).degrade(),
            pixy_spi: spim::Pins {
                sck: p.p0_13.into_push_pull_output(Level::Low).degrade(),
                mosi: Some(p.p0_11.into_push_pull_output(Level::Low).degrade()),
                miso: Some(p.p0_12.into_floating_input().degrade()),
            },
            pixy_chip_sel: p.p0_14.into_push_pull_output(Level::Low).degrade(),
            leds: (
                p.p0_25.into_push_pull_output(Level::High).degrade(),
                p.p0_24.into_push_pull_output(Level::High).degrade(),
                p.p0_23.into_push_pull_output(Level::High).degrade(),
            ),
            uart_rx: p.p0_08.into_floating_input().degrade(),
            uart_tx: p.p0_06.into_push_pull_output(Level::High).degrade(),
        }
    }
}

/// Provides access to Buckler sensors, actuators, and pins not used elsewhere.
pub struct Board {
    uart: uarte::Uarte<pac::UARTE0>,
    pub delay: delay::Delay,
    pub display: LcdDisplay<pac::SPIM1>,
    pub imu: Imu<pac::TWIM0, pac::TIMER1>,
    pub sensors: Sensors,
    pub button_0: Pin<Input<PullUp>>,
    pub switch_0: Pin<Input<PullDown>>,
    pub leds: Leds,
    pub pixy: Pixy2<pac::SPIM2, pac::TIMER0>,
}

impl Board {
    pub fn new(p: pac::Peripherals, c: pac::CorePeripherals) -> Board {
        let pins = Pins::new(p.P0);
        let mut delay = delay::Delay::new(c.SYST);
        let uart = uarte::Uarte::new(
            p.UARTE0,
            uarte::Pins {
                rxd: pins.uart_rx,
                txd: pins.uart_tx,
                cts: None,
                rts: None,
            },
            pac::uarte0::config::PARITY_A::EXCLUDED,
            pac::uarte0::baudrate::BAUDRATE_A::BAUD115200,
        );
        let spi1 = spim::Spim::new(p.SPIM1, pins.lcd_spi, spim::Frequency::M4, spim::MODE_2, 0);

        let spi_pixy =
            spim::Spim::new(p.SPIM2, pins.pixy_spi, spim::Frequency::M2, spim::MODE_3, 0);

        // Initialize display
        let mut display = LcdDisplay::new(spi1, pins.lcd_chip_sel, &mut delay).unwrap();
        display.row_0().write_str("Initializing...").unwrap();
        display.row_1().write_str("Blocking on base").unwrap();
        // Initialize IMU
        let twi0 = twim::Twim::new(p.TWIM0, pins.sensors_twi, twim::Frequency::K100);
        let imu = Imu::new(twi0, p.TIMER1);
        let sensors = Sensors::default();
        let pixy = Pixy2::new(spi_pixy, pins.pixy_chip_sel, p.TIMER0).unwrap();
        Board {
            uart,
            delay,
            display,
            imu,
            sensors,
            button_0: pins.button_0,
            switch_0: pins.switch_0,
            leds: pins.leds,
            pixy,
        }
    }

    pub fn poll_sensors(&mut self) -> Result<(), uarte::Error> {
        SensorPoller::poll(&mut self.uart, &mut self.sensors)
    }

    pub fn actuator(&mut self) -> Actuator<pac::UARTE0> {
        Actuator::new(&mut self.uart)
    }
}
