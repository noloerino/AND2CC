//! Pin definitions for the Berkeley Buckler, revision C.
//! https://github.com/lab11/buckler/blob/master/software/boards/buckler_revC/buckler.h

use super::{lcd_display::*, lsm9ds1::*};
use crate::kobuki::{actuator::*, sensors::*};
use crate::pixy2::*;
use core::fmt::Write;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use nrf52832_hal::gpio::{Floating, Input, Level, Output, Pin, PullDown, PullUp, PushPull};
use nrf52832_hal::{gpio, pac, spim, twim, uarte};
use rtt_target::rprintln;

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
    // These pins are repurposed from Grove A1/A0, respectively
    pub dock_power: Pin<Output<PushPull>>,
    pub dock_detect: Pin<Input<PullDown>>,
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
            dock_power: p.p0_03.into_push_pull_output(Level::High).degrade(),
            dock_detect: p.p0_04.into_pulldown_input().degrade(),
        }
    }
}

/// Manages the UARTE unit.
///
/// Methods requiring UART communication have unsafe implementation details because we wish to
/// activate/deactivate the UART module to save power; since instantiating Uarte requires taking
/// ownership, we end up having to coerce &mut UartMngr to UartMngr.
pub struct UartMngr {
    uart: pac::UARTE0,
    uart_rx: Pin<Input<Floating>>,
    uart_tx: Pin<Output<PushPull>>,
}

impl UartMngr {
    pub fn poll_sensors(&mut self, sensors: &mut Sensors) -> Result<(), uarte::Error> {
        // To allow the API to take an &mut self rather than consuming self, we need to do some
        // unsafe things
        use core::ptr;
        let mut uarte = unsafe { ptr::read(self) }.into_uarte();
        let r = SensorPoller::poll(&mut uarte, sensors);
        // Release UARTE resource
        unsafe {
            *(self as *mut Self) = uarte.into();
        }
        r
    }

    pub fn into_uarte(self) -> uarte::Uarte<pac::UARTE0> {
        uarte::Uarte::new(
            self.uart,
            uarte::Pins {
                rxd: self.uart_rx,
                txd: self.uart_tx,
                cts: None,
                rts: None,
            },
            pac::uarte0::config::PARITY_A::EXCLUDED,
            pac::uarte0::baudrate::BAUDRATE_A::BAUD115200,
        )
    }

    pub fn drive_direct(
        &mut self,
        left_wheel_speed: i16,
        right_wheel_speed: i16,
    ) -> Result<(), uarte::Error> {
        // To allow the API to take an &mut self rather than consuming self, we need to do some
        // unsafe things
        use core::ptr;
        let mut uarte = unsafe { ptr::read(self) }.into_uarte();
        let r = Actuator::new(&mut uarte).drive_direct(left_wheel_speed, right_wheel_speed);
        // Release UARTE resource
        unsafe {
            *(self as *mut Self) = uarte.into();
        }
        r
    }
}

impl From<uarte::Uarte<pac::UARTE0>> for UartMngr {
    fn from(other: uarte::Uarte<pac::UARTE0>) -> Self {
        let (uart, uarte::Pins { rxd, txd, .. }) = other.free();
        Self {
            uart,
            uart_rx: rxd,
            uart_tx: txd,
        }
    }
}

/// Provides access to Buckler sensors, actuators, and pins not used elsewhere.
pub struct Board {
    uart_mngr: UartMngr,
    pub display: LcdDisplay<pac::SPIM1>,
    pub imu: Imu<pac::TWIM0, pac::TIMER1>,
    pub sensors: Sensors,
    pub button_0: Pin<Input<PullUp>>,
    pub switch_0: Pin<Input<PullDown>>,
    pub leds: Leds,
    pub pixy: Pixy2<pac::SPIM2, pac::TIMER2>,
    // Even though we no longer need the docking power pin once it's been driven high,
    // we need to still maintain ownership over it to prevent other objects from reusing it.
    dock_power: Pin<Output<PushPull>>,
    dock_detect: Pin<Input<PullDown>>,
}

#[allow(non_snake_case)]
pub struct BoardInitResources {
    pub P0: pac::P0,
    pub UARTE0: pac::UARTE0,
    pub SPIM1: pac::SPIM1,
    pub SPIM2: pac::SPIM2,
    pub TWIM0: pac::TWIM0,
    pub TIMER1: pac::TIMER1,
    pub TIMER2: pac::TIMER2,
}

impl Board {
    pub fn new(p: BoardInitResources, _c: pac::CorePeripherals) -> Board {
        let pins = Pins::new(p.P0);
        let uart_mngr = UartMngr {
            uart: p.UARTE0,
            uart_rx: pins.uart_rx,
            uart_tx: pins.uart_tx,
        };
        let spi1 = spim::Spim::new(p.SPIM1, pins.lcd_spi, spim::Frequency::M4, spim::MODE_2, 0);

        let spi_pixy =
            spim::Spim::new(p.SPIM2, pins.pixy_spi, spim::Frequency::M2, spim::MODE_3, 0);

        // Initialize display
        let mut display = LcdDisplay::new(spi1, pins.lcd_chip_sel).unwrap();
        display.row_0().write_str("Initializing...").unwrap();
        display.row_1().write_str("Blocking on base").unwrap();
        // Initialize IMU
        let twi0 = twim::Twim::new(p.TWIM0, pins.sensors_twi, twim::Frequency::K100);
        let imu = Imu::new(twi0, p.TIMER1);
        let mut sensors = Sensors::default();
        // Block until UART connection is made
        // This goes before pixy connection to make sure pixy is powered as well
        rprintln!("[Init] Waiting for first sensor poll from Romi");
        let mut uarte = uart_mngr.into_uarte();
        SensorPoller::poll(&mut uarte, &mut sensors).unwrap();
        rprintln!("[Init] First sensor poll succeedeed; connected to Romi");
        rprintln!("[Init] Blocking on pixy...");
        let pixy = Pixy2::new(spi_pixy, pins.pixy_chip_sel, p.TIMER2).unwrap();
        rprintln!("[Init] Connected to pixy...");
        let mut b = Board {
            uart_mngr: uarte.into(),
            display,
            imu,
            sensors,
            button_0: pins.button_0,
            switch_0: pins.switch_0,
            leds: pins.leds,
            pixy,
            dock_power: pins.dock_power,
            dock_detect: pins.dock_detect,
        };
        b.dock_power.set_high().unwrap();
        b.display.row_0().write_str("Buckler online!").ok();
        b.display.row_1().clear().ok();
        b.uart_mngr.drive_direct(0, 0).ok();
        rprintln!("[Init] Initialization complete");
        b
    }

    /// Returns true if a connection is detected between dock_power and dock_detect by reading
    /// a digital value off the detect pin. This abstraction is here to allow us to switch between
    /// pull up and pull down as needed.
    pub fn is_docked(&self) -> bool {
        // Detect is pull down, so check if it's high
        self.dock_detect.is_high().unwrap()
    }

    pub fn poll_sensors(&mut self) -> Result<(), uarte::Error> {
        self.uart_mngr.poll_sensors(&mut self.sensors)
    }

    pub fn drive_direct(
        &mut self,
        left_wheel_speed: i16,
        right_wheel_speed: i16,
    ) -> Result<(), uarte::Error> {
        self.uart_mngr
            .drive_direct(left_wheel_speed, right_wheel_speed)
    }
}
