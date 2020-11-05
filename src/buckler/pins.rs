//! Pin definitions for the Berkeley Buckler, revision C.
//! https://github.com/lab11/buckler/blob/master/software/boards/buckler_revC/buckler.h
use nrf52832_hal::gpio::{Floating, Input, Level, Output, Pin, PullDown, PullUp, PushPull};
use nrf52832_hal::{gpio, pac, spim, twim};

pub struct Pins {
    // TODO add analog accel pins and grove headers
    // TODO IMU interrupt, light interrupt
    // TODO SD card
    pub button_0: Pin<Input<PullUp>>,
    pub switch_0: Pin<Input<PullDown>>,
    pub sensors_twi: twim::Pins,
    // TODO factor out display pins into its own struct
    pub lcd_spi: spim::Pins,
    pub lcd_chip_sel: Pin<Output<PushPull>>,
    pub leds: (
        Pin<Output<PushPull>>,
        Pin<Output<PushPull>>,
        Pin<Output<PushPull>>,
    ),
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
            leds: (
                p.p0_25.into_push_pull_output(Level::Low).degrade(),
                p.p0_24.into_push_pull_output(Level::Low).degrade(),
                p.p0_23.into_push_pull_output(Level::Low).degrade(),
            ),
            uart_rx: p.p0_08.into_floating_input().degrade(),
            uart_tx: p.p0_06.into_push_pull_output(Level::High).degrade(),
        }
    }
}
