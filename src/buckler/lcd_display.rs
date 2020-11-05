//! SPI interface to the NHD-0216KZW display.
//! https://github.com/lab11/buckler/tree/master/software/libraries/nhd_display

use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use nrf52832_hal::delay;
use nrf52832_hal::gpio::{Output, Pin, PushPull};
use nrf52832_hal::spim;
use nrf52832_hal::Spim;

pub struct LcdDisplay<'a, T> {
    spi: &'a mut Spim<T>,
    chip_select: &'a mut Pin<Output<PushPull>>,
}

impl<'a, T: spim::Instance> LcdDisplay<'a, T> {
    /// Initializees the display. Corresponds to display_init
    pub fn new<'b>(
        spi: &'a mut Spim<T>,
        chip_select: &'a mut Pin<Output<PushPull>>,
        delay: &'b mut delay::Delay,
    ) -> Result<Self, spim::Error> {
        // We cannot pass the array directly to the function because DMA requires
        // the buffer to be in data RAM
        let mut buf: [u8; 2];
        // Set function 8-bit mode
        buf = [0b1110, 0];
        spi.write(chip_select, &buf)?;
        delay.delay_ms(10u8);
        // Turn display off
        buf = [0b10, 0];
        spi.write(chip_select, &buf)?;
        delay.delay_ms(10u8);
        // Clear display
        buf = [0, 0b0100_0000];
        spi.write(chip_select, &buf)?;
        delay.delay_ms(10u8);
        // Set entry mode to increment right no shift
        buf = [1, 0b1000_0000];
        spi.write(chip_select, &buf)?;
        delay.delay_ms(10u8);
        // Move cursor home
        buf = [0, 0b1000_0000];
        spi.write(chip_select, &buf)?;
        delay.delay_ms(10u8);
        // Move cursor home
        buf = [0b11, 0b0100_0000];
        spi.write(chip_select, &buf)?;
        delay.delay_ms(10u8);
        // Read the status bit
        buf = [0b0100_0000, 0];
        spi.write(chip_select, &buf)?;
        delay.delay_ms(10u8);
        Ok(LcdDisplay { spi, chip_select })
    }

    fn write_byte(&mut self, c: u8) -> Result<(), spim::Error> {
        let base_char_0: u8 = 0b1000_0000;
        let base_char_1: u8 = 0;
        // Write the character
        // The top 6 bits then the bottom two bits
        let write_0 = base_char_0 | (c >> 2);
        let write_1 = base_char_1 | (c << 6);
        self.spi.write(self.chip_select, &[write_0, write_1])
    }

    fn do_write(&mut self, msg: &str) -> Result<(), spim::Error> {
        // Now write the characters of the string then clear the line
        for &c in msg.as_bytes() {
            self.write_byte(c)?;
        }
        for _ in msg.len()..16 {
            self.write_byte(' ' as u8)?;
        }
        Ok(())
    }

    pub fn write_row_0(&mut self, msg: &str) -> Result<(), spim::Error> {
        if msg.len() > 16 {
            return Err(spim::Error::TxBufferTooLong);
        }
        // Set the screen to the correct character (0)
        let buf = [0b0010_0000, 0];
        self.spi.write(self.chip_select, &buf)?;
        self.do_write(msg)
    }

    pub fn write_row_1(&mut self, msg: &str) -> Result<(), spim::Error> {
        if msg.len() > 16 {
            return Err(spim::Error::TxBufferTooLong);
        }
        // Set the screen to the correct character (0x40)
        let buf = [0b0011_0000, 0];
        self.spi.write(self.chip_select, &buf)?;
        self.do_write(msg)
    }
}
