//! SPI interface to the NHD-0216KZW display.
//! https://github.com/lab11/buckler/tree/master/software/libraries/nhd_display

use arrayvec::ArrayString;
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use nrf52832_hal::delay;
use nrf52832_hal::gpio::{Output, Pin, PushPull};
use nrf52832_hal::spim;
use nrf52832_hal::Spim;

/// Provides access to the LCD display.
/// The row fields implement fmt::Write.
/// The rows must not outlive the SPI and CS pins.
pub struct LcdDisplay<'a, T> {
    spi: &'a mut Spim<T>,
    chip_select: &'a mut Pin<Output<PushPull>>,
}

pub struct Row<'a, T> {
    spi: &'a mut Spim<T>,
    chip_select: &'a mut Pin<Output<PushPull>>,
    tgt_char: u8,
}

impl<T: spim::Instance> Row<'_, T> {
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
            self.write_byte(b' ')?;
        }
        Ok(())
    }
}

impl<T: spim::Instance> core::fmt::Write for Row<'_, T> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        if s.len() > 16 {
            panic!(
                "Attempted to write string of len {} to display (max 16)",
                s.len()
            );
        }
        // Set the screen to the correct character (0x40)
        let buf = [self.tgt_char, 0];
        self.spi.write(self.chip_select, &buf).unwrap();
        self.do_write(s).unwrap();
        Ok(())
    }

    /// Writes a format string produced by the format_args! macro.
    ///
    /// The default implmentation of this method makes multiple calls to write_str,
    /// which results in multiple attempts to overwrite the same line of the display.
    /// To get around this, this implementation buffers the string (since the string
    /// must at most be length 16 anyway) before writing.
    fn write_fmt(&mut self, fmt: core::fmt::Arguments) -> core::fmt::Result {
        let mut buf = ArrayString::<[_; 16]>::new();
        core::fmt::write(&mut buf, fmt).unwrap();
        self.write_str(&buf)
    }
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

    pub fn row_0(&mut self) -> Row<T> {
        Row {
            tgt_char: 0b0010_0000,
            chip_select: self.chip_select,
            spi: self.spi,
        }
    }

    pub fn row_1(&mut self) -> Row<T> {
        Row {
            tgt_char: 0b0011_0000,
            chip_select: self.chip_select,
            spi: self.spi,
        }
    }
}
