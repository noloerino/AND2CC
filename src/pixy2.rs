
use nrf52832_hal::gpio::{Output, Pin, PushPull};
use nrf52832_hal::{spim, Spim, timer, Timer}

const DEFAULT_ARGVAL: u32 = 0x8000_0000;
const BUFFERSIZE: u32 = 0x104;
const CHECKSUM_SYNC: u32 = 0xc1af;
const NO_CHECKSUM_SYNC: u32 = 0xc1ae;
const SEND_HEADER_SIZE: u32 = 4;
const MAX_PROGNAME: u32 = 33;

enum Pixy2Error {
    Error,
    Busy,
    ChecksumError,
    Timeout,
    ButtonOverride,
    ProgChanging
}

pub struct Pixy2<S: spim::Instance, T: timer::Instance> {
    spi: Spim<S>,
    chip_select: Pin<Output<PushPull>>,
    timer: Timer<T, timer::OneShot>,
    m_cs: bool,
    m_buf: [u8; BUFFERSIZE],
    m_length: u8,
    m_type: u8,
}

impl<S: spim::Instance, T: timer::Instance> Pixy2<S> {
    // spi must be 2MBs, MSBFIRST, MODE 3
    pub fn new(mut spi: Spim<S>, mut chip_select: Pin<Output<PushPull>>,
        timer_instance: T) -> Result<Self, Pixy2Error> {
        
        let pixy2 = Pixy2 {spi, chip_select, Timer::one_shot(timer_instance)};
        
        let t0 = pixy2.millis();
        while pixy2.millis() - t0 < 5000 {
            if pixy2.get_version().is_ok(){
                pixy2.get_resolution();
                return Ok(pixy2);
            }
            pixy2.timer.delay(5000); // delay 5000 microseconds.    
        }
        Err(Pixy2Error::Timeout)

    }

    pub fn get_version(self) -> Result(i8, Pixy2Error) {
        Err(Pixy2Error::Error)
    }

    fn millis(self) -> u32 {
        self.timer.read() / 1000;    
    }

    fn cumulative_sum(buf: &[u8]) -> u16 {
        let sum: u16 = 0;
        for byte in buf {
            sum += byte;
        }
        sum
    }

    fn get_sync(&mut self) -> Result((), Pixy2Error) {
        let mut buf: [u8; 1];
        let mut i: u8 = 0;
        let mut j: u8 = 0;
        let mut cprev: u8 = 0;
        let mut start: u16 = 0;

        loop {
            if self.spi.transfer(&mut self.chip_select, &buf).is_ok() {
                let c = buf[0];
                start = cprev;
                start |= c << 8;
                cprev = c;
                if (start == CHECKSUM_SYNC) {
                    self.m_cs = true;
                    return Ok(());
                }
                if (start == NO_CHECKSUM_SYNC) {
                    self.m_cs = false;
                    return Ok(());
                }
            }

            if i >= 4 {
                if j >= 4 {
                    return Err(Pixy2Error::Error);
                }
                self.timer.delay(25); // delay 25 microseconds
                ++j;
                i = 0;
            }
            ++i;
        }

        Err(Pixy2Error::Error)    
    }

    fn recv_packet(&mut self) -> Result((), Pixy2Error) {
        let mut cs_calc: u16 = 0;
        let mut cs_serial: u16 = 0;

        if let Err(error) = self.get_sync() {
            return Err(error);
        }

        if self.m_cs {
            if self.spi.transfer(&mut self.chip_select, &self.m_buf[0..4]).is_err() {
                return Err(Pixy2Error::Error);
            }    

            self.m_type = self.m_buf[0];
            self.m_length = self.m_buf[1];

            // TODO verify this is correct based on endianness.
            cs_serial = self.m_buf[3] << 8 | self.m_buf[2];

            if self.spi.transfer(&mut self.chip_select, &self.m_buf[0..self.m_length]).is_err() {
                return Err(Pixy2Error::Error);
            }

            cs_calc = cumulative_sum(self.m_buf[0..self.m_length]);
            if cs_serial != cs_calc {
                return Err(Pixy2Error::ChecksumError);
            }
        } else {
            if self.spi.transfer(&mut self.chip_select, &self.m_buf[0..2]).is_err() {
                return Err(Pixy2Error::Error);
            }
            self.m_type = self.m_buf[0];
            self.m_length = self.m_buf[1];

            if self.spi.transfer(&mut self.chip_select, &self.m_buf[0..self.m_length]).is_err() {
                return Err(Pixy2Error::Error);
            }
        }
        Ok(())
    }
}