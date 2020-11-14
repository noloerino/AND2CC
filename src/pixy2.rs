use core::convert::{From, TryFrom, TryInto};
use nrf52832_hal::gpio::{Output, Pin, PushPull};
use nrf52832_hal::{spim, timer, Spim, Timer};

const DEFAULT_ARGVAL: u32 = 0x8000_0000;
const BUFFERSIZE: usize = 0x104;
const CHECKSUM_SYNC: u16 = 0xc1af;
const NO_CHECKSUM_SYNC: u16 = 0xc1ae;
const SEND_HEADER_SIZE: usize = 4;
const MAX_PROGNAME: u32 = 33;

const TYPE_REQUEST_CHANGE_PROG: u8 = 0x02;

const TYPE_REQUEST_RESOLUTION: u8 = 0x0c;
const TYPE_RESPONSE_RESOLUTION: u8 = 0x0d;

const TYPE_REQUEST_VERSION: u8 = 0x0e;
const TYPE_RESPONSE_VERSION: u8 = 0x0f;

const TYPE_RESPONSE_RESULT: u8 = 0x01;
const TYPE_RESPONSE_ERROR: u8 = 0x03;
const TYPE_REQUEST_BRIGHTNESS: u8 = 0x10;
const TYPE_REQUEST_SERVO: u8 = 0x12;
const TYPE_REQUEST_LED: u8 = 0x14;
const TYPE_REQUEST_LAMP: u8 = 0x16;
const TYPE_REQUEST_FPS: u8 = 0x18;

#[derive(Debug)]
pub enum Pixy2Error {
    Error,
    Busy,
    ChecksumError,
    Timeout,
    ButtonOverride,
    ProgChanging,
}

#[repr(C)]
pub struct Version {
    hardware: u16,
    firmware_major: u8,
    firmware_minor: u8,
    firmware_build: u16,
    firmware_type: [u8; 10],
}

impl Version {
    fn from(bytes: &[u8; 16]) -> Version {
        Version {
            hardware: u16::from_le_bytes(bytes[0..2].try_into().unwrap()),
            firmware_major: u8::from_le(bytes[2]),
            firmware_minor: u8::from_le(bytes[3]),
            firmware_build: u16::from_le_bytes(bytes[4..6].try_into().unwrap()),
            firmware_type: bytes[6..].try_into().unwrap(),
        }
    }
}

pub struct Pixy2<S: spim::Instance, T: timer::Instance> {
    spi: Spim<S>,
    chip_select: Pin<Output<PushPull>>,
    timer: T,
    m_cs: bool,
    m_buf: [u8; BUFFERSIZE],
    m_length: u8,
    m_type: u8,
    version: Version,
    frame_width: u16,
    pub frame_height: u16,
}

impl<S: spim::Instance, T: timer::Instance> Pixy2<S, T> {
    // spi must be 2MBs, MSBFIRST, MODE 3
    pub fn new(
        mut spi: Spim<S>,
        mut chip_select: Pin<Output<PushPull>>,
        timer_instance: T,
    ) -> Result<Self, Pixy2Error> {
        let mut pixy2 = Pixy2 {
            spi: spi,
            chip_select: chip_select,
            timer: timer_instance,
            m_cs: false,
            m_buf: [0; BUFFERSIZE],
            m_length: 0,
            m_type: 0,
            version: Version::from(&[0; 16]),
            frame_width: 0,
            frame_height: 0,
        };

        pixy2.timer.disable_interrupt();
        pixy2.timer.set_oneshot();
        pixy2.timer.timer_start(u32::MAX);
        let t0 = pixy2.millis();
        while pixy2.millis() < 5000 + t0 {
            if pixy2.get_version().is_ok() {
                pixy2.get_resolution()?;
                return Ok(pixy2);
            }
            pixy2.delay_us(5000); // delay 5000 microseconds.
        }
        Err(Pixy2Error::Timeout)
    }

    fn millis(&self) -> u32 {
        self.timer.read_counter() / 1000
    }

    fn delay_us(&self, duration: u32) {
        let t0 = self.timer.read_counter();
        while self.timer.read_counter() < t0 + duration {}
    }

    fn get_sync(&mut self) -> Result<(), Pixy2Error> {
        let mut buf: [u8; 1] = [0];
        let mut i: u8 = 0;
        let mut j: u8 = 0;
        let mut cprev: u8 = 0;
        let mut start: u16;

        // This is needed for making the transmit empty
        // otherwise we'd run into a not in DMA region runtime error
        let empty_tx: [u8; 0] = [];
        loop {
            if self
                .spi
                .transfer_split_uneven(&mut self.chip_select, &empty_tx, &mut buf)
                .is_ok()
            {
                let c = buf[0];
                start = u16::from(cprev);
                start |= u16::from(c) << 8;
                cprev = c;
                if start == CHECKSUM_SYNC {
                    self.m_cs = true;
                    return Ok(());
                }
                if start == NO_CHECKSUM_SYNC {
                    self.m_cs = false;
                    return Ok(());
                }
            }

            if i >= 4 {
                if j >= 4 {
                    return Err(Pixy2Error::Error);
                }
                self.delay_us(25); // delay 25 microseconds
                j += 1;
                i = 0;
            }
            i += 1;
        }

        Err(Pixy2Error::Error)
    }

    fn recv_packet(&mut self) -> Result<(), Pixy2Error> {
        if let Err(error) = self.get_sync() {
            return Err(error);
        }

        // This is needed for making the transmit empty
        // otherwise we'd run into a not in DMA region runtime error
        let empty_tx: [u8; 0] = [];
        if self.m_cs {
            self.spi
                .transfer_split_uneven(&mut self.chip_select, &empty_tx, &mut self.m_buf[0..4])
                .map_err(|_| Pixy2Error::Error)?;

            self.m_type = self.m_buf[0];
            self.m_length = self.m_buf[1];

            // TODO verify this is correct based on endianness.
            let cs_serial = u16::from_le_bytes(self.m_buf[2..4].try_into().unwrap());
            self.spi
                .transfer_split_uneven(
                    &mut self.chip_select,
                    &empty_tx,
                    &mut self.m_buf[0..self.m_length as usize],
                )
                .map_err(|_| Pixy2Error::Error)?;

            let cs_calc: u16 = cumulative_sum(&self.m_buf[0..self.m_length as usize]);
            if cs_serial != cs_calc {
                return Err(Pixy2Error::ChecksumError);
            }
        } else {
            self.spi
                .transfer_split_uneven(&mut self.chip_select, &empty_tx, &mut self.m_buf[0..2])
                .map_err(|_| Pixy2Error::Error)?;
            self.m_type = self.m_buf[0];
            self.m_length = self.m_buf[1];
            self.spi
                .transfer_split_uneven(
                    &mut self.chip_select,
                    &empty_tx,
                    &mut self.m_buf[0..self.m_length as usize],
                )
                .map_err(|_| Pixy2Error::Error)?;
        }
        Ok(())
    }

    fn send_packet(&mut self) -> Result<(), Pixy2Error> {
        self.m_buf[0..2].copy_from_slice(&NO_CHECKSUM_SYNC.to_le_bytes());
        self.m_buf[2] = self.m_type;
        self.m_buf[3] = self.m_length;
        self.spi
            .write(
                &mut self.chip_select,
                &self.m_buf[0..self.m_length as usize + SEND_HEADER_SIZE],
            )
            .map_err(|_| Pixy2Error::Error)
    }

    pub fn get_version(&mut self) -> Result<u8, Pixy2Error> {
        self.m_length = 0;
        self.m_type = TYPE_REQUEST_VERSION;
        self.send_packet()?;
        if self.recv_packet().is_ok() {
            if self.m_type == TYPE_RESPONSE_VERSION {
                self.version = Version::from(self.m_buf[0..16].try_into().unwrap());
                return Ok(self.m_length);
            } else if self.m_type == TYPE_RESPONSE_ERROR {
                return Err(Pixy2Error::Busy);
            }
        }
        Err(Pixy2Error::Error)
    }

    fn get_resolution(&mut self) -> Result<(), Pixy2Error> {
        self.m_length = 1;
        self.m_buf[SEND_HEADER_SIZE + 0] = 0;
        self.m_type = TYPE_REQUEST_RESOLUTION;
        self.send_packet()?;
        self.recv_packet()?;
        if self.m_type != TYPE_RESPONSE_RESOLUTION {
            Err(Pixy2Error::Error)
        } else {
            self.frame_width = u16::from_le_bytes(self.m_buf[0..2].try_into().unwrap());
            self.frame_height = u16::from_le_bytes(self.m_buf[2..4].try_into().unwrap());
            Ok(())
        }
    }
}

fn cumulative_sum(buf: &[u8]) -> u16 {
    let mut sum: u16 = 0;
    for byte in buf {
        sum += u16::from(*byte);
    }
    sum
}
