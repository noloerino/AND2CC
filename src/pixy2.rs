use core::convert::{From, TryInto, TryFrom};
use nrf52832_hal::gpio::{Output, Pin, PushPull};
use nrf52832_hal::{spim, timer, Spim};
use bitflags::bitflags;

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

// I have no clue what CCC stands for but its general prefix for
// object detection related code.
const CCC_MAX_SIGNATURE: u32 = 7;
const CCC_RESPONSE_BLOCKS: u8 = 0x21;
const CCC_REQUEST_BLOCKS: u8 = 0x20;

bitflags! {
    pub struct SigMap: u8 {
        const SIG1 = 1;
        const SIG2 = 2;
        const SIG3 = 4;
        const SIG4 = 8;
        const SIG5 = 16;
        const SIG6 = 32;
        const SIG7 = 64;
        const COLOR_CODES = 128;
        const ALL = 0xff;
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Block {
   pub signature: u16,
   pub x: u16,
   pub y: u16,
   pub width: u16,
   pub height: u16,
   pub angle: u16,
   pub index: u8,
   pub age: u8, 
}

const BLOCK_SIZE: usize = 14;

impl Block {
    fn from(bytes: &[u8; 14]) -> Block {
        Block {
            signature: u16::from_le_bytes(bytes[0..2].try_into().unwrap()),
            x: u16::from_le_bytes(bytes[2..4].try_into().unwrap()),
            y: u16::from_le_bytes(bytes[4..6].try_into().unwrap()),
            width: u16::from_le_bytes(bytes[6..8].try_into().unwrap()),
            height: u16::from_le_bytes(bytes[8..10].try_into().unwrap()),
            angle: u16::from_le_bytes(bytes[10..12].try_into().unwrap()),
            index: u8::from_le(bytes[12]),
            age: u8::from_le(bytes[13]),
        }
    }
}

#[derive(Debug)]
pub enum Pixy2Error {
    Error,
    Busy,
    ChecksumError,
    Timeout,
    ButtonOverride,
    ProgChanging,
}

impl Pixy2Error {
    fn to_code(&self) -> i8 {
        use Pixy2Error::*;
        match self {
            Error => -1,
            Busy => -2,
            ChecksumError => -3,
            Timeout => -4,
            ButtonOverride => -5,
            ProgChanging => -6
        }
    }

    fn from_code(code: i8) -> Pixy2Error {
        use Pixy2Error::*;
        match code {
            -1 => Error,
            -2 => Busy,
            -3 => ChecksumError,
            -4 => Timeout,
            -5 => ButtonOverride,
            -6 => ProgChanging,
            _ => panic!("code out of range")
        }
    }

    fn ok_code() -> i8 {
        0
    }
}

#[repr(C)]
pub struct Version {
    pub hardware: u16,
    pub firmware_major: u8,
    pub firmware_minor: u8,
    pub firmware_build: u16,
    pub firmware_type: [u8; 10],
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
    pub version: Version,
    pub frame_width: u16,
    pub frame_height: u16,
    pub blocks: [Block; 18], // 18 = floor(BUFFERSIZE / sizeof(Block))
    pub num_blocks: u8,
}

impl<S: spim::Instance, T: timer::Instance> Pixy2<S, T> {
    // spi must be 2MBs, MSBFIRST, MODE 3
    pub fn new(
        spi: Spim<S>,
        chip_select: Pin<Output<PushPull>>,
        timer_instance: T,
    ) -> Result<Self, Pixy2Error> {
        let mut pixy2 = Pixy2 {
            spi,
            chip_select,
            timer: timer_instance,
            m_cs: false,
            m_buf: [0; BUFFERSIZE],
            m_length: 0,
            m_type: 0,
            version: Version::from(&[0; 16]),
            frame_width: 0,
            frame_height: 0,
            blocks: [Block::from(&[0; 14]); 18],
            num_blocks: 0,
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

    pub fn get_resolution(&mut self) -> Result<(), Pixy2Error> {
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

    pub fn get_blocks(&mut self, wait: bool, sigmap: SigMap, max_blocks: u8) -> Result<u8, Pixy2Error> {
        self.num_blocks = 0;

        loop {
            self.m_buf[SEND_HEADER_SIZE + 0] = sigmap.bits;
            self.m_buf[SEND_HEADER_SIZE + 1] = max_blocks;
            self.m_length = 2;
            self.m_type = CCC_REQUEST_BLOCKS;

            self.send_packet()?;
            self.recv_packet()?;

            if self.m_type == CCC_RESPONSE_BLOCKS {
                self.num_blocks = self.m_length / u8::try_from(BUFFERSIZE).unwrap();
                for i in 0..self.num_blocks as usize {
                    self.blocks[i] = Block::from(&self.m_buf[i * BLOCK_SIZE.. (i+1) * BLOCK_SIZE]
                        .try_into().unwrap());
                }
                return Ok(self.num_blocks);
            } else if self.m_type == TYPE_RESPONSE_ERROR {
                if self.m_buf[0] as i8 == Pixy2Error::Busy.to_code() {
                    if !wait {
                        return Err(Pixy2Error::Busy);
                    }
                } else if self.m_buf[0] as i8 != Pixy2Error::ProgChanging.to_code() {
                    if self.m_buf[0] as i8 == Pixy2Error::ok_code() {
                        return Ok(0);
                    } else {
                        return Err(Pixy2Error::from_code(self.m_buf[0] as i8));
                    }
                }
            }

            self.delay_us(500);
        }
        Err(Pixy2Error::Error)
    }
}

fn cumulative_sum(buf: &[u8]) -> u16 {
    let mut sum: u16 = 0;
    for byte in buf {
        sum += u16::from(*byte);
    }
    sum
}
