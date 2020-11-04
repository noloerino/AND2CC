//! Various sensor definitions.
//! See https://github.com/lab11/buckler/tree/master/software/libraries/kobuki
//! Some additional convenience functions are also provided.

use crate::kobuki::utilities;
use core::convert::TryInto;
use core::default;
use nrf52832_hal::uarte;
use nrf52832_hal::Uarte;
use rtt_target::rprintln;

// === TYPE DEFINITIONS ===

/// Detects whether the wheel is in contact with the ground and if bumpers are currently pressed.
#[derive(Default)]
#[repr(C)]
pub struct BumpsAndWheelDrops {
    wheel_drop_left: bool,
    wheel_drop_right: bool,
    bump_left: bool,
    bump_center: bool,
    bump_right: bool,
}

#[derive(Default, Copy, Clone)]
#[repr(C)]
pub struct Buttons {
    b0: bool,
    b1: bool,
    b2: bool,
}

#[repr(C)]
pub enum ChargerState {
    Discharging,
    DockingCharged,
    DockingCharging,
    AdapterCharged,
    AdapterCharging,
}

impl default::Default for ChargerState {
    fn default() -> Self {
        ChargerState::Discharging
    }
}

#[repr(C)]
pub enum DockingState {
    NearLeft,
    NearCenter,
    NearRight,
    FarCenter,
    FarLeft,
    FarRight,
}

impl DockingState {
    fn from_ordinal(n: u8) -> Self {
        use DockingState::*;
        match n {
            0 => NearLeft,
            1 => NearCenter,
            2 => NearRight,
            3 => FarCenter,
            4 => FarLeft,
            5 => FarRight,
            _ => panic!("Invalid ordinal value {} for DockingState", n),
        }
    }
}

impl default::Default for DockingState {
    fn default() -> Self {
        DockingState::NearLeft
    }
}

#[derive(Default)]
#[repr(C)]
pub struct Docking {
    right: DockingState,
    center: DockingState,
    left: DockingState,
}

#[derive(Default)]
#[repr(C)]
pub struct Version {
    patch: u8,
    minor: u8,
    major: u8,
}

#[derive(Default)]
#[repr(C)]
pub struct Input {
    d0: bool,
    d1: bool,
    d2: bool,
    d3: bool,
    a0: u16,
    a1: u16,
    a2: u16,
    a3: u16,
}

#[derive(Default)]
#[repr(C)]
pub struct Gain {
    user_configured: bool,
    kp: u32,
    ki: u32,
    kd: u32,
}

#[derive(Default)]
#[repr(C)]
pub struct Sensors {
    pub bumps_wheel_drops: BumpsAndWheelDrops,
    // Binary interpretations of cliff sensors (true if over cliff)
    pub cliff_left: bool,
    pub cliff_center: bool,
    pub cliff_right: bool,
    // Raw values of cliff sensors
    pub cliff_left_signal: u16,
    pub cliff_center_signal: u16,
    pub cliff_right_signal: u16,
    pub buttons: Buttons,
    // Motor Feedback
    // 16-bit unsigned roll over, forward is positive
    pub left_wheel_encoder: u16,
    pub right_wheel_encoder: u16,
    // Motor current in 10ma increments
    pub left_wheel_current: i16,
    pub right_wheel_current: i16,
    // Raw PWM value applied, 8-bit signed, forward is positive
    pub left_wheel_pwm: i8,
    pub right_wheel_pwm: i8,
    // Indicates motor over current has triggered
    pub left_wheel_over_current: bool,
    pub right_wheel_over_current: bool,
    // Timestamp, 16 bit unsigned in ms, rolls on overflow
    pub timestamp: u16,
    // Battery voltage and charging state
    pub battery_voltage: u8,
    pub charging_state: ChargerState,
    // Inertia measurement: calibrated angle of rotation around Z-axis
    pub angle: i16,
    pub angle_rate: i16,
    // Raw values from the gyro in 0.00875 deg/s increments
    pub x_axis_rate: u16,
    pub y_axis_rate: u16,
    pub z_axis_rate: u16,
    // Docking Position Feedback for the three IR docking sensors
    pub docking: Docking,
    // Hardware and software versions
    pub hardware_version: Version,
    pub software_version: Version,
    pub uid: [u32; 3],
    pub general_input: Input,
    pub controller_gain: Gain,
    // Private state used to debounce buttons
    prev_buttons: Buttons,
}

impl Sensors {
    pub fn is_bump(&self) -> bool {
        self.bumps_wheel_drops.bump_left
            || self.bumps_wheel_drops.bump_center
            || self.bumps_wheel_drops.bump_right
    }

    pub fn is_button_pressed(&mut self) -> bool {
        let mut result = false;
        let curr_b0 = self.buttons.b0;
        if curr_b0 && self.prev_buttons.b0 != curr_b0 {
            result = true;
        }
        let curr_b1 = self.buttons.b1;
        if curr_b1 && self.prev_buttons.b1 != curr_b1 {
            result = true;
        }
        let curr_b2 = self.buttons.b2;
        if curr_b2 && self.prev_buttons.b2 != curr_b2 {
            result = true;
        }
        self.prev_buttons = self.buttons;
        result
    }
}

// === POLLING ===
enum ReceiveStateType {
    WaitUntilAa,
    ReadLength,
    ReadPayload,
    ReadChecksum,
}

pub struct SensorPoller<'a, T: uarte::Instance> {
    serial: &'a mut Uarte<T>,
}

impl<'a, T: uarte::Instance> SensorPoller<'a, T> {
    fn read_feedback_packet(&mut self, packet_buffer: &mut [u8; 140]) -> Result<(), uarte::Error> {
        let mut state = ReceiveStateType::WaitUntilAa;
        let mut header_buf: [u8; 2] = [0; 2];
        let mut payload_size_buf: [u8; 1] = [0; 1];
        let mut aa_count: isize = 0;
        let mut num_checksum_failures: i32 = 0;
        loop {
            use ReceiveStateType::*;
            match state {
                WaitUntilAa => {
                    // Unlike original C code, we read both bytes at once
                    let status = self.serial.read(&mut header_buf);
                    if let Err(e) = status {
                        rprintln!("UART error reading kobuki header: {:#?}", e);
                        aa_count += 1;
                        if aa_count < 20 {
                            rprintln!("\ttrying again...");
                        } else {
                            rprintln!("Failed to recieve from robot.\n\tIs robot powered on?\n\tTry unplugging buckler from USB and power cycle robot");
                            return Err(e);
                        }
                    }
                    if header_buf[0] == 0xAA && header_buf[1] == 0x55 {
                        state = ReadLength;
                    } else {
                        state = WaitUntilAa;
                    }
                    aa_count = 0;
                }
                ReadLength => {
                    self.serial.read(&mut payload_size_buf)?;
                    if packet_buffer.len() < (payload_size_buf[0] + 3) as usize {
                        rprintln!(
                            "While reading payload size, payload size was {} but len was {}",
                            payload_size_buf[0],
                            packet_buffer.len(),
                        );
                        break;
                    }
                    state = ReadPayload;
                }
                ReadPayload => {
                    self.serial
                        .read(&mut packet_buffer[3..(3 + payload_size_buf[0] as usize + 1)])?;
                    state = ReadChecksum;
                }
                ReadChecksum => {
                    packet_buffer[0..2].copy_from_slice(&header_buf);
                    packet_buffer[2] = payload_size_buf[0];
                    let calculated_checksum = utilities::checksum(packet_buffer);
                    let byte_buffer = packet_buffer[payload_size_buf[0] as usize + 3];
                    if calculated_checksum == byte_buffer {
                        return Ok(());
                    } else {
                        state = WaitUntilAa;
                        if num_checksum_failures == 3 {
                            panic!("Too many checksum failures while reading UART feedback packet");
                        }
                        num_checksum_failures += 1;
                        rprintln!("Checksum fails: {}", num_checksum_failures);
                    }
                }
            }
        }
        panic!("Fatal error reading UART feedback packet");
    }

    fn parse_sensor_packet(&self, packet: &[u8], sensors: &mut Sensors) {
        let payload_len: usize = packet[2] as usize;
        let mut i: usize = 3;
        while i < payload_len + 3 {
            let id_field: u8 = packet[i];
            let sub_payload_length = packet[i + 1] as usize;
            match id_field {
                0x01 => {
                    // There's an ambiguity in the documentation where
                    // it says there are two headers with value 0x01:
                    // basic sensor data and controller info - although it
                    // says elsewhere that controller info has ID 0x15
                    // so we'll just check here to make sure it's the right length
                    if sub_payload_length == 0x0F {
                        sensors.timestamp =
                            u16::from_le_bytes(packet[i + 2..i + 3].try_into().unwrap());
                        sensors.bumps_wheel_drops.bump_right = packet[i + 4] & 0x01 != 0;
                        sensors.bumps_wheel_drops.bump_center = packet[i + 4] & 0x02 != 0;
                        sensors.bumps_wheel_drops.bump_left = packet[i + 4] & 0x04 != 0;
                        sensors.bumps_wheel_drops.wheel_drop_right = packet[i + 5] & 0x01 != 0;
                        sensors.bumps_wheel_drops.wheel_drop_left = packet[i + 5] & 0x02 != 0;

                        sensors.cliff_right = (packet[i + 6] & 0x01) != 0;
                        sensors.cliff_center = (packet[i + 6] & 0x02) != 0;
                        sensors.cliff_left = (packet[i + 6] & 0x04) != 0;

                        sensors.left_wheel_encoder =
                            u16::from_le_bytes(packet[i + 7..i + 8].try_into().unwrap());
                        sensors.right_wheel_encoder =
                            u16::from_le_bytes(packet[i + 9..i + 10].try_into().unwrap());

                        sensors.left_wheel_pwm = packet[i + 11] as i8;
                        sensors.right_wheel_pwm = packet[i + 12] as i8;
                        sensors.buttons.b0 = (packet[i + 13] & 0x01) != 0;
                        sensors.buttons.b1 = (packet[i + 13] & 0x02) != 0;
                        sensors.buttons.b2 = (packet[i + 13] & 0x04) != 0;

                        // Charger state
                        use ChargerState::*;
                        match packet[i + 14] {
                            0 => sensors.charging_state = Discharging,
                            2 => sensors.charging_state = DockingCharged,
                            6 => sensors.charging_state = DockingCharging,
                            18 => sensors.charging_state = AdapterCharged,
                            22 => sensors.charging_state = AdapterCharging,
                            _ => {}
                        }

                        sensors.battery_voltage = packet[i + 15];

                        sensors.left_wheel_over_current = packet[i + 16] & 0x01 != 0;
                        sensors.right_wheel_over_current = packet[i + 16] & 0x02 != 0;

                        i += sub_payload_length + 2; // + 2 for header and length
                    } else {
                        i += payload_len + 3; // add enough to terminate the outer while loop
                    }
                }
                0x03 => {
                    if sub_payload_length == 0x03 {
                        sensors.docking.right = DockingState::from_ordinal(packet[i + 2]);
                        sensors.docking.center = DockingState::from_ordinal(packet[i + 3]);
                        sensors.docking.left = DockingState::from_ordinal(packet[i + 4]);
                        i += sub_payload_length + 2;
                    } else {
                        i += payload_len + 3;
                    }
                }
                0x04 => {
                    // inertial sensor data
                    if sub_payload_length == 0x07 {
                        sensors.angle =
                            i16::from_le_bytes(packet[i + 2..i + 3].try_into().unwrap());
                        sensors.angle_rate =
                            i16::from_le_bytes(packet[i + 4..i + 5].try_into().unwrap());
                        i += sub_payload_length + 2;
                    } else {
                        i += payload_len + 3;
                    }
                }
                0x05 => {
                    // cliff sensor data
                    if sub_payload_length == 0x06 {
                        sensors.cliff_right_signal =
                            u16::from_le_bytes(packet[i + 2..i + 3].try_into().unwrap());
                        sensors.cliff_center_signal =
                            u16::from_le_bytes(packet[i + 4..i + 5].try_into().unwrap());
                        sensors.cliff_left_signal =
                            u16::from_le_bytes(packet[i + 6..i + 7].try_into().unwrap());
                        i += sub_payload_length + 2;
                    } else {
                        i += payload_len + 3;
                    }
                }
                // TODO implement other cases as needed
                // https://github.com/lab11/buckler/blob/master/software/libraries/kobuki/kobukiSensor.c
                _ => {}
            }
        }
    }

    /// Sends a request for sensor data over UART and updates the Sensors object in-place.
    pub fn poll(serial: &'a mut Uarte<T>, sensors: &mut Sensors) -> Result<(), uarte::Error> {
        let mut poller = SensorPoller { serial };
        let mut packet: [u8; 140] = [0; 140];
        poller.read_feedback_packet(&mut packet)?;
        Ok(poller.parse_sensor_packet(&packet, sensors))
    }
}
