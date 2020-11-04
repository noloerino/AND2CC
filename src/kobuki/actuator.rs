//!
//!   Original Author:        Jeff C. Jensen
//!   Rewritten By:           Joshua Adkins, Neal Jackson
//!   Rewritten in Rust By:   Jonathan Shi
//!   Revised:    2020-11-03
//!
use crate::kobuki::utilities;
use nrf52832_hal::uarte;
use nrf52832_hal::Uarte;

/// Provides access to functions that the robot sends over UART, passed in via the ::new function.
/// To allow usage of the UART instance by other structs, simply allow the Actuator to go out of
/// scope, which releases the mutable borrow.
pub struct Actuator<'a, T: uarte::Instance> {
    serial: &'a mut Uarte<T>,
}

#[derive(Debug)]
pub enum Sound {
    On,
    Off,
    Recharge,
    Button,
    Error,
    CleaningStart,
    CleaningEnd,
}

impl<'a, T: uarte::Instance> Actuator<'a, T> {
    pub fn new(serial: &'a mut Uarte<T>) -> Self {
        Actuator { serial }
    }

    fn send_payload(&mut self, payload: &[u8]) -> Result<(), uarte::Error> {
        let mut write_data: [u8; 256] = [0; 256];
        let len = payload.len() as u8;
        // Write move payload
        write_data[0] = 0xAA;
        write_data[1] = 0x55;
        write_data[2] = len;
        write_data[3..(3 + len as usize)].copy_from_slice(payload);
        write_data[3 + len as usize] = utilities::checksum(&write_data[..(3 + len) as usize]);
        self.serial.write(&write_data[..(4 + len as usize)])
    }

    /// Wheel speed is defined in mm/s
    pub fn drive_direct(
        &mut self,
        left_wheel_speed: i16,
        right_wheel_speed: i16,
    ) -> Result<(), uarte::Error> {
        let cmd_speed: i32 = if right_wheel_speed.wrapping_abs() > left_wheel_speed.wrapping_abs() {
            right_wheel_speed.into()
        } else {
            left_wheel_speed.into()
        };
        let mut cmd_radius: i32;
        if right_wheel_speed == left_wheel_speed {
            cmd_radius = 0;
        } else {
            // don't really know what's happening here, but I copy/pasted so whatever
            // f32::round() appears to be in std not core, so we ignore it /shrug
            cmd_radius = (((right_wheel_speed + left_wheel_speed) as f32)
                / (2.0 * ((right_wheel_speed - left_wheel_speed) as f32) / 123.0))
                as i32;
            if cmd_radius > 327667 || cmd_radius < -32768 {
                cmd_radius = 0;
            }
            if cmd_radius == 0 {
                cmd_radius = 1;
            }
        }
        self.drive_radius(cmd_radius as i16, cmd_speed as i16)
    }

    /// Speed is defined in mm/s, radius is defined in mm
    pub fn drive_radius(&mut self, radius: i16, speed: i16) -> Result<(), uarte::Error> {
        let mut payload: [u8; 6] = [0; 6];
        payload[0] = 0x01;
        payload[1] = 0x04;
        payload[2..4].copy_from_slice(&speed.to_le_bytes());
        payload[4..6].copy_from_slice(&radius.to_le_bytes());
        self.send_payload(&payload)
    }

    /// Sets the PID gains on the robot wheel control to the defaults
    pub fn set_controller_default(&mut self) -> Result<(), uarte::Error> {
        let mut payload: [u8; 15] = [0; 15];
        payload[0] = 0x0D; // PID type 13
        payload[1] = 0x0D; // 13 byte PID length
        payload[2] = 0x00; // Default gain
        self.send_payload(&payload)
    }

    /// Sets the PID gains on the robot wheel control to user specified values
    /// P = Kp/1000
    /// I = Ki/1000
    /// D = Kd/1000
    ///
    /// Defaults:
    /// P = 100
    /// I = 0.1
    /// D = 2
    pub fn kobuki_set_controller_user(
        &mut self,
        kp: u32,
        ki: u32,
        kd: u32,
    ) -> Result<(), uarte::Error> {
        let mut payload: [u8; 15] = [0; 15];
        payload[0] = 0x0D; // PID type 13
        payload[1] = 0x0D; // 13 byte PID length
        payload[2] = 0x01; // User gain

        //The values passed in are multiplied by 1000 to be represented by integers
        // P = Kp/1000
        // I = Ki/1000;
        // D = Kd/1000;
        payload[3..7].copy_from_slice(&kp.to_le_bytes());
        payload[7..11].copy_from_slice(&ki.to_le_bytes());
        payload[11..15].copy_from_slice(&kd.to_le_bytes());
        self.send_payload(&payload)
    }

    /// Play a predefined sound from the above sound types
    pub fn play_sound_sequence(&mut self, sound: Sound) -> Result<(), uarte::Error> {
        let mut payload: [u8; 3] = [0; 3];
        payload[0] = 0x04;
        payload[1] = 0x01;
        payload[2] = sound as u8;
        self.send_payload(&payload)
    }

    /// Request hardware version, firmware version and unique ID on the next data packet
    pub fn request_information(&mut self) -> Result<(), uarte::Error> {
        let mut payload: [u8; 4] = [0; 4];
        payload[0] = 0x09;
        payload[1] = 0x02;
        payload[2] = 0x08 | 0x02 | 0x01;
        payload[3] = 0x00;
        self.send_payload(&payload)
    }
}
