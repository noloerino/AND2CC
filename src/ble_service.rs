//! Handles BLE service stuff.
//! Roughly based off the implementation of BatteryServiceAttrs
//! https://github.com/jonas-schievink/rubble/blob/master/rubble/src/gatt/mod.rs

use rubble::att::{Attribute, Handle};
use rubble::utils::HexSlice;
use rubble::uuid::Uuid128;

pub struct RomiServiceAttrs {
    // Change this field manually when adding new attrs
    attributes: [Attribute<'static>; 1],
}

impl RomiServiceAttrs {
    pub fn new() -> Self {
        // 32e61089-2b22-4db5-a914-43ce41986c70 (from lab)
        let led_uuid128 = [
            0x70, 0x6C, 0x98, 0x41, 0xCE, 0x43, 0x14, 0xA9, 0xB5, 0x4D, 0x22, 0x2B, 0x89, 0x10,
            0xE6, 0x32,
        ];

        // https://github.com/lab11/nrf52x-base/blob/master/lib/simple_ble/simple_ble.c#L905
        let led_uuid16 = [led_uuid128[13], led_uuid128[12]];
        Self {
            attributes: [Attribute {
                att_type: Uuid128::from_bytes(led_uuid128).into(),
                handle: Handle::from_raw(led_uuid16),
                value: HexSlice(&[
                    0x02 | 0x08 | 0x04 | 0x10, // 1 byte properties: READ = 0x02, WRITE_REQ = 0x08, WRITE_CMD = 0x04, NOTIFICATION = 0x10
                    0x8A,
                    0x10, // 2 byte handle of characteristic
                ]),
            }],
        }
    }
}
