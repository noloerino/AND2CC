//! Handles BLE service stuff.
//! Roughly based off the implementation of BatteryServiceAttrs/MidiServiceAttrs
//! https://github.com/jonas-schievink/rubble/blob/master/rubble/src/gatt/mod.rs

use core::cmp;
use rubble::att::{AttUuid, Attribute, AttributeProvider, Handle, HandleRange};
use rubble::uuid::{Uuid128, Uuid16};

pub struct RomiServiceAttrs {
    // Change this field manually when adding new attrs
    attributes: [Attribute<'static>; 4],
}

impl Default for RomiServiceAttrs {
    fn default() -> Self {
        Self::new()
    }
}

// 32e61089-2b22-4db5-a914-43ce41986c70 (from lab)
const LED_UUID128: [u8; 16] = [
    0x70, 0x6C, 0x98, 0x41, 0xCE, 0x43, 0x14, 0xA9, 0xB5, 0x4D, 0x22, 0x2B, 0x89, 0x10, 0xE6, 0x32,
];
// Replace bytes 12/13 (0x1089) of the 128-bit UUID with 0x108A
const LED_STATE_CHAR_UUID128: [u8; 16] = [
    0x70, 0x6C, 0x98, 0x41, 0xCE, 0x43, 0x14, 0xA9, 0xB5, 0x4D, 0x22, 0x2B, 0x8A, 0x10, 0xE6, 0x32,
];

const PRIMARY_SERVICE_UUID16: Uuid16 = Uuid16(0x2800);
const CHARACTERISTIC_UUID16: Uuid16 = Uuid16(0x2803);

const LED_CHAR_DECL_VALUE: [u8; 19] = [
    0x02 | 0x08, // 1 byte properties: read = 0x02, write request/response = 0x08
    // 2 byte handle pointing to characteristic value
    0x03,
    0x00,
    // 128-bit UUID of characteristic value (copied from above constant)
    0x70,
    0x6C,
    0x98,
    0x41,
    0xCE,
    0x43,
    0x14,
    0xA9,
    0xB5,
    0x4D,
    0x22,
    0x2B,
    0x8A,
    0x10,
    0xE6,
    0x32,
];

type RomiBleState = [u8; 2];

// This array represents underlying data shared by all instances of RomiServiceAttrs
// TODO turn this into RTIC state and pass a reference to the service thing so we can lock
pub static mut TEST_STATE: RomiBleState = [0x12, 0x34];
pub const LED_CHAR_VALUE_HANDLE: u16 = 0x3;

// https://www.oreilly.com/library/view/getting-started-with/9781491900550/ch04.html
// The above link is extremely helpful in determining the structure of BLE stuff
// Permission flags are in the order they appear in table 4-6
// https://btprodspecificationrefs.blob.core.windows.net/assigned-values/16-bit%20UUID%20Numbers%20Document.pdf
// This lists reserved 16b UUIDs in case something weird appears
impl RomiServiceAttrs {
    pub fn new() -> Self {
        Self {
            attributes: [
                Attribute::new(
                    PRIMARY_SERVICE_UUID16.into(),
                    Handle::from_raw(0x1),
                    &LED_UUID128,
                ),
                Attribute::new(
                    CHARACTERISTIC_UUID16.into(),
                    Handle::from_raw(0x2),
                    &LED_CHAR_DECL_VALUE,
                ),
                // Characteristic value
                Attribute::new(
                    Uuid128::from_bytes(LED_STATE_CHAR_UUID128).into(),
                    Handle::from_raw(LED_CHAR_VALUE_HANDLE),
                    unsafe { &TEST_STATE },
                ),
                // Client Characteristic Configuration Descriptor (CCCD)
                Attribute::new(Uuid16(0x2902).into(), Handle::from_raw(0x4), &[0x0, 0x0]),
            ],
        }
    }

    // Updates the static data; doesn't take in self because it's static
    pub fn update_data(new_data: &RomiBleState) {
        unsafe {
            TEST_STATE = *new_data;
        }
    }
}

impl AttributeProvider for RomiServiceAttrs {
    // Copied from https://github.com/jonas-schievink/rubble/blob/master/rubble/src/gatt/mod.rs#L47
    fn for_attrs_in_range(
        &mut self,
        range: HandleRange,
        mut f: impl FnMut(&Self, Attribute<'_>) -> Result<(), rubble::Error>,
    ) -> Result<(), rubble::Error> {
        let count = self.attributes.len();
        let start = usize::from(range.start().as_u16() - 1); // handles start at 1, not 0
        let end = usize::from(range.end().as_u16() - 1);

        let attrs = if start >= count {
            &[]
        } else {
            let end = cmp::min(count - 1, end);
            &self.attributes[start..=end]
        };

        for attr in attrs {
            f(
                self,
                Attribute {
                    att_type: attr.att_type,
                    handle: attr.handle,
                    value: attr.value,
                },
            )?;
        }
        Ok(())
    }

    fn is_grouping_attr(&self, uuid: AttUuid) -> bool {
        uuid == PRIMARY_SERVICE_UUID16 || uuid == CHARACTERISTIC_UUID16
    }

    fn group_end(&self, handle: Handle) -> Option<&Attribute<'_>> {
        // don't subtract 1 from len since handles start at 1
        if handle.as_u16() < self.attributes.len() as u16 {
            self.attributes.last()
        } else {
            None
        }
    }
}
