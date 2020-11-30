//! Some extra wrapper stuff to make writeable attributes possible within Rubble.

use crate::ble_service::{RomiServiceAttrs, LED_CHAR_VALUE_HANDLE, TEST_STATE};
use core::cmp;
use rtt_target::rprintln;
use rubble::att::{
    pdus::{AttError, AttPdu, Opcode},
    AttributeServer, Handle,
};
use rubble::bytes::{ByteReader, FromBytes};
use rubble::l2cap::{
    BleChannelMap, Channel, ChannelData, ChannelMapper, Protocol, ProtocolObj, Sender,
};
use rubble::security::NoSecurity;
use rubble::Error;

struct RomiAttrServer {
    wrapped: BleChannelMap<RomiServiceAttrs, NoSecurity>,
}

impl Protocol for RomiAttrServer {
    // Copied from AttributeServer's definition
    const RSP_PDU_SIZE: u8 = 23;
}

impl ProtocolObj for RomiAttrServer {
    fn process_message(&mut self, message: &[u8], mut responder: Sender<'_>) -> Result<(), Error> {
        let pdu = &AttPdu::from_bytes(&mut ByteReader::new(message))?;
        let opcode = pdu.opcode();
        match opcode {
            Opcode::WriteReq | Opcode::WriteCommand => {
                match self.process_request(pdu, &mut responder) {
                    Ok(()) => Ok(()),
                    Err(att_error) => responder.send(AttPdu::ErrorRsp {
                        opcode,
                        handle: att_error.handle(),
                        error_code: att_error.error_code(),
                    }),
                }
            }
            _ => self
                .wrapped
                .att()
                .protocol()
                .process_message(message, responder),
        }
    }
}

impl RomiAttrServer {
    fn process_request(
        &mut self,
        msg: &AttPdu<'_>,
        responder: &mut Sender<'_>,
    ) -> Result<(), AttError> {
        rprintln!("[ble] Processing request: {:#?}", msg);
        match msg {
            AttPdu::WriteReq { handle, value } => {
                // TODO do state change and stuff
                rprintln!(
                    "[ble] Received WriteReq w/ handle {:#?} value {:#?}",
                    handle,
                    value
                );
                if *handle == Handle::from_raw(LED_CHAR_VALUE_HANDLE) {
                    rprintln!("updating state");
                    let value_slice = value.as_ref();
                    unsafe {
                        TEST_STATE[..cmp::min(TEST_STATE.len(), value_slice.len())]
                            .clone_from_slice(
                                &value_slice[..cmp::min(TEST_STATE.len(), value_slice.len())],
                            )
                    }
                }
                responder
                    .send_with(|writer| -> Result<(), Error> {
                        writer.write_u8(Opcode::WriteRsp.into())?;
                        Ok(())
                    })
                    .unwrap();
                Ok(())
            }
            AttPdu::WriteCommand { handle, value } => {
                rprintln!(
                    "[ble] Received WriteCommand w/ handle {:#?} value {:#?}",
                    handle,
                    value
                );
                // TODO change state and stuff
                // No response is needed for WriteCommand
                Ok(())
            }
            _ => unimplemented!("RomiAttrServer can only handle WriteReq and WriteCommand"),
        }
    }
}

pub struct RomiChannelMap {
    actual_server: RomiAttrServer,
}

impl Default for RomiChannelMap {
    fn default() -> Self {
        Self::new()
    }
}

impl RomiChannelMap {
    pub fn new() -> Self {
        Self {
            actual_server: RomiAttrServer {
                wrapped: BleChannelMap::with_attributes(RomiServiceAttrs::new()),
            },
        }
    }
}

impl ChannelMapper for RomiChannelMap {
    type AttributeProvider = RomiServiceAttrs;

    fn lookup(&mut self, channel: Channel) -> Option<ChannelData<'_, dyn ProtocolObj + '_>> {
        if let Channel::ATT = channel {
            Some(ChannelData::new_dyn(channel, &mut self.actual_server))
        } else {
            self.actual_server.wrapped.lookup(channel)
        }
    }

    fn att(&mut self) -> ChannelData<'_, AttributeServer<Self::AttributeProvider>> {
        self.actual_server.wrapped.att()
    }
}
