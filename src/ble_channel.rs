//! Some extra wrapper stuff to make writeable attributes possible within Rubble.

use crate::ble_service::RomiServiceAttrs;
use rubble::att::{
    pdus::{AttError, AttPdu, Opcode},
    AttributeServer,
};
use rubble::bytes::{ByteReader, FromBytes};
use rubble::l2cap::{
    BleChannelMap, Channel, ChannelData, ChannelMapper, Protocol, ProtocolObj, Sender,
};
use rubble::security::NoSecurity;
use rubble::Error;

struct RomiAttrServer {
    attrs: RomiServiceAttrs,
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
        match msg {
            AttPdu::WriteReq {
                handle: _,
                value: _,
            } => {
                // TODO do state change and stuff
                responder
                    .send_with(|writer| -> Result<(), Error> {
                        writer.write_u8(Opcode::WriteRsp.into())?;
                        Ok(())
                    })
                    .unwrap();
                Ok(())
            }
            AttPdu::WriteCommand {
                handle: _,
                value: _,
            } => {
                // TODO change state and stuff
                // No response is needed for WriteCommand
                Ok(())
            }
            _ => unimplemented!("RomiAttrServer can only handle WriteReq and WriteCommand"),
        }
    }
}

struct RomiChannelMap {
    actual_server: RomiAttrServer,
}

impl RomiChannelMap {
    pub fn new() -> Self {
        Self {
            actual_server: RomiAttrServer {
                attrs: RomiServiceAttrs::new(),
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
        // match channel {
        //     Channel::ATT => Some(ChannelData::new_dyn(channel, &mut self.att)),
        //     Channel::LE_SIGNALING => Some(ChannelData::new_dyn(channel, &mut self.signaling)),
        //     Channel::LE_SECURITY_MANAGER => Some(ChannelData::new_dyn(channel, &mut self.sm)),
        //     _ => None,
        // }
    }

    fn att(&mut self) -> ChannelData<'_, AttributeServer<Self::AttributeProvider>> {
        // ChannelData::new(Channel::ATT, &mut self.actual_server)
        self.actual_server.wrapped.att()
    }
}
