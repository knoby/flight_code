use crate::fc;
use defmt::debug;
use hal::prelude::*;
use heapless::consts::U32;

/// Function called from the main app macro to clean the main.rs
pub fn idle(cx: crate::idle::Context) -> ! {
    defmt::debug!("Entering Idle Task");
    let mut buffer = heapless::Vec::<u8, U32>::new();
    let mut length = None;
    let mut reciving_msg = false;
    loop {
        while let Some(val) = cx.resources.CONS_SERIAL_READ.dequeue() {
            cx.resources.LED_NE.toggle().unwrap();

            // wait for start byte
            if (val == copter_com::START_BYTE) && !reciving_msg {
                reciving_msg = true;
                length = None;
                buffer.clear();
            }

            // Add byte to buffer
            if reciving_msg {
                buffer.push(val).ok();
                let mut raw = [0; 32];
                for (i, byte) in buffer.iter().enumerate() {
                    raw[i] = *byte;
                }
            }

            // Is the length byte?
            if buffer.len() == 2 {
                if val <= 30 {
                    length = Some(val);
                } else {
                    reciving_msg = false;
                }
            }

            // Check end of message
            if let Some(len) = length {
                if (len as u16 + 2) == (buffer.len() as u16) {
                    // Parse the message from buffer
                    let msg = copter_com::Message::parse(&buffer);
                    debug!("{:?}", msg);
                    // eval the message and return an answer for the message
                    if let Ok(msg) = msg {
                        let answer = match msg {
                            copter_com::Message::Ping(_) => msg,
                            copter_com::Message::EnableMotor => {
                                if cx
                                    .resources
                                    .PROD_APP_CMD
                                    .enqueue(fc::AppCommand::EnableMotors)
                                    .is_ok()
                                {
                                    copter_com::Message::Ack
                                } else {
                                    copter_com::Message::NoAck
                                }
                            }
                            copter_com::Message::DisableMotor => {
                                if cx
                                    .resources
                                    .PROD_APP_CMD
                                    .enqueue(fc::AppCommand::DisableMotors)
                                    .is_ok()
                                {
                                    copter_com::Message::Ack
                                } else {
                                    copter_com::Message::NoAck
                                }
                            }
                            copter_com::Message::ChangeSetvalue(setvalue) => {
                                if cx
                                    .resources
                                    .PROD_APP_CMD
                                    .enqueue(fc::AppCommand::ChangeSetValue(setvalue))
                                    .is_ok()
                                {
                                    copter_com::Message::Ack
                                } else {
                                    copter_com::Message::NoAck
                                }
                            }
                            _ => copter_com::Message::NoAck,
                        };
                        // Send answer to message
                        cx.spawn.serial_send(answer).unwrap();
                        // Toggle led to indicate a message was processed
                        cx.resources.LED_E.toggle().unwrap()
                    };
                    reciving_msg = false;
                    length = None;
                }
            }
        }
    }
}
