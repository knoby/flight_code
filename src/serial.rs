type SerialTxPin = hal::gpio::PC4<
    hal::gpio::PullNone,
    hal::gpio::AltFn<hal::gpio::AF7, hal::gpio::PushPull, hal::gpio::HighSpeed>,
>;

type SerialRxPin = hal::gpio::PC5<
    hal::gpio::PullNone,
    hal::gpio::AltFn<hal::gpio::AF7, hal::gpio::PushPull, hal::gpio::HighSpeed>,
>;

type SerialUart = hal::device::USART1;

pub type SerialTx = hal::serial::Tx<SerialUart>;
pub type SerialRx = hal::serial::Rx<SerialUart>;

pub type Serial = hal::serial::Serial<SerialUart, (SerialTxPin, SerialRxPin)>;

pub fn create_tx_rx(mut serial: Serial) -> (SerialTx, SerialRx) {
    // Enable Interrupts
    serial.listen(hal::serial::Event::Rxne);

    // Get Objects
    let (tx, rx) = serial.split();

    (tx, rx)
}

// List of possible Commands for serial communication
pub enum Command {
    StartMotor(Option<crate::motors::Position>),
    StopMotor(Option<crate::motors::Position>),
    ToggleLed,
}

impl core::convert::TryFrom<u8> for Command {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Command::ToggleLed),
            10 => Ok(Command::StartMotor(None)),
            11 => Ok(Command::StopMotor(None)),

            _ => Err(()),
        }
    }
}

use rc_framing::framing;

/// Returns true if the recived byte is the end byte of a frame
pub fn check_frame_end(last_byte: u8) -> bool {
    last_byte == framing::END
}

/// Try to decode the buffer and return a Command if successfull
pub fn decode_buffer(rec: &[u8]) -> Result<Command, ()> {
    let mut buffer = [0_u8; 64];

    // Decode the frame
    framing::decode(rec, &mut buffer).map_err(|_| ())?;

    //

    Err(())
}
