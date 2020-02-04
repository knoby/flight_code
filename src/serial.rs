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

use rc_framing::framing;

/// Returns true if the recived byte is the end byte of a frame
pub fn check_frame_end(last_byte: u8) -> bool {
    last_byte == framing::END
}
