#[cfg(not(feature = "serial_usb"))]
mod serial_bt {
    pub type SerialTxPin = hal::gpio::gpiod::PD5<hal::gpio::AF7>;

    pub type SerialRxPin = hal::gpio::gpiod::PD6<hal::gpio::AF7>;

    pub type SerialUart = hal::stm32::USART2;
}

#[cfg(feature = "serial_usb")]
mod serial_usb {
    pub type SerialTxPin = hal::gpio::gpioc::PC4<hal::gpio::AF7>;

    pub type SerialRxPin = hal::gpio::gpioc::PC5<hal::gpio::AF7>;

    pub type SerialUart = hal::stm32::USART1;
}

#[cfg(not(feature = "serial_usb"))]
use serial_bt as serial_types;
#[cfg(feature = "serial_usb")]
use serial_usb as serial_types;

use serial_types::*;

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

/// Returns true if the recived byte is the end byte of a frame
pub fn check_frame_end(last_byte: u8) -> bool {
    last_byte == 13
}
