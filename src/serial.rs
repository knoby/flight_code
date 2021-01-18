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

#[cfg(feature = "serial_usb")]
pub type SerialTxDMA = hal::dma::dma1::C4;
#[cfg(not(feature = "serial_usb"))]
pub type SerialTxDMA = hal::dma::dma1::C7;

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

/// Function called from a task in the app
pub fn serial_send(cx: crate::serial_send::Context, msg: copter_com::Message) {
    // Take ownership of resources
    let serial = cx.resources.SERIAL_TX.take().unwrap();
    let dma = cx.resources.DMA_SERIAL_TX.take().unwrap();
    let dma_buffer = cx.resources.DMA_BUFFER_TX.take().unwrap();

    // Serialize message
    let buffer = msg.serialize();

    // Clear buffer
    for byte in dma_buffer.iter_mut() {
        *byte = 0;
    }
    // Set message
    for (msg_byte, buffer_byte) in buffer.iter().zip(dma_buffer.iter_mut()) {
        *buffer_byte = *msg_byte;
    }

    let (dma_buffer, dma, serial) = serial.write_all(dma_buffer, dma).wait();

    *cx.resources.SERIAL_TX = Some(serial);
    *cx.resources.DMA_BUFFER_TX = Some(dma_buffer);
    *cx.resources.DMA_SERIAL_TX = Some(dma);
}
