#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Define Panic behaivior
extern crate panic_semihosting;

// Used traits from the HAL crate
use stm32f1xx_hal::{
    gpio,
    prelude::*,
    serial::{Event, Serial},
    stm32,
};

// Asynchronous API
use nb;

// Message Passing between Idle, Interrupt and Periodic
use heapless::{
    consts::*,
    spsc::{Consumer, Producer, Queue},
};

use rtfm::app;

#[app(device = stm32f1xx_hal::stm32)]
const APP: () = {
    //Resourcen
    static mut LD2: gpio::gpioa::PA5<gpio::Output<gpio::PushPull>> = ();
    static mut PB: gpio::gpioc::PC13<gpio::Input<gpio::Floating>> = ();
    static mut TX: stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART2> = ();
    static mut RX: stm32f1xx_hal::serial::Rx<stm32f1xx_hal::stm32::USART2> = ();
    static mut P: Producer<'static, Option<u8>, U8> = ();
    static mut C: Consumer<'static, Option<u8>, U8> = ();

    #[init]
    fn init() {
        //Create Ringbuffer at beginning of init function --> error if not
        static mut Q: Option<Queue<Option<u8>, U8>> = None;

        // Cortex-M peripherals
        let _core: rtfm::Peripherals = core;

        // Device specific peripherals
        let device: stm32::Peripherals = device;

        //Freeze clock frequencies
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        let clocks = rcc
            .cfgr
            .sysclk(64.mhz())
            .pclk1(32.mhz())
            .pclk2(32.mhz())
            .freeze(&mut flash.acr);

        //Configuration of PA5 as output and PC13 as Input
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);

        //Configuration of the Pins TX and RX for UART2
        let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let rx = gpioa.pa3.into_floating_input(&mut gpioa.crl);

        let mut serial = Serial::usart2(
            device.USART2,
            (tx, rx),
            &mut afio.mapr,
            9_600.bps(),
            clocks,
            &mut rcc.apb1,
        );

        //Enable Interrupt for USART2
        serial.listen(Event::Rxne);
        let (tx, rx) = serial.split();

        //Create que for serial read
        *Q = Some(Queue::new());
        let (p, c) = Q.as_mut().unwrap().split();

        //Assign late resources
        LD2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);
        PB = gpioc.pc13.into_floating_input(&mut gpioc.crh);
        TX = tx;
        RX = rx;

        P = p;
        C = c;
    }

    #[idle(resources = [C, TX])]
    fn idle() -> ! {
        loop {
            if let Some(item) = resources.C.dequeue() {
                if let Some(byte) = item {
                    nb::block!(resources.TX.write(byte)).unwrap();
                } else {
                    nb::block!(resources.TX.write(0x45)).unwrap();
                    nb::block!(resources.TX.write(0x52)).unwrap();
                    nb::block!(resources.TX.write(0x52)).unwrap();
                    nb::block!(resources.TX.write(0x4F)).unwrap();
                    nb::block!(resources.TX.write(0x52)).unwrap();
                    nb::block!(resources.TX.write(0x21)).unwrap();
                }
            }
        }
    }

    #[interrupt(priority = 10, resources = [RX, P])]
    fn USART2() {
        //Read the recived Byte from the interface and push it in the
        if let Ok(byte) = resources.RX.read() {
            resources.P.enqueue(Some(byte)).is_err();
        } else {
            resources.P.enqueue(None).is_err();
        }
    }
};
