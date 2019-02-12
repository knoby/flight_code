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

// Runtime Imports
use rtfm::{app, Instant};

#[app(device = stm32f1xx_hal::stm32)]
const APP: () = {
    //Resourcen

    /// On Board LED
    static mut LED: gpio::gpioa::PA5<gpio::Output<gpio::PushPull>> = ();

    /// Transmission Pin for Serial Interface
    static mut TX: stm32f1xx_hal::serial::Tx<stm32f1xx_hal::stm32::USART2> = ();
    /// Resive Pin for Serial Interface
    static mut RX: stm32f1xx_hal::serial::Rx<stm32f1xx_hal::stm32::USART2> = ();

    /// Que for passing data from Serial Interrupt to Idle Task
    static mut PSerialRead: Producer<'static, Option<u8>, U4> = ();
    static mut CSerialRead: Consumer<'static, Option<u8>, U4> = ();

    /// Que for passing target positions from Idel Task to Periodic Task
    static mut PTargetPoint: Producer<'static, [f32; 3], U4> = ();
    static mut CTargetPoint: Consumer<'static, [f32; 3], U4> = ();

    #[init(schedule = [periodic_task])]
    fn init() {
        //Create Ringbuffer at beginning of init function
        //TODO: check WHY
        static mut QSerialRead: Option<Queue<Option<u8>, U4>> = None;
        static mut QTargetPoint: Option<Queue<[f32; 3], U4>> = None;

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
            .pclk1(32.mhz()) //TODO: Fix bug in hal crate to use different clocks for pclk1/pclk2
            .pclk2(32.mhz())
            .freeze(&mut flash.acr);

        //Configuration of the Pins TX and RX for UART2
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
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
        *QSerialRead = Some(Queue::new());
        let (ps, cs) = QSerialRead.as_mut().unwrap().split();

        //Create que for target points
        *QTargetPoint = Some(Queue::new());
        let (pt, ct) = QTargetPoint.as_mut().unwrap().split();

        // Schedule Periodic Task
        schedule
            .periodic_task(Instant::now() + 32_000_000.cycles())
            .unwrap();

        //Assign late resources
        LED = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);
        TX = tx;
        RX = rx;

        PSerialRead = ps;
        CSerialRead = cs;

        PTargetPoint = pt;
        CTargetPoint = ct;
    }

    /// Idle Task for non critical jobs
    #[idle(resources = [CSerialRead, PTargetPoint, TX])]
    fn idle() -> ! {
        loop {
            if let Some(item) = resources.CSerialRead.dequeue() {
                if let Some(byte) = item {
                    nb::block!(resources.TX.write(byte)).unwrap();
                }
            }
        }
    }

    /// Periodic task for real time critical things
    #[task(priority = 5 , resources = [CTargetPoint, LED], schedule = [periodic_task])]
    fn periodic_task() {
        //Toggle LED
        resources.LED.toggle();

        //Rescedule task
        schedule
            .periodic_task(scheduled + 32_000_000.cycles())
            .unwrap();
    }

    /// Interrupt for reading data from serial interface
    #[interrupt(priority = 10, resources = [RX, PSerialRead])]
    fn USART2() {
        //Read the recived Byte from the interface and push it in the queue
        if let Ok(byte) = resources.RX.read() {
            resources.PSerialRead.enqueue(Some(byte)).is_err();
        } else {
            resources.PSerialRead.enqueue(None).is_err();
        }
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn EXTI0();
    }
};
