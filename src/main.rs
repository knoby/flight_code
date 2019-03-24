#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Define Panic behaivior
// extern crate panic_halt;
extern crate panic_semihosting;

// Used traits from the HAL crate
use alt_stm32f30x_hal::prelude::*;

// Message Passing between Idle, Interrupt and Periodic
use heapless::{
    consts::*,
    spsc::{Consumer, Producer, Queue},
};

mod hardware;

// Runtime Imports
use rtfm::{app, Instant};

#[app(device = alt_stm32f30x_hal::stm32f30x)]
const APP: () = {
    //Resourcen

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
        let device: alt_stm32f30x_hal::stm32f30x::Peripherals = device;

        //Freeze clock frequencies
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .sysclk(64.mhz())
            .pclk1(32.mhz()) //TODO: Fix bug in hal crate to use different clocks for pclk1/pclk2
            .pclk2(32.mhz())
            .freeze(&mut flash.acr);

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
        PSerialRead = ps;
        CSerialRead = cs;

        PTargetPoint = pt;
        CTargetPoint = ct;
    }

    /// Idle Task for non critical jobs
    #[idle(resources = [CSerialRead, PTargetPoint])]
    fn idle() -> ! {
        // Buffer for message evaluation
        let mut in_buffer = [0_u8; 32];
        let mut rec_len = 0;
        let mut msg = [0_u8; 32];

        loop {
            cortex_m::asm::nop();
        }
    }

    /// Periodic task for real time critical things
    #[task(priority = 5 , resources = [CTargetPoint], schedule = [periodic_task])]
    fn periodic_task() {
        //Rescedule task
        schedule
            .periodic_task(scheduled + 32_000_000.cycles())
            .unwrap();
    }

    /// Interrupt for reading data from serial interface
    #[interrupt(priority = 10, resources = [ PSerialRead])]
    fn USART2_EXTI26() {}

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn EXTI0();
    }
};
