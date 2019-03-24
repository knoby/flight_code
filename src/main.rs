#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Define Panic behaivior
// extern crate panic_halt;
extern crate panic_semihosting;

// Used traits from the HAL crate
extern crate alt_stm32f30x_hal as hal;
use hal::prelude::*;

// Message Passing between Idle, Interrupt and Periodic
use heapless::{
    consts::*,
    spsc::{Consumer, Producer, Queue},
};

mod led;
//mod motors;
mod position;

// Runtime Imports
use rtfm::{app, Instant};

#[app(device = hal::stm32f30x)]
const APP: () = {
    //Resourcen

    /// Que for passing data from Serial Interrupt to Idle Task
    static mut PSerialRead: Producer<'static, Option<u8>, U4> = ();
    static mut CSerialRead: Consumer<'static, Option<u8>, U4> = ();

    /// Que for passing target positions from Idel Task to Periodic Task
    static mut PTargetPoint: Producer<'static, [f32; 3], U4> = ();
    static mut CTargetPoint: Consumer<'static, [f32; 3], U4> = ();

    /// Sensor Data
    static mut Sensors: position::Sensors = ();
    static mut LEDs: led::Leds = ();

    #[init(schedule = [periodic_task])]
    fn init() {
        //Create Ringbuffer at beginning of init function
        //TODO: check WHY
        static mut QSerialRead: Option<Queue<Option<u8>, U4>> = None;
        static mut QTargetPoint: Option<Queue<[f32; 3], U4>> = None;

        // Cortex-M peripherals
        let _core: rtfm::Peripherals = core;

        // Device specific peripherals
        let device: hal::stm32f30x::Peripherals = device;

        //Freeze clock frequencies
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .sysclk(64.mhz())
            .pclk1(32.mhz())
            .pclk2(64.mhz())
            .freeze(&mut flash.acr);

        // Setup the I2C Bus for the Magneto and Accelero Meter
        let gpiob = device.GPIOB.split(&mut rcc.ahb);
        let scl = gpiob.pb6.alternating(hal::gpio::AF4);
        let sda = gpiob.pb7.alternating(hal::gpio::AF4);
        let i2c = device.I2C1.i2c((scl, sda), 400.khz(), clocks);

        let acc_sensor = lsm303dlhc::Lsm303dlhc::new(i2c).unwrap();

        // Setup the on board LEDs
        let gpioe = device.GPIOE.split(&mut rcc.ahb);
        let leds = led::Leds::new(gpioe);

        //Create que for serial read
        *QSerialRead = Some(Queue::new());
        let (ps, cs) = QSerialRead.as_mut().unwrap().split();

        //Create que for target points
        *QTargetPoint = Some(Queue::new());
        let (pt, ct) = QTargetPoint.as_mut().unwrap().split();

        // Schedule Periodic Task
        schedule
            .periodic_task(Instant::now() + 64_000_000.cycles())
            .unwrap();

        //Assign late resources
        PSerialRead = ps;
        CSerialRead = cs;

        PTargetPoint = pt;
        CTargetPoint = ct;

        Sensors = position::Sensors { acc: acc_sensor };
        LEDs = leds;
    }

    /// Idle Task for non critical jobs
    #[idle(resources = [CSerialRead, PTargetPoint])]
    fn idle() -> ! {
        // Buffer for message evaluation
        let _in_buffer = [0_u8; 32];
        let _rec_len = 0;
        let _msg = [0_u8; 32];

        loop {
            cortex_m::asm::nop();
        }
    }

    /// Periodic task for real time critical things
    #[task(priority = 5 , resources = [CTargetPoint, Sensors, LEDs], schedule = [periodic_task])]
    fn periodic_task() {
        static mut led_status: bool = false;

        //Try to read Sensor Data
        resources.Sensors.acc.temp().unwrap();

        // Toggle LEDs
        for led in resources.LEDs.iter_mut() {
            if *led_status {
                led.off();
            } else {
                led.on();
            }
        }

        *led_status = !*led_status;

        //Rescedule task
        schedule
            .periodic_task(scheduled + 64_000_000.cycles())
            .unwrap();
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn EXTI0();
    }
};
