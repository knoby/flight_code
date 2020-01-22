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

use rtfm::cyccnt::{Instant, U32Ext};

mod led;
//mod motors;
mod position;

// Runtime Imports
use rtfm::app;

#[app(device = hal::device, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // Que for passing data from Serial Interrupt to Idle Task
        PSerialRead: Producer<'static, Option<u8>, U4>,
        CSerialRead: Consumer<'static, Option<u8>, U4>,

        // Sensor Data
        Sensors: position::Sensors,
        LEDs: led::Leds,
    }

    #[init(schedule = [periodic_task])]
    fn init(mut cx: init::Context) -> init::LateResources {
        //Create Ringbuffer at beginning of init function
        static mut QSerialRead: Option<Queue<Option<u8>, U4>> = None;

        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        //Freeze clock frequencies
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .sysclk(64.mhz())
            .pclk1(32.mhz())
            .pclk2(64.mhz())
            .freeze(&mut flash.acr);

        // Setup the I2C Bus for the Magneto and Accelero Meter
        let gpiob = cx.device.GPIOB.split(&mut rcc.ahb);
        let scl = gpiob.pb6.alternating(hal::gpio::AF4);
        let sda = gpiob.pb7.alternating(hal::gpio::AF4);
        let i2c = cx.device.I2C1.i2c((scl, sda), 400.khz(), clocks);

        let acc_sensor = lsm303dlhc::Lsm303dlhc::new(i2c).unwrap();

        // Setup the on board LEDs
        let gpioe = cx.device.GPIOE.split(&mut rcc.ahb);
        let leds = led::Leds::new(gpioe);

        //Create que for serial read
        *QSerialRead = Some(Queue::new());
        let (ps, cs) = QSerialRead.as_mut().unwrap().split();

        // Schedule Periodic Task
        cx.schedule
            .periodic_task(cx.start + 64_000_000.cycles())
            .unwrap();

        //Assign late resources
        init::LateResources {
            PSerialRead: ps,
            CSerialRead: cs,

            Sensors: position::Sensors { acc: acc_sensor },
            LEDs: leds,
        }
    }

    /// Idle Task for non critical jobs
    #[idle(resources = [CSerialRead])]
    fn idle(_cx: idle::Context) -> ! {
        // Buffer for message evaluation
        let _in_buffer = [0_u8; 32];
        let _rec_len = 0;
        let _msg = [0_u8; 32];

        loop {
            cortex_m::asm::nop();
        }
    }

    /// Periodic task for real time critical things
    #[task(priority = 5 , resources = [Sensors, LEDs], schedule = [periodic_task])]
    fn periodic_task(cx: periodic_task::Context) {
        static mut led_status: bool = false;

        //Try to read Sensor Data
        cx.resources.Sensors.acc.temp().unwrap();

        // Toggle LEDs
        for led in cx.resources.LEDs.iter_mut() {
            if *led_status {
                led.off();
            } else {
                led.on();
            }
        }

        *led_status = !*led_status;

        //Rescedule task
        cx.schedule
            .periodic_task(cx.scheduled + 64_000_000.cycles())
            .unwrap();
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn EXTI0();
    }
};
