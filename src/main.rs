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

use rtfm::cyccnt::U32Ext;

mod led;
mod motors;
mod position;
mod serial;

// Runtime Imports
use rtfm::app;

// Program Constants
const CONTROL_CYCLE: u32 = 640_000; // Sysclock = 64 Mhz --> 64_000_000 / 100 = 640_000 Cycles per Task = 10ms

#[app(device = hal::device, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // Que for passing data from Serial Interrupt to Idle Task
        PSerialRead: Producer<'static, u8, U4>,
        CSerialRead: Consumer<'static, u8, U4>,

        // Communication to Base Station
        SerialRx: serial::SerialRx,
        SerialTx: serial::SerialTx,

        // Sensor Data
        Sensors: position::Sensors,

        // LEDs
        LED_N: led::LedN,
    }

    #[init(schedule = [periodic_task])]
    fn init(mut cx: init::Context) -> init::LateResources {
        //Create Ringbuffer at beginning of init function
        static mut Q_SERIAL_READ: Option<Queue<u8, U4>> = None;

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

        // Setup the on board LEDs
        let gpioe = cx.device.GPIOE.split(&mut rcc.ahb);
        let led_n = gpioe.pe9.output().output_speed(hal::gpio::MediumSpeed);

        // Setup the I2C Bus for the Magneto and Accelero Meter
        let gpiob = cx.device.GPIOB.split(&mut rcc.ahb);
        let scl = gpiob.pb6.alternating(hal::gpio::AF4);
        let sda = gpiob.pb7.alternating(hal::gpio::AF4);
        let i2c = cx.device.I2C1.i2c((scl, sda), 400.khz(), clocks);

        let acc_sensor = lsm303dlhc::Lsm303dlhc::new(i2c).unwrap();

        // Setup the Spi bus forthe Gyro
        let gpioa = cx.device.GPIOA.split(&mut rcc.ahb);
        let sck = gpioa
            .pa5
            .alternating(hal::gpio::AF5)
            .output_speed(hal::gpio::HighSpeed);
        let miso = gpioa
            .pa6
            .alternating(hal::gpio::AF5)
            .output_speed(hal::gpio::HighSpeed);
        let mosi = gpioa
            .pa7
            .alternating(hal::gpio::AF5)
            .output_speed(hal::gpio::HighSpeed);
        let nss = gpioe.pe3.output().output_speed(hal::gpio::HighSpeed);
        let spi = cx
            .device
            .SPI1
            .spi((sck, miso, mosi), l3gd20::MODE, 1.mhz(), clocks);

        let gyro_sensor = l3gd20::L3gd20::new(spi, nss).unwrap();

        // Setup PWM Outputs for Servor Motors
        let gpioc = cx.device.GPIOC.split(&mut rcc.ahb);
        let timer = alt_stm32f30x_hal::timer::tim3::Timer::new(cx.device.TIM3, 100.hz(), clocks);
        let (channels, _timer) = timer.use_pwm();
        let mut pwm_pin_motor_vl = gpioc
            .pc6
            .alternating(hal::gpio::AF1)
            .output_speed(hal::gpio::HighSpeed)
            .to_pwm(channels.0, hal::gpio::HighSpeed);
        let _pwm_pin_motor_vr = gpioc
            .pc7
            .alternating(hal::gpio::AF1)
            .output_speed(hal::gpio::HighSpeed)
            .to_pwm(channels.1, hal::gpio::HighSpeed);
        let _pwm_pin_motor_hl = gpioc
            .pc8
            .alternating(hal::gpio::AF1)
            .output_speed(hal::gpio::HighSpeed)
            .to_pwm(channels.2, hal::gpio::HighSpeed);
        let _pwm_pin_motor_hr = gpioc
            .pc9
            .alternating(hal::gpio::AF1)
            .output_speed(hal::gpio::HighSpeed)
            .to_pwm(channels.3, hal::gpio::HighSpeed);

        pwm_pin_motor_vl.set_duty(9);

        //Create que for serial read
        *Q_SERIAL_READ = Some(Queue::new());
        let (ps, cs) = Q_SERIAL_READ.as_mut().unwrap().split();

        // Create USART Port for communication to remote station
        let tx_pin = gpioc
            .pc4
            .alternating(hal::gpio::AF7)
            .output_speed(hal::gpio::HighSpeed);
        let rx_pin = gpioc
            .pc5
            .alternating(hal::gpio::AF7)
            .output_speed(hal::gpio::HighSpeed);
        let serial = cx
            .device
            .USART1
            .serial((tx_pin, rx_pin), hal::time::Bps(9600), clocks);
        let (tx, rx) = serial::create_tx_rx(serial);

        // Schedule Periodic Task
        cx.schedule
            .periodic_task(cx.start + CONTROL_CYCLE.cycles())
            .unwrap();

        //Assign late resources
        init::LateResources {
            PSerialRead: ps,
            CSerialRead: cs,

            SerialRx: rx,
            SerialTx: tx,

            Sensors: position::Sensors {
                acc: acc_sensor,
                gyro: gyro_sensor,
            },
            LED_N: led_n,
        }
    }

    /// Idle Task for non critical jobs
    #[idle(resources = [CSerialRead, LED_N])]
    fn idle(cx: idle::Context) -> ! {
        let mut buffer = heapless::Vec::<u8, U32>::new();
        loop {
            if let Some(val) = cx.resources.CSerialRead.dequeue() {
                // Read from que
                if serial::check_frame_end(val) {
                    // Check if byte is end from frame
                    if let Ok(cmd) = serial::decode_buffer(buffer.as_ref()) {
                        // Try to decode
                        use serial::Command;
                        match cmd {
                            // Execute Command
                            Command::ToggleLed => cx.resources.LED_N.toggle().unwrap(),
                            Command::StopMotor(_) => (),
                            Command::StartMotor(_) => (),
                        }
                    }
                    buffer.clear(); // Reset Index
                } else {
                    // Try to Add to buffer. Do not care if full
                    buffer.push(val).ok();
                }
            }
        }
    }

    /// Periodic task for real time critical things
    #[task(priority = 5 , resources = [Sensors], schedule = [periodic_task])]
    fn periodic_task(cx: periodic_task::Context) {
        //Rescedule task
        cx.schedule
            .periodic_task(cx.scheduled + CONTROL_CYCLE.cycles())
            .unwrap();
    }

    /// Interrupt for reciving bytes from serial and sending to idle task
    #[task(binds = USART1_EXTI25, priority = 8, resources = [SerialRx, PSerialRead])]
    fn read_serial_byte(cx: read_serial_byte::Context) {
        match cx.resources.SerialRx.read() {
            Ok(b) => {
                // Send Data to main task
                cx.resources.PSerialRead.enqueue(b).ok(); // Do not care if full
            }
            Err(nb::Error::Other(e)) => {
                if let hal::serial::Error::Overrun = e {
                    cx.resources.SerialRx.clear_overrun_error();
                } else {
                    // Ignore other Errors
                }
            }
            Err(nb::Error::WouldBlock) => {} // Ignore errors
        }
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn EXTI0();
    }
};
