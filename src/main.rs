#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Define Panic behaivior
//extern crate panic_halt;
extern crate panic_semihosting;

// Used traits from the HAL crate
extern crate alt_stm32f30x_hal as hal;
use hal::prelude::*;

#[macro_use(block)]
extern crate nb;

// Message Passing between Idle, Interrupt and Periodic
use heapless::{
    consts::*,
    spsc::{Consumer, Producer, Queue},
};

extern crate cortex_m_semihosting;
use cortex_m_semihosting::hprintln;

mod fc;
mod led;
mod motors;
mod position;
mod serial;

// Runtime Imports
use rtfm::app;

// Program Constants

#[app(device = hal::device, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // Que for passing data from Serial Interrupt to Idle Task
        PROD_SERIAL_READ: Producer<'static, u8, U4>,
        CONS_SERIAL_READ: Consumer<'static, u8, U4>,

        PROD_MOTION_TO_IDLE: Producer<'static, (f32, f32, f32), U4>,
        CONS_MOTION_TO_IDLE: Consumer<'static, (f32, f32, f32), U4>,

        // Communication to Base Station
        SERIAL_RX: serial::SerialRx,
        SERIAL_TX: serial::SerialTx,

        // Sensor Data
        SENSORS: position::Sensors,

        // Motor Control
        MOTORS: motors::Motors,

        // LEDs
        LED_N: led::LedN,
        LED_NE: led::LedNE,
        LED_E: led::LedE,

        // Flight Controller
        FC: fc::FlighController,
    }

    #[init()]
    fn init(mut cx: init::Context) -> init::LateResources {
        //Create Ringbuffer at beginning of init function
        static mut Q_SERIAL_READ: Option<Queue<u8, U4>> = None;
        static mut Q_MOTION_TO_IDLE: Option<Queue<(f32, f32, f32), U4>> = None;

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
        let led_ne = gpioe.pe10.output().output_speed(hal::gpio::MediumSpeed);
        let led_e = gpioe.pe11.output().output_speed(hal::gpio::MediumSpeed);

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

        // Create Pins for PWM Output to control the ESC
        let gpioc = cx.device.GPIOC.split(&mut rcc.ahb);
        let pwm_pin_motor_vl = gpioc
            .pc6
            .alternating(hal::gpio::AF2)
            .output_speed(hal::gpio::HighSpeed);
        let pwm_pin_motor_vr = gpioc
            .pc7
            .alternating(hal::gpio::AF2)
            .output_speed(hal::gpio::HighSpeed);
        let pwm_pin_motor_hl = gpioc
            .pc8
            .alternating(hal::gpio::AF2)
            .output_speed(hal::gpio::HighSpeed);
        let pwm_pin_motor_hr = gpioc
            .pc9
            .alternating(hal::gpio::AF2)
            .output_speed(hal::gpio::HighSpeed);

        //Create que for serial read
        *Q_SERIAL_READ = Some(Queue::new());
        let (ps, cs) = Q_SERIAL_READ.as_mut().unwrap().split();

        //Create que for serial read
        *Q_MOTION_TO_IDLE = Some(Queue::new());
        let (psmt, csmt) = Q_MOTION_TO_IDLE.as_mut().unwrap().split();

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

        //Assign late resources
        init::LateResources {
            PROD_SERIAL_READ: ps,
            CONS_SERIAL_READ: cs,

            PROD_MOTION_TO_IDLE: psmt,
            CONS_MOTION_TO_IDLE: csmt,

            SERIAL_RX: rx,
            SERIAL_TX: tx,

            SENSORS: position::Sensors::new(acc_sensor, gyro_sensor, l3gd20::Scale::Dps2000),
            MOTORS: motors::Motors::new(
                pwm_pin_motor_vl,
                pwm_pin_motor_vr,
                pwm_pin_motor_hl,
                pwm_pin_motor_hr,
                cx.device.TIM3,
                clocks,
            ),
            LED_N: led_n,
            LED_NE: led_ne,
            LED_E: led_e,
            FC: fc::FlighController::default(),
        }
    }

    /// Idle Task for non critical jobs
    #[idle(resources = [CONS_SERIAL_READ, CONS_MOTION_TO_IDLE, LED_N, LED_NE, LED_E, SERIAL_TX])]
    fn idle(cx: idle::Context) -> ! {
        let mut buffer = heapless::Vec::<u8, U32>::new();
        let mut angle = (0.0, 0.0, 0.0);
        loop {
            // Toggle LED for Idle Mode is running
            cx.resources.LED_NE.toggle().unwrap();
            if let Some(val) = cx.resources.CONS_MOTION_TO_IDLE.dequeue() {
                angle = val;
            }
            if let Some(val) = cx.resources.CONS_SERIAL_READ.dequeue() {
                // Read from que
                if serial::check_frame_end(val) {
                    // Check if byte is end from frame
                    if let Ok(cmd) = copter_defs::Command::from_slip(&buffer) {
                        // Toggle LED for Reciving Data
                        cx.resources.LED_E.toggle().unwrap();
                        // Try to decode
                        use copter_defs::Command::*;
                        match cmd {
                            // Execute Command
                            ToggleLed => cx.resources.LED_N.toggle().unwrap(),
                            GetMotionState => {
                                let test_state = SendMotionState(nalgebra::Vector3::new(
                                    angle.0, angle.1, angle.2,
                                ));
                                test_state
                                    .to_slip(&mut buffer)
                                    .and_then(|_| {
                                        buffer
                                            .iter()
                                            .try_for_each(|byte| {
                                                block!(cx.resources.SERIAL_TX.write(*byte))
                                            })
                                            .map_err(|_| ())
                                    })
                                    .ok(); // Ignore if not working
                            }
                            _ => (),
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
    #[task(binds = TIM3, priority = 5 , resources = [FC, SENSORS, PROD_MOTION_TO_IDLE, MOTORS])]
    fn periodic_task(cx: periodic_task::Context) {
        // Update orientation
        cx.resources.SENSORS.update(cx.resources.MOTORS.period());

        let (vl, vr, hl, hr) = cx.resources.FC.update(
            cx.resources.SENSORS.angle_vel(),
            cx.resources.SENSORS.euler_angles(),
            cx.resources.MOTORS.period(),
        );

        cx.resources.MOTORS.set_speed(vl, vr, hl, hr);

        cx.resources
            .PROD_MOTION_TO_IDLE
            .enqueue(cx.resources.SENSORS.euler_angles())
            .ok();
        // Reset the ISR Flag
        cx.resources.MOTORS.reset_isr_flag();
    }

    /// Interrupt for reciving bytes from serial and sending to idle task
    #[task(binds = USART1_EXTI25, priority = 8, resources = [SERIAL_RX, PROD_SERIAL_READ])]
    fn read_serial_byte(cx: read_serial_byte::Context) {
        match cx.resources.SERIAL_RX.read() {
            Ok(b) => {
                // Send Data to main task
                cx.resources.PROD_SERIAL_READ.enqueue(b).ok(); // Do not care if full
            }
            Err(nb::Error::Other(e)) => {
                if let hal::serial::Error::Overrun = e {
                    cx.resources.SERIAL_RX.clear_overrun_error();
                } else {
                    // Ignore other Errors
                }
            }
            Err(nb::Error::WouldBlock) => {} // Ignore errors
        }
    }

    /// Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn EXTI0();
    }
};
