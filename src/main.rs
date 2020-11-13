#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Global Logger
use defmt_rtt as _;
// Define Panic behaivior
use panic_probe as _;

// Used traits from the HAL crate
extern crate stm32f3xx_hal as hal;
use hal::prelude::*;

// Logger macros
use defmt::debug;

// Message Passing between Idle, Interrupt and Periodic
use heapless::consts::{U128, U32, U4};
use heapless::spsc::{Consumer, Producer, Queue};

// Local modules
mod fc;
mod led;
mod motors;
mod position;
mod serial;

// Runtime Imports
use rtic::app;

// Program Constants
const CORE_FREQUENCY: u32 = 64_000_000;
const CONTROL_LOOP_PERIOD: u32 = CORE_FREQUENCY / 100;
const MOTOR_ARMING_TIME: u32 = CORE_FREQUENCY / 1;

#[app(device = hal::stm32 ,peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // Queue for passing data from Serial Interrupt to Idle Task
        PROD_SERIAL_READ: Producer<'static, u8, U32>,
        CONS_SERIAL_READ: Consumer<'static, u8, U32>,

        // Queue for passing application commands
        PROD_APP_CMD: Producer<'static, fc::AppCommand, U4>,
        CONS_APP_CMD: Consumer<'static, fc::AppCommand, U4>,

        // Communication to Base Station
        SERIAL_RX: serial::SerialRx,
        SERIAL_TX: serial::SerialTx,

        // Sensor Data
        SENSORS: position::Sensors,

        STATE: fc::State,
        SETPOINT: fc::SetValues,

        // Motor Control
        MOTORS: motors::Motors,

        // LEDs
        LED_N: led::LedN,
        LED_NE: led::LedNE,
        LED_E: led::LedE,
        LED_SE: led::LedSE,
        LED_S: led::LedS,
        LED_SW: led::LedSW,
        LED_W: led::LedW,
        LED_NW: led::LedNW,
    }

    #[init(spawn=[state_estimation])]
    fn init(mut cx: init::Context) -> init::LateResources {
        //Create Ringbuffer at beginning of init function
        static mut Q_SERIAL_READ: Option<Queue<u8, U32>> = None;
        static mut Q_APP_COMMANDS: Option<Queue<fc::AppCommand, U4>> = None;

        debug!("Rustocupter Wakeup after reset.");
        debug!("Start Init...");

        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        //Freeze clock frequencies
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        debug!("Setting Up Clocks");
        let clocks = rcc
            .cfgr
            .sysclk(CORE_FREQUENCY.hz())
            .pclk1(32.mhz())
            .pclk2(64.mhz())
            .freeze(&mut flash.acr);

        // Activate GPIOS that are needed
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb);
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.ahb);
        let mut gpiod = cx.device.GPIOD.split(&mut rcc.ahb);
        let mut gpioe = cx.device.GPIOE.split(&mut rcc.ahb);

        debug!("Config of on bord Leds");
        // Setup the on board LEDs
        let led_n = gpioe
            .pe9
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        let led_ne = gpioe
            .pe10
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        let led_e = gpioe
            .pe11
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        let led_se = gpioe
            .pe12
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        let led_s = gpioe
            .pe13
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        let led_sw = gpioe
            .pe14
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        let led_w = gpioe
            .pe15
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        let led_nw = gpioe
            .pe8
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

        debug!("Setup I2C Bus for acceleration sensor");
        // Setup the I2C Bus for the Magneto and Accelero Meter
        let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
        let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
        let i2c = hal::i2c::I2c::i2c1(cx.device.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1);

        let acc_sensor = lsm303dlhc::Lsm303dlhc::new(i2c).unwrap();

        debug!("Setup SPI Bus for gyro sensor");
        // Setup the Spi bus forthe Gyro
        let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
        let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
        let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
        let nss = gpioe
            .pe3
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        let spi = hal::spi::Spi::spi1(
            cx.device.SPI1,
            (sck, miso, mosi),
            l3gd20::MODE,
            1.mhz(),
            clocks,
            &mut rcc.apb2,
        );

        let gyro_sensor = l3gd20::L3gd20::new(spi, nss).unwrap();

        debug!("Configuration of PWM Output pins");
        // Create Pins for PWM Output to control the ESC
        let pwm_pin_motor_vl = gpioc.pc6.into_af2(&mut gpioc.moder, &mut gpioc.afrl);
        let pwm_pin_motor_vr = gpioc.pc7.into_af2(&mut gpioc.moder, &mut gpioc.afrl);
        let pwm_pin_motor_hl = gpioc.pc8.into_af2(&mut gpioc.moder, &mut gpioc.afrh);
        let pwm_pin_motor_hr = gpioc.pc9.into_af2(&mut gpioc.moder, &mut gpioc.afrh);

        debug!("Setup Communication Channel for Serial data to idle task");
        //Create que for serial read
        *Q_SERIAL_READ = Some(Queue::new());
        let (ps, cs) = Q_SERIAL_READ.as_mut().unwrap().split();

        debug!("Setup Communication Channel for Application Commands");
        *Q_APP_COMMANDS = Some(Queue::new());
        let (pa, ca) = Q_APP_COMMANDS.as_mut().unwrap().split();

        debug!("Configuration of serial interface");
        // Create USART Port for communication to remote station
        #[cfg(not(feature = "serial_usb"))]
        let serial = {
            let tx_pin = gpiod.pd5.into_af7(&mut gpiod.moder, &mut gpiod.afrl);
            let rx_pin = gpiod.pd6.into_af7(&mut gpiod.moder, &mut gpiod.afrl);
            hal::serial::Serial::usart2(
                cx.device.USART2,
                (tx_pin, rx_pin),
                hal::time::Bps(38400),
                clocks,
                &mut rcc.apb1,
            )
        };
        #[cfg(feature = "serial_usb")]
        let serial = {
            let tx_pin = gpioc.pc4.into_af7(&mut gpioc.moder, &mut gpioc.afrl);
            let rx_pin = gpioc.pc5.into_af7(&mut gpioc.moder, &mut gpioc.afrl);
            hal::serial::Serial::usart1(
                cx.device.USART1,
                (tx_pin, rx_pin),
                hal::time::Bps(9600),
                clocks,
                &mut rcc.apb2,
            )
        };

        let (tx, rx) = serial::create_tx_rx(serial);

        // When finished start the periodic tast
        cx.spawn.state_estimation().unwrap();

        //Assign late resources
        init::LateResources {
            PROD_SERIAL_READ: ps,
            CONS_SERIAL_READ: cs,

            PROD_APP_CMD: pa,
            CONS_APP_CMD: ca,

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

            STATE: fc::State::default(),
            SETPOINT: fc::SetValues::DirectControl([0.0; 4]),

            LED_N: led_n,
            LED_NE: led_ne,
            LED_E: led_e,
            LED_SE: led_se,
            LED_S: led_s,
            LED_SW: led_sw,
            LED_W: led_w,
            LED_NW: led_nw,
        }
    }

    // ====================================================
    // ==== Background and communication tasks ====
    // ====================================================

    /// Idle Task for non critical jobs
    #[idle(resources = [CONS_SERIAL_READ, LED_NE, LED_E, SERIAL_TX, PROD_APP_CMD ])]
    fn idle(cx: idle::Context) -> ! {
        debug!("Entering Idle Task");
        let mut buffer = heapless::Vec::<u8, U32>::new();
        let serial = cx.resources.SERIAL_TX;
        let mut length = None;
        let mut reciving_msg = false;
        loop {
            while let Some(val) = cx.resources.CONS_SERIAL_READ.dequeue() {
                cx.resources.LED_NE.toggle().unwrap();

                // wait for start byte
                if (val == copter_com::START_BYTE) && !reciving_msg {
                    reciving_msg = true;
                    length = None;
                    buffer.clear();
                }

                // Add byte to buffer
                if reciving_msg {
                    buffer.push(val).ok();
                    let mut raw = [0; 32];
                    for (i, byte) in buffer.iter().enumerate() {
                        raw[i] = *byte;
                    }
                }

                // Is the length byte?
                if buffer.len() == 2 {
                    if val <= 30 {
                        length = Some(val);
                    } else {
                        reciving_msg = false;
                    }
                }

                // Check end of message
                if let Some(len) = length {
                    if (len as u16 + 2) == (buffer.len() as u16) {
                        let msg = copter_com::Message::parse(&buffer);
                        debug!("{:?}", msg);
                        if let Ok(msg) = msg {
                            match msg {
                                copter_com::Message::Ping(_) => {
                                    for byte in msg.serialize().iter() {
                                        nb::block!(serial.write(*byte)).ok();
                                    }
                                }
                            }
                            cx.resources.LED_E.toggle().unwrap()
                        };
                        reciving_msg = false;
                        length = None;
                    }
                }
            }
        }
    }

    // ====================================================
    // ==== Task for flight control ====
    // ====================================================

    /// Task to update the current estimation of the state
    #[task(priority = 5, resources = [SENSORS, STATE], spawn = [control_algorithm], schedule = [state_estimation])]
    fn state_estimation(cx: state_estimation::Context) {
        // Calculate new position estimation
        cx.resources
            .SENSORS
            .update(CONTROL_LOOP_PERIOD as f32 / CORE_FREQUENCY as f32);

        // Construct estimated state
        cx.resources.STATE.euler_angle = cx.resources.SENSORS.euler_angles();
        cx.resources.STATE.angle_vel = cx.resources.SENSORS.angle_vel();

        // Spawn control loop
        cx.spawn.control_algorithm().unwrap();

        // Schedule next Update
        cx.schedule
            .state_estimation(
                cx.scheduled + rtic::cyccnt::Duration::from_cycles(CONTROL_LOOP_PERIOD),
            )
            .unwrap();
    }

    /// Task to calculate the setpoints depending on the chosen control strategie
    #[task(priority = 5, resources = [MOTORS, STATE, SETPOINT, LED_S, CONS_APP_CMD])]
    fn control_algorithm(cx: control_algorithm::Context) {
        // Data for handling the current flight controler state
        static mut STATE: fc::ControlState = fc::ControlState::Disabled;
        static mut STATE_OLD: fc::ControlState = fc::ControlState::Disabled;
        static mut CYCLE_COUNT: u32 = 0;

        // Handle Commands from Main Application
        while let Some(cmd) = cx.resources.CONS_APP_CMD.dequeue() {
            match cmd {
                fc::AppCommand::DisableMotors => {
                    *STATE = fc::ControlState::Disabled;
                }
                fc::AppCommand::EnableMotors => {
                    if *STATE == fc::ControlState::Disabled {
                        *STATE = fc::ControlState::Arming;
                    }
                }
            }
        }

        // Detect state change
        let new = *STATE != *STATE_OLD;
        if new {
            debug!("New State: {:?}", *STATE);
            *CYCLE_COUNT = 0;
        }
        *STATE_OLD = *STATE;

        match *STATE {
            fc::ControlState::Disabled => {
                cx.resources.MOTORS.disable();
                cx.resources.LED_S.set_low().unwrap();
            }
            fc::ControlState::Arming => {
                cx.resources.MOTORS.enable();
                if (*CYCLE_COUNT % 10) == 0 {
                    cx.resources.LED_S.toggle().unwrap();
                }
                if *CYCLE_COUNT >= (MOTOR_ARMING_TIME / CONTROL_LOOP_PERIOD) {
                    *STATE = fc::ControlState::Running;
                }
            }
            fc::ControlState::Running => {
                cx.resources.LED_S.set_high().unwrap();
                // Calculate output depending on given setpoint
                let _motor_speed = match cx.resources.SETPOINT {
                    fc::SetValues::DirectControl(_motor_speed) => (),
                    fc::SetValues::YPRTControl(_yprt) => (),
                    fc::SetValues::AngleCtrl(_angles) => (),
                };

                cx.resources.MOTORS.set_speed(10.0, 20.0, 30.0, 40.0)
            }
        }

        *CYCLE_COUNT = CYCLE_COUNT.wrapping_add(1);
    }

    // ====================================================
    // ==== Interrupt handlers ====
    // ====================================================

    /// Interrupt for reciving bytes from serial and sending to idle task
    #[task(binds = USART2_EXTI26, priority = 8, resources = [SERIAL_RX, PROD_SERIAL_READ])]
    fn read_serial_byte_usart2(cx: read_serial_byte_usart2::Context) {
        match cx.resources.SERIAL_RX.read() {
            Ok(b) => {
                // Send Data to main task
                cx.resources.PROD_SERIAL_READ.enqueue(b).ok(); // Do not care if full
            }
            Err(hal::nb::Error::Other(e)) => {
                if let hal::serial::Error::Overrun = e {
                    debug!("Serial Overrun Error");
                } else {
                    // Ignore other Errors
                    debug!("Other Serial Error");
                }
            }
            Err(hal::nb::Error::WouldBlock) => {} // Ignore errors
        }
    }

    #[task(binds = USART1_EXTI25, priority = 8, resources = [SERIAL_RX, PROD_SERIAL_READ])]
    fn read_serial_byte_usart1(cx: read_serial_byte_usart1::Context) {
        match cx.resources.SERIAL_RX.read() {
            Ok(b) => {
                // Send Data to main task
                cx.resources.PROD_SERIAL_READ.enqueue(b).ok(); // Do not care if full
            }
            Err(hal::nb::Error::Other(e)) => {
                if let hal::serial::Error::Overrun = e {
                    debug!("Serial Overrun Error");
                } else {
                    // Ignore other Errors
                    debug!("Other Serial Error");
                }
            }
            Err(hal::nb::Error::WouldBlock) => {} // Ignore errors
        }
    }

    /// Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn EXTI0();
    }
};

#[no_mangle]
fn fminf(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

#[no_mangle]
fn fmaxf(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}
