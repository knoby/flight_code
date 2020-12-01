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
use heapless::consts::{U32, U4};
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
const CONTROL_LOOP_DT: f32 = 0.01;
const MOTOR_ARMING_TIME: u32 = CORE_FREQUENCY;

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
        SERIAL_TX: Option<serial::SerialTx>,
        DMA_SERIAL_TX: Option<serial::SerialTxDMA>,
        DMA_BUFFER_TX: Option<&'static mut [u8; 32]>,

        // Sensor Data
        SENSORS: position::Sensors,

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

        // Parameter and Status
        PARAMETER: fc::Parameter,
        STATUS: fc::Status,

        // Flight ctrl things
        FLIGHT_CTRL: fc::FlighController,
    }

    #[init(spawn=[state_estimation, cyclic_status])]
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
        #[cfg(not(feature = "serial_usb"))]
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
                hal::time::Bps(38400),
                clocks,
                &mut rcc.apb2,
            )
        };

        let (tx, rx) = serial::create_tx_rx(serial);

        #[cfg(feature = "serial_usb")]
        let dma_serial_tx = cx.device.DMA1.split(&mut rcc.ahb).ch4;
        #[cfg(not(feature = "serial_usb"))]
        let dma_serial_tx = cx.device.DMA1.split(&mut rcc.ahb).ch7;

        let dma_buffer_tx = cortex_m::singleton!(: [u8;32] = [0; 32]).unwrap();

        // When finished start the periodic tast
        cx.spawn.state_estimation().unwrap();
        cx.spawn.cyclic_status().unwrap();

        //Assign late resources
        init::LateResources {
            PROD_SERIAL_READ: ps,
            CONS_SERIAL_READ: cs,

            PROD_APP_CMD: pa,
            CONS_APP_CMD: ca,

            SERIAL_RX: rx,
            SERIAL_TX: Some(tx),

            DMA_SERIAL_TX: Some(dma_serial_tx),

            DMA_BUFFER_TX: Some(dma_buffer_tx),

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
            LED_SE: led_se,
            LED_S: led_s,
            LED_SW: led_sw,
            LED_W: led_w,
            LED_NW: led_nw,

            PARAMETER: fc::Parameter::default(),
            STATUS: fc::Status::default(),

            FLIGHT_CTRL: fc::FlighController::default(),
        }
    }

    // ====================================================
    // ==== Background and communication tasks ====
    // ====================================================

    /// Idle Task for non critical jobs
    #[idle(resources = [CONS_SERIAL_READ, LED_NE, LED_E, STATUS, PROD_APP_CMD], spawn = [serial_send] )]
    fn idle(cx: idle::Context) -> ! {
        debug!("Entering Idle Task");
        let mut buffer = heapless::Vec::<u8, U32>::new();
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
                                    cx.spawn.serial_send(msg).unwrap();
                                }
                                copter_com::Message::Attitude(_) => (),
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
    // ==== Task for sending data over serial ====
    // ====================================================
    #[task(priority = 2, resources = [STATUS], spawn = [serial_send], schedule = [cyclic_status])]
    fn cyclic_status(mut cx: cyclic_status::Context) {
        // Counter to track number of sendings
        static mut TIMESTAMP: u32 = 0;

        *TIMESTAMP += 1;

        let mut status = fc::Status::default();
        cx.resources.STATUS.lock(|val| {
            status = *val;
        });
        let attitude = copter_com::Message::Attitude(copter_com::Attitude {
            timestamp: *TIMESTAMP,
            roll: status.roll_angle,
            pitch: status.pitch_angle,
            yaw: status.yaw_angle,
            roll_speed: status.roll_angle_vel,
            pitch_speed: status.pitch_angle_vel,
            yaw_speed: status.yaw_angle_vel,
        });
        cx.schedule
            .cyclic_status(cx.scheduled + rtic::cyccnt::Duration::from_cycles(CORE_FREQUENCY / 10))
            .unwrap();
        cx.spawn.serial_send(attitude).unwrap();
    }

    #[task(priority = 2, resources = [SERIAL_TX, DMA_SERIAL_TX, DMA_BUFFER_TX], capacity = 4)]
    fn serial_send(cx: serial_send::Context, msg: copter_com::Message) {
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

    // ====================================================
    // ==== Task for flight control ====
    // ====================================================

    /// Task to update the current estimation of the state
    #[task(priority = 5, resources = [SENSORS,  STATUS], spawn = [control_algorithm], schedule = [state_estimation])]
    fn state_estimation(cx: state_estimation::Context) {
        // Calculate new position estimation
        cx.resources
            .SENSORS
            .update(CONTROL_LOOP_PERIOD as f32 / CORE_FREQUENCY as f32);

        // Construct estimated state
        let euler_angle = cx.resources.SENSORS.euler_angles();
        let angle_vel = cx.resources.SENSORS.angle_vel();
        let vert_acc = cx.resources.SENSORS.vert_vel();

        // Set Status
        cx.resources.STATUS.roll_angle = euler_angle.0;
        cx.resources.STATUS.pitch_angle = euler_angle.1;
        cx.resources.STATUS.yaw_angle = euler_angle.2;
        cx.resources.STATUS.roll_angle_vel = angle_vel[0];
        cx.resources.STATUS.pitch_angle_vel = angle_vel[1];
        cx.resources.STATUS.yaw_angle_vel = angle_vel[2];
        cx.resources.STATUS.vert_acc = vert_acc;

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
    #[task(priority = 5, resources = [MOTORS, LED_S, CONS_APP_CMD, STATUS, PARAMETER, FLIGHT_CTRL])]
    fn control_algorithm(cx: control_algorithm::Context) {
        // Data for handling the current flight controler state
        static mut STATE: fc::ControlState = fc::ControlState::Disabled;
        static mut STATE_OLD: fc::ControlState = fc::ControlState::Disabled;
        static mut CYCLE_COUNT: u32 = 0;

        // De-Structing Resources
        let MOTORS = cx.resources.MOTORS;
        let LED = cx.resources.LED_S;
        let FC = cx.resources.FLIGHT_CTRL;
        let STATUS = cx.resources.STATUS;

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
                MOTORS.disable();
                LED.set_low().unwrap();
                *STATE = fc::ControlState::Arming;
            }
            fc::ControlState::Arming => {
                MOTORS.enable();
                if (*CYCLE_COUNT % 10) == 0 {
                    LED.toggle().unwrap();
                }
                if *CYCLE_COUNT >= (MOTOR_ARMING_TIME / CONTROL_LOOP_PERIOD) {
                    *STATE = fc::ControlState::Running;
                }
            }
            fc::ControlState::Running => {
                LED.set_high().unwrap();
                // Calculate output depending on given setpoint
                let act_angle = &[STATUS.roll_angle, STATUS.pitch_angle, STATUS.yaw_angle];
                let act_angle_vel = &[
                    STATUS.roll_angle_vel,
                    STATUS.pitch_angle_vel,
                    STATUS.yaw_angle_vel,
                ];
                let motor_speed = match cx.resources.PARAMETER.setpoint {
                    fc::SetValues::DirectControl(motor_speed) => FC.direct_ctrl(motor_speed),

                    fc::SetValues::YPRTControl(yprt) => FC.yprt_ctrl(yprt),
                    fc::SetValues::Stabalize => {
                        FC.stabalize(act_angle, act_angle_vel, STATUS.vert_acc, CONTROL_LOOP_DT)
                    }
                    fc::SetValues::SequenceTest => FC.sequence_test(CONTROL_LOOP_DT),
                    fc::SetValues::AngleCtrl(_angles) => unimplemented!(),
                };

                MOTORS.set_speed(motor_speed, CONTROL_LOOP_DT);
            }
        }

        // Set Status
        STATUS.motor_speed = MOTORS.get_speed();

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
        fn DMA1_CH1();
        fn DMA1_CH2();
        fn DMA1_CH3();
        fn DMA1_CH4();
        fn DMA1_CH5();
        fn DMA1_CH6();
        fn DMA1_CH7();
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
