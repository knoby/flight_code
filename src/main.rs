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

use rtt_target::rprintln;
mod fc;
mod ipc;
mod led;
mod motors;
mod position;
mod serial;

// Runtime Imports
use rtic::app;

// Program Constants

#[app(device = hal::device, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // Que for passing data from Serial Interrupt to Idle Task
        PROD_SERIAL_READ: Producer<'static, u8, U32>,
        CONS_SERIAL_READ: Consumer<'static, u8, U32>,

        // Que from idel to motion task to send commands to the control task
        PROD_IDLE_TO_MOTION: Producer<'static, ipc::IPC, U4>,
        CONS_IDLE_TO_MOTION: Consumer<'static, ipc::IPC, U4>,

        // Communication to Base Station
        SERIAL_RX: serial::SerialRx,
        SERIAL_TX: serial::SerialTx,

        // Sensor Data
        SENSORS: position::Sensors,

        // Motor Control
        MOTORS: motors::Motors,

        // Motion State
        MOTOR_STATE: copter_defs::MotorState,
        ORIENTATION: [f32; 3],

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
        static mut Q_SERIAL_READ: Option<Queue<u8, U32>> = None;
        static mut Q_IDLE_TO_MOTION: Option<Queue<ipc::IPC, U4>> = None;

        // Init RTT Communictaion
        rtt_target::rtt_init_print!();
        rprintln!("Rustocupter Wakeup after reset.");
        rprintln!("Start Init...");

        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        //Freeze clock frequencies
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        rprintln!("Setting Up Clocks");
        let clocks = rcc
            .cfgr
            .sysclk(64.mhz())
            .pclk1(32.mhz())
            .pclk2(64.mhz())
            .freeze(&mut flash.acr);

        rprintln!("Config of on bord Leds");
        // Setup the on board LEDs
        let gpioe = cx.device.GPIOE.split(&mut rcc.ahb);
        let led_n = gpioe.pe9.output().output_speed(hal::gpio::MediumSpeed);
        let led_ne = gpioe.pe10.output().output_speed(hal::gpio::MediumSpeed);
        let led_e = gpioe.pe11.output().output_speed(hal::gpio::MediumSpeed);

        rprintln!("Setup I2C Bus for acceleration sensor");
        // Setup the I2C Bus for the Magneto and Accelero Meter
        let gpiob = cx.device.GPIOB.split(&mut rcc.ahb);
        let scl = gpiob.pb6.alternating(hal::gpio::AF4);
        let sda = gpiob.pb7.alternating(hal::gpio::AF4);
        let i2c = cx.device.I2C1.i2c((scl, sda), 400.khz(), clocks);

        let acc_sensor = lsm303dlhc::Lsm303dlhc::new(i2c).unwrap();

        rprintln!("Setup SPI Bus for gyro sensor");
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

        rprintln!("Configuration of PWM Output pins");
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

        rprintln!("Setup Communication Channel for Serial data to idle task");
        //Create que for serial read
        *Q_SERIAL_READ = Some(Queue::new());
        let (ps, cs) = Q_SERIAL_READ.as_mut().unwrap().split();

        rprintln!("Setup Commuincation Channel for Idle to Motion Task");
        *Q_IDLE_TO_MOTION = Some(Queue::new());
        let (pitm, citm) = Q_IDLE_TO_MOTION.as_mut().unwrap().split();

        rprintln!("Configuration of serial interface");
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
            .serial((tx_pin, rx_pin), hal::time::Bps(19200), clocks);
        let (tx, rx) = serial::create_tx_rx(serial);

        //Assign late resources
        init::LateResources {
            PROD_SERIAL_READ: ps,
            CONS_SERIAL_READ: cs,

            PROD_IDLE_TO_MOTION: pitm,
            CONS_IDLE_TO_MOTION: citm,

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

            MOTOR_STATE: copter_defs::MotorState::default(),
            ORIENTATION: [0.0_f32; 3],

            LED_N: led_n,
            LED_NE: led_ne,
            LED_E: led_e,
            FC: fc::FlighController::default(),
        }
    }

    /// Idle Task for non critical jobs
    #[idle(resources = [CONS_SERIAL_READ, LED_N, LED_NE, LED_E, SERIAL_TX, PROD_IDLE_TO_MOTION, ORIENTATION, MOTOR_STATE])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut buffer = heapless::Vec::<u8, U64>::new();
        loop {
            // Toggle LED for Idle Mode is running
            cx.resources.LED_NE.toggle().unwrap();
            if let Some(val) = cx.resources.CONS_SERIAL_READ.dequeue() {
                // Read from que
                if serial::check_frame_end(val) {
                    // Check if byte is end from frame
                    if let Ok(cmd) = copter_defs::Command::from_slip(&buffer) {
                        // Toggle LED for Command Recived
                        cx.resources.LED_E.toggle().unwrap();
                        // Try to decode
                        use copter_defs::Command::*;
                        match cmd {
                            // Execute Command
                            ToggleLed => cx.resources.LED_N.toggle().unwrap(),
                            StartMotor => {
                                cx.resources
                                    .PROD_IDLE_TO_MOTION
                                    .enqueue(ipc::IPC::EnableMotors)
                                    .ok();
                            }
                            StopMotor => {
                                cx.resources
                                    .PROD_IDLE_TO_MOTION
                                    .enqueue(ipc::IPC::DisableMotors)
                                    .ok();
                            }
                            GetMotionState => {
                                let mut orientation = [0.0_f32; 3];
                                let mut motor_speed = [0.0_f32; 4];
                                let mut armed = false;
                                cx.resources
                                    .ORIENTATION
                                    .lock(|motion_value| orientation = motion_value.clone());
                                cx.resources.MOTOR_STATE.lock(|motor_state| {
                                    armed = motor_state.armed;
                                    motor_speed[0] = motor_state.front_left;
                                    motor_speed[1] = motor_state.front_right;
                                    motor_speed[2] = motor_state.rear_left;
                                    motor_speed[3] = motor_state.rear_right;
                                });
                                let test_state = SendMotionState(orientation, motor_speed, armed);
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
                            SetTargetMotorSpeed(set_motor_speed) => {
                                cx.resources
                                    .PROD_IDLE_TO_MOTION
                                    .enqueue(ipc::IPC::SetCtrlMode(ipc::CtrlMode::DirectCtrl(
                                        set_motor_speed,
                                    )))
                                    .ok();
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
    #[task(binds = TIM3, priority = 5 , resources = [FC, SENSORS, MOTORS, CONS_IDLE_TO_MOTION, ORIENTATION, MOTOR_STATE])]
    #[allow(deprecated)] // Replacementfunction is not implemented in nalgebra::RealField::abs ...
    fn periodic_task(cx: periodic_task::Context) {
        // Read new command
        if let Some(command) = cx.resources.CONS_IDLE_TO_MOTION.dequeue() {
            match command {
                ipc::IPC::EnableMotors => cx.resources.MOTOR_STATE.armed = true,
                ipc::IPC::DisableMotors => cx.resources.MOTOR_STATE.armed = false,
                ipc::IPC::SetCtrlMode(ctrl_mode) => match ctrl_mode {
                    ipc::CtrlMode::DirectCtrl(speed) => {
                        cx.resources.MOTOR_STATE.front_left = speed[0].max(0.0).min(100.0);
                        cx.resources.MOTOR_STATE.front_right = speed[1].max(0.0).min(100.0);
                        cx.resources.MOTOR_STATE.rear_left = speed[2].max(0.0).min(100.0);
                        cx.resources.MOTOR_STATE.rear_right = speed[3].max(0.0).min(100.0);
                    }
                    _ => (),
                },
            }
        }

        // Update orientation
        cx.resources.SENSORS.update(cx.resources.MOTORS.period());
        let angle_pos = cx.resources.SENSORS.euler_angles();

        // Check Angle. If roll or pitch higher than 20° switch off
        use nalgebra::abs;
        let max_angle = 20.0 * (core::f32::consts::PI * 2.0) / 360.0;
        if (abs(&angle_pos.0) > max_angle) || (abs(&angle_pos.1) > max_angle) {
            cx.resources.MOTOR_STATE.armed = false;
        }

        if cx.resources.MOTOR_STATE.armed {
            cx.resources.MOTORS.enable();
            cx.resources.MOTORS.set_speed(
                cx.resources.MOTOR_STATE.front_left,
                cx.resources.MOTOR_STATE.front_right,
                cx.resources.MOTOR_STATE.rear_left,
                cx.resources.MOTOR_STATE.rear_right,
            );
        } else {
            cx.resources.MOTOR_STATE.front_right = 0.0;
            cx.resources.MOTOR_STATE.front_left = 0.0;
            cx.resources.MOTOR_STATE.rear_left = 0.0;
            cx.resources.MOTOR_STATE.rear_right = 0.0;
            cx.resources.MOTORS.disable();
        }

        // Save Temp Values to Resoruces
        *cx.resources.ORIENTATION = [angle_pos.0, angle_pos.1, angle_pos.2];

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
                    rprintln!("Serial Overrun Error");
                    cx.resources.SERIAL_RX.clear_overrun_error();
                } else {
                    // Ignore other Errors
                    rprintln!("Other Serial Error");
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
