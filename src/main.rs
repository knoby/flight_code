#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Define Panic behaivior
//extern crate panic_halt;
extern crate panic_semihosting;

// Used traits from the HAL crate
extern crate stm32f3xx_hal as hal;
use hal::nb::block;
use hal::prelude::*;

// Message Passing between Idle, Interrupt and Periodic
use heapless::consts::{U32, U4, U64};
use heapless::spsc::{Consumer, Producer, Queue};

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

#[app(device = hal::stm32 ,peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
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
    }

    #[init(spawn=[periodic_task])]
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
        let mut gpioe = cx.device.GPIOE.split(&mut rcc.ahb);
        let led_n = gpioe
            .pe9
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        let led_ne = gpioe
            .pe10
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        let led_e = gpioe
            .pe11
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

        rprintln!("Setup I2C Bus for acceleration sensor");
        // Setup the I2C Bus for the Magneto and Accelero Meter
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb);
        let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
        let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
        let i2c = hal::i2c::I2c::i2c1(cx.device.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1);

        let acc_sensor = lsm303dlhc::Lsm303dlhc::new(i2c).unwrap();

        rprintln!("Setup SPI Bus for gyro sensor");
        // Setup the Spi bus forthe Gyro
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb);
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

        rprintln!("Configuration of PWM Output pins");
        // Create Pins for PWM Output to control the ESC
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.ahb);
        let pwm_pin_motor_vl = gpioc.pc6.into_af2(&mut gpioc.moder, &mut gpioc.afrl);
        let pwm_pin_motor_vr = gpioc.pc7.into_af2(&mut gpioc.moder, &mut gpioc.afrl);
        let pwm_pin_motor_hl = gpioc.pc8.into_af2(&mut gpioc.moder, &mut gpioc.afrh);
        let pwm_pin_motor_hr = gpioc.pc9.into_af2(&mut gpioc.moder, &mut gpioc.afrh);

        rprintln!("Setup Communication Channel for Serial data to idle task");
        //Create que for serial read
        *Q_SERIAL_READ = Some(Queue::new());
        let (ps, cs) = Q_SERIAL_READ.as_mut().unwrap().split();

        rprintln!("Setup Commuincation Channel for Idle to Motion Task");
        *Q_IDLE_TO_MOTION = Some(Queue::new());
        let (pitm, citm) = Q_IDLE_TO_MOTION.as_mut().unwrap().split();

        rprintln!("Configuration of serial interface");
        // Create USART Port for communication to remote station
        let mut gpiod = cx.device.GPIOD.split(&mut rcc.ahb);
        let tx_pin = gpiod.pd5.into_af7(&mut gpiod.moder, &mut gpiod.afrl);
        let rx_pin = gpiod.pd6.into_af7(&mut gpiod.moder, &mut gpiod.afrl);
        let serial = hal::serial::Serial::usart2(
            cx.device.USART2,
            (tx_pin, rx_pin),
            hal::time::Bps(38400),
            clocks,
            &mut rcc.apb1,
        );
        let (tx, rx) = serial::create_tx_rx(serial);

        // When finished start the periodic tast
        cx.spawn.periodic_task().unwrap();

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
        }
    }

    /// Idle Task for non critical jobs
    #[idle(resources = [CONS_SERIAL_READ, LED_N, LED_NE, LED_E, SERIAL_TX, PROD_IDLE_TO_MOTION, ORIENTATION, MOTOR_STATE])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut buffer = heapless::Vec::<u8, U64>::new();
        loop {
            // Toggle LED for Idle Mode is running
            if let Some(val) = cx.resources.CONS_SERIAL_READ.dequeue() {
                cx.resources.LED_NE.toggle().unwrap();
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
                                    .lock(|&mut motion_value| orientation = motion_value);
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
                                    .enqueue(ipc::IPC::SetCtrlMode(
                                        copter_defs::CtrlMode::DirectCtrl(set_motor_speed),
                                    ))
                                    .ok();
                            }
                            SetTargetAngle(set_angle) => {
                                cx.resources
                                    .PROD_IDLE_TO_MOTION
                                    .enqueue(ipc::IPC::SetCtrlMode(
                                        copter_defs::CtrlMode::AngleCtrl(set_angle),
                                    ))
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
    #[task(schedule=[periodic_task], priority = 5 , resources = [ SENSORS, MOTORS, CONS_IDLE_TO_MOTION, ORIENTATION, MOTOR_STATE])]
    #[allow(deprecated)] // Replacementfunction is not implemented in nalgebra::RealField::abs ...
    fn periodic_task(cx: periodic_task::Context) {
        // Local Vars for this task
        static mut STATE: fc::ControlState = fc::ControlState::Disabled;
        static mut OLD_STATE: fc::ControlState = fc::ControlState::Disabled;
        static mut CYCLE_COUNT: u32 = 0;
        static mut CTRL_MODE: copter_defs::CtrlMode = copter_defs::CtrlMode::DirectCtrl([0.0; 4]);
        static mut FC: Option<fc::FlighController> = None;

        // Create FC in first run
        if FC.is_none() {
            *FC = Some(fc::FlighController::default());
        }

        // Destructing Resources for easy access
        let MOTOR_STATE = cx.resources.MOTOR_STATE;
        let MOTORS = cx.resources.MOTORS;

        // Read new command
        if let Some(command) = cx.resources.CONS_IDLE_TO_MOTION.dequeue() {
            match command {
                ipc::IPC::EnableMotors => MOTOR_STATE.armed = true,
                ipc::IPC::DisableMotors => MOTOR_STATE.armed = false,
                ipc::IPC::SetCtrlMode(ctrl_mode) => {
                    // Limit the set value
                    match ctrl_mode {
                        copter_defs::CtrlMode::DirectCtrl(mut speed) => {
                            for s in speed.iter_mut() {
                                *s = s.min(100.0).max(100.0);
                            }
                            *CTRL_MODE = copter_defs::CtrlMode::DirectCtrl(speed);
                        }
                        copter_defs::CtrlMode::AngleCtrl(mut angle) => {
                            for a in angle.iter_mut() {
                                *a = a.min(10.0).max(-10.0);
                            }
                            *CTRL_MODE = copter_defs::CtrlMode::AngleCtrl(angle);
                        }
                    }
                }
            }
        }

        // Update orientation
        cx.resources.SENSORS.update(MOTORS.period());
        let angle_pos = cx.resources.SENSORS.euler_angles();
        let angle_vel = cx.resources.SENSORS.angle_vel();

        // Check Angle. If roll or pitch higher than 20° switch off
        use nalgebra::abs;
        let max_angle = 20.0 * (core::f32::consts::PI * 2.0) / 360.0;
        if (abs(&angle_pos.0) > max_angle) || (abs(&angle_pos.1) > max_angle) {
            MOTOR_STATE.armed = false;
        }

        // Switch to disabled always from here
        if !MOTOR_STATE.armed {
            *STATE = fc::ControlState::Disabled;
        }

        // State machine handling
        if *STATE != *OLD_STATE {
            *CYCLE_COUNT = 0;
        };
        *OLD_STATE = *STATE;
        *CYCLE_COUNT = (*CYCLE_COUNT).wrapping_add(1);

        // State machien in this task
        match *STATE {
            // Just disable and wait for enabling of the motors
            fc::ControlState::Disabled => {
                // Disable Motors
                MOTORS.disable();
                // Reset Motor State
                MOTOR_STATE.front_left = 0.0;
                MOTOR_STATE.front_right = 0.0;
                MOTOR_STATE.rear_left = 0.0;
                MOTOR_STATE.rear_right = 0.0;
                if MOTOR_STATE.armed {
                    *STATE = fc::ControlState::Arming;
                    MOTORS.enable();
                };
            }
            // Arm the motors. We have to wait some time. For example 50 cycles ...
            fc::ControlState::Arming => {
                if *CYCLE_COUNT == 50 {
                    *STATE = fc::ControlState::ChooseCtrlMode;
                }
            }
            // Change Ctrl Mode
            fc::ControlState::ChooseCtrlMode => {
                rprintln!("Changed Ctrl Mode: {:?}", *CTRL_MODE);
                match *CTRL_MODE {
                    copter_defs::CtrlMode::DirectCtrl(_) => {
                        *STATE = fc::ControlState::DirectControl;
                    }
                    copter_defs::CtrlMode::AngleCtrl(_) => {
                        *STATE = fc::ControlState::AngleControl;
                    }
                };
            }
            // Direct Ctrl Mode. Motor Speed is set from serial interface direct
            fc::ControlState::DirectControl => {
                if let copter_defs::CtrlMode::DirectCtrl(set_speed) = *CTRL_MODE {
                    let delta_up = 1.0_f32;
                    let delta_down = 2.0_f32;
                    // Calculate new setpoint
                    MOTOR_STATE.front_left += (set_speed[0] - MOTOR_STATE.front_left)
                        .min(delta_up)
                        .max(-delta_down);
                    MOTOR_STATE.front_right += (set_speed[1] - MOTOR_STATE.front_right)
                        .min(delta_up)
                        .max(-delta_down);
                    MOTOR_STATE.rear_left += (set_speed[2] - MOTOR_STATE.rear_left)
                        .min(delta_up)
                        .max(-delta_down);
                    MOTOR_STATE.rear_right += (set_speed[3] - MOTOR_STATE.rear_right)
                        .min(delta_up)
                        .max(-delta_down);
                } else {
                    // Ctrl mode has changed --> Do the switching
                    *STATE = fc::ControlState::ChooseCtrlMode;
                };
            }
            // Angle control with flight controller
            fc::ControlState::AngleControl => {
                if let copter_defs::CtrlMode::AngleCtrl(_set_angle) = *CTRL_MODE {
                    if let Some(fc) = &mut *FC {
                        let (fl, fr, rl, rr) = fc.update(angle_vel, angle_pos, MOTORS.period());
                        MOTOR_STATE.front_left = fl + 50.0;
                        MOTOR_STATE.front_right = fr + 50.0;
                        MOTOR_STATE.rear_left = rl + 50.0;
                        MOTOR_STATE.rear_right = rr + 50.0;
                    }
                } else {
                    // Ctrl mode has changed --> Do the switching
                    *STATE = fc::ControlState::ChooseCtrlMode;
                };
            }
        }

        // Set Motor Speed
        MOTORS.set_speed(
            MOTOR_STATE.front_left,
            MOTOR_STATE.front_right,
            MOTOR_STATE.rear_left,
            MOTOR_STATE.rear_right,
        );

        // Save Temp Values to Resoruces
        *cx.resources.ORIENTATION = [angle_pos.0, angle_pos.1, angle_pos.2];

        // Spawn next run
        cx.schedule.periodic_task(cx.scheduled).unwrap();
    }

    /// Interrupt for reciving bytes from serial and sending to idle task
    #[task(binds = USART2_EXTI26, priority = 8, resources = [SERIAL_RX, PROD_SERIAL_READ])]
    fn read_serial_byte(cx: read_serial_byte::Context) {
        match cx.resources.SERIAL_RX.read() {
            Ok(b) => {
                // Send Data to main task
                cx.resources.PROD_SERIAL_READ.enqueue(b).ok(); // Do not care if full
            }
            Err(hal::nb::Error::Other(e)) => {
                if let hal::serial::Error::Overrun = e {
                    rprintln!("Serial Overrun Error");
                } else {
                    // Ignore other Errors
                    rprintln!("Other Serial Error");
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
