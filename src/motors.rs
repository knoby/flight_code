use hal::prelude::*;

use hal::gpio::{HighSpeed, PullNone, PushPull, AF2};

type PwmPinVL = hal::gpio::PC6<PullNone, hal::gpio::AltFn<AF2, PushPull, HighSpeed>>;
type PwmPinVR = hal::gpio::PC7<PullNone, hal::gpio::AltFn<AF2, PushPull, HighSpeed>>;
type PwmPinHL = hal::gpio::PC8<PullNone, hal::gpio::AltFn<AF2, PushPull, HighSpeed>>;
type PwmPinHR = hal::gpio::PC9<PullNone, hal::gpio::AltFn<AF2, PushPull, HighSpeed>>;
type PwmTimer = hal::timer::tim3::Timer<hal::timer::PwmTaken>;
type PwmVL =
    hal::pwm::PwmBinding<PwmPinVL, hal::timer::tim3::Channel<hal::timer::CH1, hal::timer::Pwm1>>;
type PwmVR =
    hal::pwm::PwmBinding<PwmPinVR, hal::timer::tim3::Channel<hal::timer::CH2, hal::timer::Pwm1>>;
type PwmHL =
    hal::pwm::PwmBinding<PwmPinHL, hal::timer::tim3::Channel<hal::timer::CH3, hal::timer::Pwm1>>;
type PwmHR =
    hal::pwm::PwmBinding<PwmPinHR, hal::timer::tim3::Channel<hal::timer::CH4, hal::timer::Pwm1>>;
type Timer = hal::device::TIM3;

pub struct Motors {
    pwm_vl: PwmVL,
    pwm_vr: PwmVR,
    pwm_hl: PwmHL,
    pwm_hr: PwmHR,
    duty_stop: u32,
    duty_full: u32,
    timer: PwmTimer,
}

impl Motors {
    /// Create New Struct controling the motors
    pub fn new(
        vl: PwmPinVL,
        vr: PwmPinVR,
        hl: PwmPinHL,
        hr: PwmPinHR,
        timer: Timer,
        clocks: hal::rcc::Clocks,
    ) -> Self {
        // Arm/Stop HIGH < 1060us
        // Full Power 1860us

        // Setup the timer
        let mut timer = alt_stm32f30x_hal::timer::tim3::Timer::new(timer, 100.hz(), clocks);

        // Enable Interrupt
        timer.listen(hal::timer::Event::TimeOut);

        let (channels, mut timer) = timer.use_pwm();

        // Config PWM Pins
        let mut vl = vl.to_pwm(channels.0, HighSpeed);
        let mut vr = vr.to_pwm(channels.1, HighSpeed);
        let mut hl = hl.to_pwm(channels.2, HighSpeed);
        let mut hr = hr.to_pwm(channels.3, HighSpeed);

        // Calculate duty
        let duty_max = vl.get_max_duty() as f32;
        let duty_stop = duty_max / 10_000.0 * 1060.0;
        let duty_full = duty_max / 10_000.0 * 1860.0;

        // Enable PWM
        vl.enable();
        vr.enable();
        hl.enable();
        hr.enable();

        timer.enable();

        let mut motors = Self {
            pwm_vl: vl,
            pwm_vr: vr,
            pwm_hl: hl,
            pwm_hr: hr,
            duty_stop: duty_stop as u32,
            duty_full: duty_full as u32,
            timer,
        };

        // Arm Motors
        motors.stop();
        cortex_m::asm::delay(32_000_000);

        // Spinn Motors for 1 second
        motors.set_speed(10.0, 10.0, 10.0, 10.0);
        cortex_m::asm::delay(32_000_000);

        // Stop Motors again
        motors.stop();

        // Return the Motor Struct:
        motors
    }

    /// Set the rotaion speed of all three Motors.
    /// Allowed values are from 0.0 (Stop), 100.0 (Full Speed)
    /// Values out of this range will be limited to the range
    pub fn set_speed(&mut self, vl: f32, vr: f32, hl: f32, hr: f32) {
        self.pwm_vl.set_duty(self.speed_to_duty(vl));
        self.pwm_vr.set_duty(self.speed_to_duty(vr));
        self.pwm_hl.set_duty(self.speed_to_duty(hl));
        self.pwm_hr.set_duty(self.speed_to_duty(hr));
    }

    /// Stop the Motors. Syntactic Sugar for motors.set_speed(0.0, 0.0, 0.0, 0.0);
    pub fn stop(&mut self) {
        self.set_speed(0.0, 0.0, 0.0, 0.0);
    }

    /// Calculate duty from set speed
    fn speed_to_duty(&self, speed: f32) -> u32 {
        if speed >= 100.0 {
            self.duty_full
        } else if speed <= 0.0 {
            self.duty_stop
        } else {
            let var_duty = (self.duty_full - self.duty_stop) as f32 * speed / 100.0;
            self.duty_stop + var_duty as u32
        }
    }

    /// Returns the Period of the underlying timer in seconds
    pub fn period(&self) -> f32 {
        0.01
    }

    /// Reset ISR Flag (UIF)
    pub fn reset_isr_flag(&mut self) {
        self.timer.reset_uif();
    }
}
