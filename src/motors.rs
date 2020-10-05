use hal::prelude::*;

type PwmPinVL = hal::gpio::gpioc::PC6<hal::gpio::AF2>;
type PwmPinVR = hal::gpio::gpioc::PC7<hal::gpio::AF2>;
type PwmPinHL = hal::gpio::gpioc::PC8<hal::gpio::AF2>;
type PwmPinHR = hal::gpio::gpioc::PC9<hal::gpio::AF2>;
type PwmVL = hal::pwm::PwmChannel<hal::pwm::TIM3_CH1, hal::pwm::WithPins>;
type PwmVR = hal::pwm::PwmChannel<hal::pwm::TIM3_CH2, hal::pwm::WithPins>;
type PwmHL = hal::pwm::PwmChannel<hal::pwm::TIM3_CH3, hal::pwm::WithPins>;
type PwmHR = hal::pwm::PwmChannel<hal::pwm::TIM3_CH4, hal::pwm::WithPins>;
type Timer = hal::stm32::TIM3;

pub struct Motors {
    armed: bool,
    pwm_vl: PwmVL,
    pwm_vr: PwmVR,
    pwm_hl: PwmHL,
    pwm_hr: PwmHR,
    duty_stop: u16,
    duty_full: u16,
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

        let (ch_vl, ch_vr, ch_hl, ch_hr) = hal::pwm::tim3(timer, 20_000, 50.hz(), &clocks);

        // Config PWM Pins
        let mut ch_vl = ch_vl.output_to_pc6(vl);
        let mut ch_vr = ch_vr.output_to_pc7(vr);
        let mut ch_hl = ch_hl.output_to_pc8(hl);
        let mut ch_hr = ch_hr.output_to_pc9(hr);

        // Calculate duty
        let duty_max = ch_vl.get_max_duty() as f32;
        let duty_stop = duty_max / 10_000.0 * 1060.0;
        let duty_full = duty_max / 10_000.0 * 1860.0;

        // Enable PWM
        ch_vl.enable();
        ch_vr.enable();
        ch_hl.enable();
        ch_hr.enable();

        let mut motors = Self {
            armed: false,
            pwm_vl: ch_vl,
            pwm_vr: ch_vr,
            pwm_hl: ch_hl,
            pwm_hr: ch_hr,
            duty_stop: duty_stop as u16,
            duty_full: duty_full as u16,
        };

        // Disable the Motors
        motors.disable();

        // Return the Motor Struct:
        motors
    }

    /// Set the rotaion speed of all three Motors.
    /// Allowed values are from 0.0 (Stop), 100.0 (Full Speed)
    /// Values out of this range will be limited to the range
    pub fn set_speed(&mut self, vl: f32, vr: f32, hl: f32, hr: f32) {
        if self.armed {
            self.pwm_vl.set_duty(self.speed_to_duty(vl));
            self.pwm_vr.set_duty(self.speed_to_duty(vr));
            self.pwm_hl.set_duty(self.speed_to_duty(hl));
            self.pwm_hr.set_duty(self.speed_to_duty(hr));
        };
    }

    /// Stop the Motors. Syntactic Sugar for motors.set_speed(0.0, 0.0, 0.0, 0.0);
    pub fn stop(&mut self) {
        if self.armed {
            self.set_speed(0.0, 0.0, 0.0, 0.0);
        }
    }

    /// Disable the Motor Stages with setting the duty to zero
    pub fn disable(&mut self) {
        self.armed = false;
        self.pwm_vl.set_duty(0);
        self.pwm_vr.set_duty(0);
        self.pwm_hl.set_duty(0);
        self.pwm_hr.set_duty(0);
    }

    pub fn enable(&mut self) {
        self.armed = true;
        self.stop();
    }

    /// Calculate duty from set speed
    fn speed_to_duty(&self, speed: f32) -> u16 {
        if speed >= 100.0 {
            self.duty_full
        } else if speed <= 0.0 {
            self.duty_stop
        } else {
            let var_duty = (self.duty_full - self.duty_stop) as f32 * speed / 100.0;
            self.duty_stop + var_duty as u16
        }
    }

    /// Returns the Period of the underlying timer in seconds
    pub fn period(&self) -> f32 {
        0.01
    }

    /// Reset ISR Flag (UIF)
    pub fn reset_isr_flag(&mut self) {
        unimplemented!()
    }
}
