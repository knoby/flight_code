use hal::prelude::*;

type PwmPinFL = hal::gpio::gpioc::PC6<hal::gpio::AF2>;
type PwmPinFR = hal::gpio::gpioc::PC7<hal::gpio::AF2>;
type PwmPinRL = hal::gpio::gpioc::PC8<hal::gpio::AF2>;
type PwmPinRR = hal::gpio::gpioc::PC9<hal::gpio::AF2>;
type PwmFL = hal::pwm::PwmChannel<hal::pwm::TIM3_CH1, hal::pwm::WithPins>;
type PwmFR = hal::pwm::PwmChannel<hal::pwm::TIM3_CH2, hal::pwm::WithPins>;
type PwmRL = hal::pwm::PwmChannel<hal::pwm::TIM3_CH3, hal::pwm::WithPins>;
type PwmRR = hal::pwm::PwmChannel<hal::pwm::TIM3_CH4, hal::pwm::WithPins>;
type Timer = hal::stm32::TIM3;

pub struct Motors {
    armed: bool,
    pwm_fl: PwmFL,
    pwm_fr: PwmFR,
    pwm_rl: PwmRL,
    pwm_rr: PwmRR,
    duty_stop: u16,
    duty_full: u16,
    rate_limit: f32,
    act_speed: (f32, f32, f32, f32),
}

impl Motors {
    /// Create New Struct controling the motors
    pub fn new(
        fl: PwmPinFL,
        fr: PwmPinFR,
        rl: PwmPinRL,
        rr: PwmPinRR,
        timer: Timer,
        clocks: hal::rcc::Clocks,
    ) -> Self {
        // Arm/Stop HIGH < 1060us
        // Full Power 1860us

        let (ch_fl, ch_fr, ch_rl, ch_rr) = hal::pwm::tim3(timer, 20_000, 50.hz(), &clocks);

        // Config PWM Pins
        let mut ch_fl = ch_fl.output_to_pc6(fl);
        let mut ch_fr = ch_fr.output_to_pc7(fr);
        let mut ch_rl = ch_rl.output_to_pc8(rl);
        let mut ch_rr = ch_rr.output_to_pc9(rr);

        // Calculate duty
        let duty_max = ch_fl.get_max_duty() as f32;
        let duty_stop = duty_max / 10_000.0 * 1060.0;
        let duty_full = duty_max / 10_000.0 * 1860.0;

        // Enable PWM
        ch_fl.enable();
        ch_fr.enable();
        ch_rl.enable();
        ch_rr.enable();

        let mut motors = Self {
            armed: false,
            pwm_fl: ch_fl,
            pwm_fr: ch_fr,
            pwm_rl: ch_rl,
            pwm_rr: ch_rr,
            duty_stop: duty_stop as u16,
            duty_full: duty_full as u16,
            rate_limit: 200.0,
            act_speed: (0.0, 0.0, 0.0, 0.0),
        };

        // Disable the Motors
        motors.disable();

        // Return the Motor Struct:
        motors
    }

    /// Set the rotaion speed of all three Motors.
    /// Allowed values are from 0.0 (Stop), 100.0 (Full Speed)
    /// Values out of this range will be limited to the range
    pub fn set_speed(&mut self, set_speed: (f32, f32, f32, f32), dt: f32) {
        if self.armed {
            let limit = self.rate_limit / dt;

            let (set_fl, set_fr, set_rl, set_rr) = set_speed;
            let (act_fl, act_fr, act_rl, act_rr) = self.act_speed;

            let new_fl = act_fl + (set_fl - act_fl).min(limit).max(-limit);
            let new_fr = act_fr + (set_fr - act_fr).min(limit).max(-limit);
            let new_rl = act_rl + (set_rl - act_rl).min(limit).max(-limit);
            let new_rr = act_rr + (set_rr - act_rr).min(limit).max(-limit);

            self.pwm_fl.set_duty(self.speed_to_duty(new_fl));
            self.pwm_fr.set_duty(self.speed_to_duty(new_fr));
            self.pwm_rl.set_duty(self.speed_to_duty(new_rl));
            self.pwm_rr.set_duty(self.speed_to_duty(new_rr));

            self.act_speed = (new_fl, new_fr, new_rl, new_rr);
        };
    }

    pub fn get_speed(&self) -> (f32, f32, f32, f32) {
        self.act_speed
    }

    /// Stop the Motors. Syntactic Sugar for motors.set_speed(0.0, 0.0, 0.0, 0.0);
    pub fn stop(&mut self) {
        if self.armed {
            self.set_speed((0.0, 0.0, 0.0, 0.0), 1000.0);
        }
    }

    /// Disable the Motor Stages with setting the duty to zero
    pub fn disable(&mut self) {
        self.armed = false;
        self.pwm_fl.set_duty(0);
        self.pwm_fr.set_duty(0);
        self.pwm_rl.set_duty(0);
        self.pwm_rr.set_duty(0);
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
}
