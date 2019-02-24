use stm32f1xx_hal::pwm;

type MotorF = pwm::Pwm<stm32f1::stm32f103::TIM4, pwm::C1>;
type MotorL = pwm::Pwm<stm32f1::stm32f103::TIM4, pwm::C2>;
type MotorR = pwm::Pwm<stm32f1::stm32f103::TIM4, pwm::C3>;
type MotorH = pwm::Pwm<stm32f1::stm32f103::TIM4, pwm::C4>;

pub struct Motors2x2 {
    pub motor_pwm: (MotorF, MotorL, MotorR, MotorH),
}
