use num_traits::float::FloatCore;
use num_traits::Float;

use core::f32::consts::PI;

/// Structs holds all information about current status
#[derive(Debug, Default, Clone, Copy, PartialEq, defmt::Format)]
pub struct Status {
    pub roll_angle: f32,
    pub pitch_angle: f32,
    pub yaw_angle: f32,
    pub roll_angle_vel: f32,
    pub pitch_angle_vel: f32,
    pub yaw_angle_vel: f32,
    pub roll_thrust: f32,
    pub pitch_thrust: f32,
    pub yaw_thrust: f32,
    pub thrust: f32,
    pub motor_speed: (f32, f32, f32, f32),
    pub vert_acc: f32,
    pub state: ControlState,
}

/// Structs holds all parameter for the flight controller
#[derive(Debug, Default, Clone, Copy, PartialEq, defmt::Format)]
pub struct Parameter {
    pub setpoint: SetValues,
}

/// Commands for the actual flight code from the application
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AppCommand {
    DisableMotors,
    EnableMotors,
    ChangeSetValue(SetValues),
}

/// States for the state machine in the high task
#[derive(Debug, Copy, Clone, PartialEq, defmt::Format)]
pub enum ControlState {
    /// Motors are disabled
    Disabled,
    /// Arming the Motors (takes approx 500ms)
    Arming,
    /// Running the motors with the given ctrl algorithm
    Running,
}

impl Default for ControlState {
    fn default() -> Self {
        ControlState::Disabled
    }
}

/*#[derive(Debug, Copy, Clone, PartialEq, defmt::Format)]
pub enum SetValues {
    /// Set the speed for the motors direct
    DirectControl((f32, f32, f32, f32)),
    /// Set the Forces/Torques for moving Yaw, Pitch, Roll and Thrust direct
    YPRTControl((f32, f32, f32, f32)),
    /// Only stabalize
    Stabalize,
    /// Closed loop control for Angles
    AngleCtrl([f32; 3]),
    /// Sequence Test
    SequenceTest,
}*/
pub use copter_com::SetValues;

pub struct FlighController {
    roll_vel_ctrl: PIDController<f32>,
    pitch_vel_ctrl: PIDController<f32>,
    yaw_vel_ctrl: PIDController<f32>,

    roll_pos_ctrl: PIDController<f32>,
    pitch_pos_ctrl: PIDController<f32>,

    thrust_ctrl: PIDController<f32>,

    time: f32,
    motor: MotorSide,
}

impl Default for FlighController {
    /// Create a new Flight Controller and init it
    fn default() -> FlighController {
        FlighController {
            roll_vel_ctrl: PIDController::new(
                1.0, // 360.0 / (2.0 * PI) * 0.5,
                None, None, -10.0, 10.0, 0.0,
            ),
            pitch_vel_ctrl: PIDController::new(
                1.0, //360.0 / (2.0 * PI) * 0.5,
                None, None, -10.0, 10.0, 0.0,
            ),
            yaw_vel_ctrl: PIDController::new(
                0.0, //360.0 / (2.0 * PI) * 0.5,
                None, None, -10.0, 10.0, 0.0,
            ),

            roll_pos_ctrl: PIDController::new(
                1.0,
                None,
                None,
                -20.0 / 360.0 * 2.0 * PI,
                20.0 / 360.0 * 2.0 * PI,
                0.0,
            ),
            pitch_pos_ctrl: PIDController::new(
                1.0,
                None,
                None,
                -20.0 / 360.0 * 2.0 * PI,
                20.0 / 360.0 * 2.0 * PI,
                0.0,
            ),

            thrust_ctrl: PIDController::new(100.0, None, None, -1000.0, 1000.0, 0.0),

            time: 0.0,
            motor: MotorSide::FrontLeft,
        }
    }
}

impl FlighController {
    pub fn direct_ctrl(&mut self, motor_speed: (f32, f32, f32, f32)) -> (f32, f32, f32, f32) {
        motor_speed
    }

    pub fn yprt_ctrl(&mut self, pryt: (f32, f32, f32, f32)) -> (f32, f32, f32, f32) {
        Self::motor_mixer(pryt)
    }

    pub fn stabalize(
        &mut self,
        act_angle: &[f32; 3],
        act_vel: &[f32; 3],
        vertical_acc: f32,
        dt: f32,
    ) -> (f32, f32, f32, f32) {
        let r_angle_set = 0.0;
        let p_angle_set = 0.0;

        self.roll_pos_ctrl.setpoint = r_angle_set;
        self.pitch_pos_ctrl.setpoint = p_angle_set;

        let r_vel_set = self.roll_pos_ctrl.calc_next_output(act_angle[0], dt);
        let p_vel_set = self.pitch_pos_ctrl.calc_next_output(act_angle[1], dt);
        let y_vel_set = 0.0;

        self.roll_vel_ctrl.setpoint = r_vel_set;
        self.pitch_vel_ctrl.setpoint = p_vel_set;
        self.yaw_vel_ctrl.setpoint = y_vel_set;

        let r_set = self.roll_vel_ctrl.calc_next_output(act_vel[0], dt);
        let p_set = self.pitch_vel_ctrl.calc_next_output(act_vel[1], dt);
        let y_set = self.yaw_vel_ctrl.calc_next_output(act_vel[2], dt);

        self.thrust_ctrl.setpoint = 0.0;
        let thrust = self.thrust_ctrl.calc_next_output(vertical_acc, dt);

        let force = Self::motor_mixer((r_set, p_set, y_set, thrust));
        (
            force.0.max(0.0).sqrt(),
            force.1.max(0.0).sqrt(),
            force.2.max(0.0).sqrt(),
            force.3.max(0.0).sqrt(),
        )
    }

    pub fn reset(&mut self) {
        self.roll_pos_ctrl.reset();
        self.pitch_pos_ctrl.reset();
        self.roll_vel_ctrl.reset();
        self.pitch_vel_ctrl.reset();
        self.yaw_vel_ctrl.reset();
        self.thrust_ctrl.reset();
    }

    fn motor_mixer(rpyt: (f32, f32, f32, f32)) -> (f32, f32, f32, f32) {
        let (r, p, y, t) = rpyt;

        let fl = r + p - y + t;
        let fr = -r + p + y + t;
        let rl = r - p - y + t;
        let rr = -r - p + y + t;

        (fl, fr, rl, rr)
    }

    pub fn sequence_test(&mut self, dt: f32) -> (f32, f32, f32, f32) {
        self.time += dt;
        if self.time >= 2.0 {
            self.time = 0.0;
            self.motor = match self.motor {
                MotorSide::FrontLeft => MotorSide::FrontRight,
                MotorSide::FrontRight => MotorSide::RearLeft,
                MotorSide::RearLeft => MotorSide::RearRight,
                MotorSide::RearRight => MotorSide::FrontLeft,
            };
        }
        match self.motor {
            MotorSide::FrontLeft => (10.0, 0.0, 0.0, 0.0),
            MotorSide::FrontRight => (0.0, 10.0, 0.0, 0.0),
            MotorSide::RearLeft => (0.0, 0.0, 10.0, 0.0),
            MotorSide::RearRight => (0.0, 0.0, 0.0, 10.0),
        }
    }
}

enum MotorSide {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}

#[allow(non_snake_case)]
struct PIDController<T: FloatCore> {
    /// Saved Integral Value
    integral: T,
    /// Last Error for Calculating the differentail
    last_error: Option<T>,
    /// Setpoint for the PID Controll
    pub setpoint: T,
    /// Positive Output Limit
    pub limit_pos: T,
    /// Negative Output Limit
    pub limit_neg: T,
    /// Proportional Gain
    pub K_p: T,
    /// Integral Time. If None the Integral Value is disabled. Panics if Integral Time is Zero
    pub T_i: Option<T>,
    /// Derivative Time. If None the Derivative Value is disabled.
    pub T_d: Option<T>,
}

#[allow(non_snake_case)]
impl<T> PIDController<T>
where
    T: FloatCore,
{
    /// Creates a new PIDController
    pub fn new(
        K_p: T,
        T_i: Option<T>,
        T_d: Option<T>,
        limit_neg: T,
        limit_pos: T,
        setpoint: T,
    ) -> Self {
        PIDController {
            integral: T::zero(),
            last_error: None,
            setpoint,
            limit_pos,
            limit_neg,
            K_p,
            T_i,
            T_d,
        }
    }

    /// Reset the Integral Value and the saved value for calculating the Derivative value.
    #[allow(unused)]
    pub fn reset(&mut self) {
        self.integral = T::zero();
        self.last_error = None;
    }

    /// Output the next Controll Output
    pub fn calc_next_output(&mut self, measurement: T, dt: T) -> T {
        // Control Error
        let error = self.setpoint - measurement;

        // Proportional Error
        // =====
        let p_output = error * self.K_p;

        // Integral Error
        // =====
        let mut new_integral = T::zero();
        let mut i_output = T::zero();

        // Check if Integral is to be calculated
        if let Some(T_i) = self.T_i {
            new_integral = self.integral + dt * error;
            i_output = new_integral * self.K_p / T_i;
        }

        // Differential Error
        // =====
        let mut d_output = T::zero();
        if let Some(T_d) = self.T_d {
            if let Some(last_error) = self.last_error {
                d_output = self.K_p * T_d / dt * (error - last_error);
            }
        }

        let output_unlimit = p_output + i_output + d_output;

        let mut limit_active = false;
        // Check Limits
        let output = if output_unlimit > self.limit_pos {
            limit_active = true;
            self.limit_pos
        } else if output_unlimit < self.limit_neg {
            limit_active = true;
            self.limit_neg
        } else {
            output_unlimit
        };

        // If Limit is not active safe the integral Value.
        if !limit_active & self.T_i.is_some() {
            self.integral = new_integral;
        }

        output
    }
}
