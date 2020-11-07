use num_traits::float::FloatCore;

use core::f32::consts::PI;

pub type Vector = nalgebra::Vector3<f32>;

/// Commands for the actual flight code from the application
#[derive(Debug, Clone, Copy, PartialOrd, PartialEq)]
pub enum AppCommand {
    DisableMotors,
    EnableMotors,
}

/// States for the state machine in the high task
#[derive(Copy, Clone, PartialEq)]
pub enum ControlState {
    /// Motors are disabled
    Disabled,
    /// Arming the Motors (takes approx 500ms)
    Arming,
    /// Running the motors with the given ctrl algorithm
    Running,
}

#[derive(Copy, Clone, PartialEq)]
pub enum SetValues {
    /// Set the speed for the motors direct
    DirectControl([f32; 4]),
    /// Set the Forces/Torques for moving Yaw, Pitch, Roll and Thrust direct
    YPRTControl([f32; 4]),
    /// Closed loop control for Angles
    AngleCtrl([f32; 3]),
}
#[derive(Debug, Default, Clone, Copy, PartialEq)]
/// The current estimated state
pub struct State {
    pub euler_angle: (f32, f32, f32),
    pub angle_vel: Vector,
    pub position: Vector,
    pub velocity: Vector,
}

pub struct FlighController {
    roll_vel_ctrl: PIDController<f32>,
    pitch_vel_ctrl: PIDController<f32>,
    yaw_vel_ctrl: PIDController<f32>,

    roll_pos_ctrl: PIDController<f32>,
    pitch_pos_ctrl: PIDController<f32>,
}

impl Default for FlighController {
    /// Create a new Flight Controller and init it
    fn default() -> FlighController {
        FlighController {
            roll_vel_ctrl: PIDController::new(
                360.0 / (2.0 * PI) * 0.5,
                None,
                None,
                -10.0,
                10.0,
                0.0,
            ),
            pitch_vel_ctrl: PIDController::new(
                360.0 / (2.0 * PI) * 0.5,
                None,
                None,
                -10.0,
                10.0,
                0.0,
            ),
            yaw_vel_ctrl: PIDController::new(1.0, None, None, -10.0, 10.0, 0.0),

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
        }
    }
}

impl FlighController {
    /// Calculate the FlightController
    pub fn update(
        &mut self,
        angle_vel: (f32, f32, f32),
        angle_pos: (f32, f32, f32),
        dt: f32,
    ) -> (f32, f32, f32, f32) {
        (0.0, 0.0, 0.0, 0.0)
    }
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
        let error = self.setpoint - measurement;

        let p_output = error * self.K_p;

        let mut new_integral = T::zero();
        let mut i_output = T::zero();
        // Check if Integral is to be calculated
        if let Some(T_i) = self.T_i {
            new_integral = self.integral + dt * error;
            i_output = new_integral * self.K_p / T_i;
        }

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
