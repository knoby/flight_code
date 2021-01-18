use num_traits::Float;

mod pid;
use pid::*;

type MotorSpeed = (f32, f32, f32, f32);

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
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct Parameter {
    pub setpoint: SetValues,
    pub mass: f32,      // [kg]
    pub inertia_x: f32, // [kgm²]
    pub inertia_y: f32, // [kgm²]
    pub inertia_z: f32, // [kgm²]
    pub lever_x: f32,   // [m]
    pub lever_y: f32,   // [m]
    pub speed_min: f32, // [%]
    pub speed_max: f32, // [%]
    pub ctrl_par_pitch: CtrlParameter,
    pub ctrl_par_roll: CtrlParameter,
    pub ctrl_par_pitch_vel: CtrlParameter,
    pub ctrl_par_roll_vel: CtrlParameter,
    pub ctrl_par_yaw_vel: CtrlParameter,
    pub ctrl_par_thrust: CtrlParameter,
    pub thrust_fact: f32, // [%/N] - Scaling of the forced from the pid controls
    pub torque_fact: f32, // [%/N] - Scaling of the torque from the pid controls
}

impl Default for Parameter {
    #[allow(clippy::clippy::eq_op)] // For Parameter init with scaling ° to rad
    fn default() -> Self {
        let angle_vel_ctrl_par = CtrlParameter {
            k_p: 10.0, // [1/s]
            t_i: None,
            t_d: None,
            limit_pos: 1440.0 / 360.0 * 2.0 * core::f32::consts::PI, // [rad/s²]
            limit_neg: 1440.0 / 360.0 * 2.0 * core::f32::consts::PI, // [rad/s²]
        };
        let angle_pos_ctrl_par = CtrlParameter {
            k_p: 50.0, // [1/s]
            t_i: None,
            t_d: None,
            limit_pos: 360.0 / 360.0 * 2.0 * core::f32::consts::PI, // [rad/s²]
            limit_neg: 360.0 / 360.0 * 2.0 * core::f32::consts::PI, // [rad/s²]
        };
        let pid_off = CtrlParameter {
            k_p: 0.0,
            t_i: None,
            t_d: None,
            limit_pos: 0.0,
            limit_neg: 0.0,
        };
        Parameter {
            setpoint: SetValues::SequenceTest,
            mass: 0.7,
            inertia_x: 0.0186,
            inertia_y: 0.0186,
            inertia_z: 0.0186,
            lever_x: 0.2,
            lever_y: 0.2,
            speed_min: 0.0,
            speed_max: 100.0,
            ctrl_par_pitch: angle_pos_ctrl_par,
            ctrl_par_roll: pid_off,
            ctrl_par_roll_vel: pid_off,
            ctrl_par_pitch_vel: angle_vel_ctrl_par,
            ctrl_par_yaw_vel: pid_off,
            ctrl_par_thrust: pid_off,
            thrust_fact: 1.0,
            torque_fact: 1.0,
        }
    }
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

pub use copter_com::SetValues;

pub struct FlighController {
    roll_vel_ctrl: PIDController,
    pitch_vel_ctrl: PIDController,
    yaw_vel_ctrl: PIDController,

    roll_pos_ctrl: PIDController,
    pitch_pos_ctrl: PIDController,

    thrust_ctrl: PIDController,

    sequence_time: f32,
    sequence_motor: MotorSide,
}

impl Default for FlighController {
    /// Create a new Flight Controller and init it
    fn default() -> FlighController {
        FlighController {
            roll_vel_ctrl: PIDController::new(1.0, None, None, 0.0, 100.0, 0.0),
            pitch_vel_ctrl: PIDController::new(1.0, None, None, 0.0, 0.0, 0.0),
            yaw_vel_ctrl: PIDController::new(1.0, None, None, 0.0, 100.0, 0.0),

            roll_pos_ctrl: PIDController::new(1.0, None, None, 0.0, 100.0, 0.0),
            pitch_pos_ctrl: PIDController::new(1.0, None, None, 0.0, 100.0, 0.0),

            thrust_ctrl: PIDController::new(1.0, None, None, 0.0, 100.0, 0.0),

            sequence_time: 0.0,
            sequence_motor: MotorSide::FrontLeft,
        }
    }
}

impl FlighController {
    pub fn new(par: &Parameter) -> Self {
        FlighController {
            pitch_vel_ctrl: PIDController::new_from_par(&par.ctrl_par_pitch),
            roll_vel_ctrl: PIDController::new_from_par(&par.ctrl_par_pitch),
            yaw_vel_ctrl: PIDController::new_from_par(&par.ctrl_par_pitch),

            pitch_pos_ctrl: PIDController::new_from_par(&par.ctrl_par_pitch),
            roll_pos_ctrl: PIDController::new_from_par(&par.ctrl_par_pitch),

            thrust_ctrl: PIDController::new_from_par(&par.ctrl_par_thrust),

            sequence_motor: MotorSide::FrontLeft,
            sequence_time: 0.0,
        }
    }

    /// Direct control of the motor speed
    pub fn direct_ctrl(&mut self, motor_speed: (f32, f32, f32, f32)) -> MotorSpeed {
        motor_speed
    }

    /// Direct control of the yaw pitch roll and thrust force
    pub fn yprt_ctrl(&mut self, pryt: (f32, f32, f32, f32)) -> MotorSpeed {
        Self::motor_mixer(pryt)
    }

    /// Stablalize with angle = 0 ° and angle_vel = °/s
    pub fn stabalize(
        &mut self,
        act_angle: &[f32; 3],
        act_vel: &[f32; 3],
        vertical_acc: f32,
        dt: f32,
        par: &Parameter,
    ) -> MotorSpeed {
        // Set Value is zero for stabalize
        let r_angle_set = 0.0;
        let p_angle_set = 0.0;

        // Calculate Angle Controler:
        // - Input Angle [rad]
        // - Output AngleVelocity [rad/s]

        let r_vel_set = self
            .roll_pos_ctrl
            .calc_next_output(Some(r_angle_set), act_angle[0], dt);
        let p_vel_set = self
            .pitch_pos_ctrl
            .calc_next_output(Some(p_angle_set), act_angle[1], dt);
        let y_vel_set = 0.0; // Setvalue for the yaw is zero

        // Calcualte the angle vel controler
        let (r_acc_set, p_acc_set, y_acc_set) = self.angle_vel_control_loop(
            (r_vel_set, p_vel_set, y_vel_set),
            (act_vel[0], act_vel[1], act_vel[2]),
            dt,
        );

        // Thrustcontroler for stabalize is controling the vertical acceleration
        // - Input m/s²
        // - Ouptut F
        let thrust = self.thrust_ctrl.calc_next_output(None, vertical_acc, dt);

        // Calcualte motorspeed
        self.calc_motor_speed((r_acc_set, p_acc_set, y_acc_set), thrust, par)
    }

    /// Resets the PID Controler
    pub fn reset(&mut self) {
        self.roll_pos_ctrl.reset();
        self.pitch_pos_ctrl.reset();
        self.roll_vel_ctrl.reset();
        self.pitch_vel_ctrl.reset();
        self.yaw_vel_ctrl.reset();
        self.thrust_ctrl.reset();
    }

    /// Mix the force signsals from yaw pitch roll to the signals for the four motors
    fn motor_mixer(rpyt: (f32, f32, f32, f32)) -> (f32, f32, f32, f32) {
        let (r, p, y, t) = rpyt;

        let fl = r + p - y + t;
        let fr = -r + p + y + t;
        let rl = r - p - y + t;
        let rr = -r - p + y + t;

        (fl, fr, rl, rr)
    }

    /// Sequence Test to check rotation of the rotors and correct assignment of the motors on the quadrocopter
    pub fn sequence_test(&mut self, dt: f32) -> MotorSpeed {
        self.sequence_time += dt;
        if self.sequence_time >= 2.0 {
            self.sequence_time = 0.0;
            self.sequence_motor = match self.sequence_motor {
                MotorSide::FrontLeft => MotorSide::FrontRight,
                MotorSide::FrontRight => MotorSide::RearLeft,
                MotorSide::RearLeft => MotorSide::RearRight,
                MotorSide::RearRight => MotorSide::FrontLeft,
            };
        }
        match self.sequence_motor {
            MotorSide::FrontLeft => (10.0, 0.0, 0.0, 0.0),
            MotorSide::FrontRight => (0.0, 10.0, 0.0, 0.0),
            MotorSide::RearLeft => (0.0, 0.0, 10.0, 0.0),
            MotorSide::RearRight => (0.0, 0.0, 0.0, 10.0),
        }
    }

    /// Function to test the PID Tuning of the angle Velocity controllers
    pub fn pid_angle_vel_test(&mut self, _dt: f32) -> MotorSpeed {
        unimplemented!()
    }

    /// Angle Velocity Control Loop
    /// Returns the angle acceleration for yaw pitch and roll rotation
    fn angle_vel_control_loop(
        &mut self,
        vel_set: (f32, f32, f32),
        vel_act: (f32, f32, f32),
        dt: f32,
    ) -> (f32, f32, f32) {
        // Deconstruct arguments
        let (r_vel_set, p_vel_set, y_vel_set) = vel_set;
        let (r_vel_act, p_vel_act, y_vel_act) = vel_act;

        // Calculate AngleVelocity Controler
        // - Input Anglevelocity [rad/s]
        // - Output AngleAcceleration [rad/s²]
        let r_acc_set = self
            .roll_vel_ctrl
            .calc_next_output(Some(r_vel_set), r_vel_act, dt);
        let p_acc_set = self
            .pitch_vel_ctrl
            .calc_next_output(Some(p_vel_set), p_vel_act, dt);
        let y_acc_set = self
            .yaw_vel_ctrl
            .calc_next_output(Some(y_vel_set), y_vel_act, dt);

        // Return accelerations
        (r_acc_set, p_acc_set, y_acc_set)
    }

    /// Calculate motorspeed from angle velocity setpoint and thrust setpoint
    fn calc_motor_speed(
        &self,
        acc_set: (f32, f32, f32),
        thrust: f32,
        par: &Parameter,
    ) -> MotorSpeed {
        // Deconstruct tuple
        let (r_acc_set, p_acc_set, y_acc_set) = acc_set;

        // Force from the three controllers is calculated
        let f_roll = r_acc_set * par.inertia_x / par.lever_y * par.thrust_fact; // [N]
        let f_pitch = p_acc_set * par.inertia_y / par.lever_x * par.thrust_fact; // [N]
        let f_yaw = y_acc_set * par.inertia_z * par.torque_fact * par.torque_fact; // [Nm]
        let f_thrust = thrust * par.mass * par.thrust_fact; // [N]
        let force = Self::motor_mixer((f_roll, f_pitch, f_yaw, f_thrust));
        let mut force = [force.0, force.1, force.2, force.3];

        // The force rises with the power of 2 of the speed.
        // Prevent sqrt with values smaler than zero
        // Limit the speed output to max and min speed
        for f in force.iter_mut() {
            *f = f.max(0.0).sqrt().min(par.speed_max).max(par.speed_min);
        }
        (force[0], force[1], force[2], force[3])
    }
}

enum MotorSide {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}
