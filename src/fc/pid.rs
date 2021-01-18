#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
// Parameter to initialiase a pid controler
pub struct CtrlParameter {
    pub k_p: f32,
    pub t_i: Option<f32>,
    pub t_d: Option<f32>,
    pub limit_pos: f32,
    pub limit_neg: f32,
}

impl Default for CtrlParameter {
    fn default() -> Self {
        CtrlParameter {
            k_p: 1.0,
            t_i: None,
            t_d: None,
            limit_pos: 100.0,
            limit_neg: 0.0,
        }
    }
}

#[allow(non_snake_case)]
pub struct PIDController {
    /// Saved Integral Value
    integral: f32,
    /// Last Error for Calculating the differentail
    last_error: Option<f32>,
    /// Setpoint for the PID Controll
    pub setpoint: f32,
    /// Positive Output Limit
    pub limit_pos: f32,
    /// Negative Output Limit
    pub limit_neg: f32,
    /// Proportional Gain
    pub K_p: f32,
    /// Integral Time. If None the Integral Value is disabled. Panics if Integral Time is Zero
    pub T_i: Option<f32>,
    /// Derivative Time. If None the Derivative Value is disabled.
    pub T_d: Option<f32>,
}

#[allow(non_snake_case)]
impl PIDController {
    /// Creates a new PIDController
    pub fn new(
        K_p: f32,
        T_i: Option<f32>,
        T_d: Option<f32>,
        limit_neg: f32,
        limit_pos: f32,
        setpoint: f32,
    ) -> Self {
        PIDController {
            integral: 0.0,
            last_error: None,
            setpoint,
            limit_pos,
            limit_neg,
            K_p,
            T_i,
            T_d,
        }
    }

    /// Create from parameter struct
    pub fn new_from_par(par: &CtrlParameter) -> Self {
        PIDController::new(par.k_p, par.t_i, par.t_d, par.limit_neg, par.limit_pos, 0.0)
    }

    /// Reset the Integral Value and the saved value for calculating the Derivative value.
    #[allow(unused)]
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = None;
    }

    /// Output the next Controll Output
    pub fn calc_next_output(&mut self, measurement: f32, dt: f32) -> f32 {
        // Control Error
        let error = self.setpoint - measurement;

        // Proportional Error
        // =====
        let p_output = error * self.K_p;

        // Integral Error
        // =====
        let mut new_integral = 0.0;
        let mut i_output = 0.0;

        // Check if Integral is to be calculated
        if let Some(T_i) = self.T_i {
            new_integral = self.integral + dt * error;
            i_output = new_integral * self.K_p / T_i;
        }

        // Differential Error
        // =====
        let mut d_output = 0.0;
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
