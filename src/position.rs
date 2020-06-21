use l3gd20;
use lsm303dlhc;

use hal::gpio::{HighSpeed, PullNone, PushPull, AF4, AF5};

use rtt_target::rprintln;

/// I2C Configuration
type I2cSclPin = hal::gpio::PB6<PullNone, hal::gpio::AltFn<AF4, PushPull, HighSpeed>>;
type I2cSdaPin = hal::gpio::PB7<PullNone, hal::gpio::AltFn<AF4, PushPull, HighSpeed>>;
type I2c = hal::device::I2C1;

/// SPI Configuration
type SpiSckPin = hal::gpio::PA5<PullNone, hal::gpio::AltFn<AF5, PushPull, HighSpeed>>;
type SpiMisoPin = hal::gpio::PA6<PullNone, hal::gpio::AltFn<AF5, PushPull, HighSpeed>>;
type SpiMosiPin = hal::gpio::PA7<PullNone, hal::gpio::AltFn<AF5, PushPull, HighSpeed>>;
type SpiNssPin = hal::gpio::PE3<PullNone, hal::gpio::Output<PushPull, HighSpeed>>;
type Spi = hal::device::SPI1;

/// On board L3GD20 connected to the SPI1 bus via the pins PA5, PA6, PA7 and PE3
pub type L3gd20 =
    l3gd20::L3gd20<hal::spi::Spi<Spi, (SpiSckPin, SpiMisoPin, SpiMosiPin)>, SpiNssPin>;

/// On board LSM303DLHC connected to the I2C1 bus via the PB6 and PB7 pins
pub type Lsm303dlhc = lsm303dlhc::Lsm303dlhc<hal::i2c::I2c<I2c, (I2cSclPin, I2cSdaPin)>>;

// used types
type Vector = nalgebra::Vector3<f32>;

pub struct Sensors {
    acc: Lsm303dlhc,
    gyro: L3gd20,
    gyro_offset: Vector,
    gyro_scale: l3gd20::Scale,
    angle_vel: Vector,
    pub angle: nalgebra::UnitQuaternion<f32>,
}

impl Sensors {
    pub fn new(acc: Lsm303dlhc, gyro: L3gd20, gyro_scale: l3gd20::Scale) -> Self {
        let mut sensors = Self {
            acc,
            gyro,
            gyro_offset: Vector::zeros(),
            gyro_scale,
            angle_vel: Vector::zeros(),
            angle: nalgebra::UnitQuaternion::<f32>::identity(),
        };

        sensors.init_sensors();
        sensors
    }

    pub fn init_sensors(&mut self) {
        rprintln!("Configuration of Gyro Sensor");
        // Set Output data rate. Interval is 10 ms --> min ODR is 190 Hz
        self.gyro.set_odr(l3gd20::Odr::Hz190).unwrap();
        // Use a high bandwith
        self.gyro.set_bandwidth(l3gd20::Bandwidth::High).unwrap();
        // for first try use maximum dps so that every movement is recognised. if resolution is a
        // problemthis can be decreased to Dps500
        self.gyro.set_scale(self.gyro_scale).unwrap();

        rprintln!("Configuration of Acceleration Sensor");
        // Set Output data rate for acc and mag. Controll loop runs at 100Hz --> 200 Hz is minimum
        // for acc reading
        self.acc.accel_odr(lsm303dlhc::AccelOdr::Hz200).unwrap();
        // Set Output Data Rate for Mag rading. The controll loop runs at 100Hz --Y 220 Hz is the
        // datarate to be chosen
        self.acc.mag_odr(lsm303dlhc::MagOdr::Hz220).unwrap();
        // Set Sensitivity. To be able to detect acc upwards we need to detect more than 1g
        self.acc
            .set_accel_sensitivity(lsm303dlhc::Sensitivity::G4)
            .unwrap();

        // Calculate the Offset of the gyre reading by reading 100 Values and divinding the sum by
        // 100
        rprintln!("Collect Data for Gyro Callibration");
        let mut gyro_offset = Vector::zeros();
        for _ in 0..100 {
            let reading = self.gyro.gyro().unwrap();
            gyro_offset.x += self.gyro_scale.radians(reading.x);
            gyro_offset.y += self.gyro_scale.radians(reading.y);
            gyro_offset.z += self.gyro_scale.radians(reading.z);

            cortex_m::asm::delay(640_000);
        }
        self.gyro_offset = -gyro_offset / 100.0;
    }

    pub fn update(&mut self, dt: f32) {
        // Update the Orientation with the algorithem from ttp://philstech.blogspot.com/

        // Update gyro data
        let gyro_data = self.gyro.gyro().unwrap();
        // Update Acc Data
        let acc_data = self.acc.accel().unwrap();
        // Update Mag data
        let mag_data = self.acc.mag().unwrap();

        // Convert Gyrodata to si units and compensate Offset
        self.angle_vel = Vector::new(
            -self.gyro_scale.radians(gyro_data.x),
            self.gyro_scale.radians(gyro_data.y),
            self.gyro_scale.radians(gyro_data.z),
        ) + self.gyro_offset;

        // Calculate gravity vector in body frame
        let acc_body = Vector::new(
            acc_data.y as f32 / 16384.0 * 9.81 * 4.0,
            acc_data.x as f32 / 16384.0 * 9.81 * 4.0,
            acc_data.z as f32 / 16384.0 * 9.81 * 4.0,
        );

        // Calculate mag vector in body frame
        let mag_body = Vector::new(mag_data.x as f32, mag_data.y as f32, mag_data.z as f32);

        // Convert gravity Vector to world frame with estimation of orientation from last cycle
        let mut acc_world = self.angle.transform_vector(&acc_body);
        acc_world /= acc_world.norm();
        // Convert mag vector to world frame with estimation of orientation from last cycle
        let mut mag_world = self.angle.transform_vector(&mag_body);
        // Remove z-component/Inclination
        mag_world.z = 0.0;
        mag_world /= mag_world.norm();

        // Compare the normalised acc vector with gravity
        let grav_world = Vector::new(0.0, 0.0, 1.0);
        let grav_correction_world = acc_world.cross(&grav_world) * 0.01;

        // Compare with north direction/x-Axis
        let north_world = Vector::new(1.0, 0.0, 0.0);
        let mag_correction_world = mag_world.cross(&north_world) * 0.01;

        // Rotate correction Vector back to Body frame
        let correction_body = self
            .angle
            .inverse_transform_vector(&(grav_correction_world + mag_correction_world));

        // Dived by the sample rate and add to gyro measurement
        let gyro_data_with_correction = (self.angle_vel * dt) + correction_body;

        // Integrate Velocit and add to current estimation
        let delta_angle = nalgebra::geometry::UnitQuaternion::from_euler_angles(
            gyro_data_with_correction.x,
            gyro_data_with_correction.y,
            gyro_data_with_correction.z,
        );
        self.angle *= delta_angle;
    }

    /// Get roll pitch and yaw angle vel
    pub fn angle_vel(&self) -> (f32, f32, f32) {
        (self.angle_vel.x, self.angle_vel.y, self.angle_vel.z)
    }

    /// Get Euler Angles
    pub fn euler_angles(&self) -> (f32, f32, f32) {
        self.angle.euler_angles()
    }
}
