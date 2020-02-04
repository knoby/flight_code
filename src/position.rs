use l3gd20;
use lsm303dlhc;

use hal::gpio::{HighSpeed, PullNone, PushPull, AF4, AF5};

extern crate cortex_m_semihosting;
use cortex_m_semihosting::dbg;

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

pub struct Sensors {
    acc: Lsm303dlhc,
    acc_offset: [f32; 3],
    gyro: L3gd20,
    gyro_offset: [f32; 3],
    gyro_scale: l3gd20::Scale,
    angle_vel: nalgebra::Vector3<f32>,
    angle: nalgebra::UnitQuaternion<f32>,
}

impl Sensors {
    pub fn new(acc: Lsm303dlhc, gyro: L3gd20, gyro_scale: l3gd20::Scale) -> Self {
        let mut sensors = Self {
            acc,
            acc_offset: [0.0; 3],
            gyro,
            gyro_offset: [0.0; 3],
            gyro_scale,

            angle_vel: nalgebra::Vector3::<f32>::zeros(),
            angle: nalgebra::UnitQuaternion::<f32>::identity(),
        };

        sensors.init_sensors();
        sensors
    }

    pub fn init_sensors(&mut self) {
        // Set Output data rate. Interval is 10 ms --> min ODR is 190 Hz
        self.gyro.set_odr(l3gd20::Odr::Hz190).unwrap();
        // Use a high bandwith
        self.gyro.set_bandwidth(l3gd20::Bandwidth::High).unwrap();
        // for first try use maximum dps so that every movement is recognised. if resolution is a
        // problemthis can be decreased to Dps500
        self.gyro.set_scale(self.gyro_scale).unwrap();

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
        let mut gyro_offset = [0.0_f32; 3];
        let mut acc_offset = [0.0_f32; 3];
        for _ in 0..100 {
            let reading = self.gyro.gyro().unwrap();
            gyro_offset[0] += self.gyro_scale.degrees(reading.x);
            gyro_offset[1] += self.gyro_scale.degrees(reading.y);
            gyro_offset[2] += self.gyro_scale.degrees(reading.z);

            let reading = self.acc.accel().unwrap();
            acc_offset[0] += reading.x as f32 / 16384.0 * 9.81 * 4.0;
            acc_offset[1] += reading.y as f32 / 16384.0 * 9.81 * 4.0;
            acc_offset[2] += reading.z as f32 / 16384.0 * 9.81 * 4.0;

            cortex_m::asm::delay(640_000);
        }
        for (i, value) in gyro_offset.iter().enumerate() {
            self.gyro_offset[i] = value / 100.0;
        }
        for (i, value) in acc_offset.iter().enumerate() {
            self.acc_offset[i] = value / 100.0;
        }
        self.acc_offset[2] -= 9.81; // The gravity Offset is not needed here
        dbg!(self.acc_offset);
    }

    pub fn update(&mut self) {
        // Update gyro data
        let gyro_data = self.gyro.all().unwrap();
        // Update Acc Data
        let _acc_data = self.acc.accel().unwrap();
        // Update Mag data
        let _mag_data = self.acc.mag().unwrap();

        // Convert Gyrodata to si units
        self.angle_vel = [
            self.gyro_scale.radians(gyro_data.gyro.x),
            self.gyro_scale.radians(gyro_data.gyro.x),
            self.gyro_scale.radians(gyro_data.gyro.x),
        ]
        .into();

        // Convert the acc data to Normalized Vector
    }
}
