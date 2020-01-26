use l3gd20;
use lsm303dlhc;

use hal::gpio::{HighSpeed, PullNone, PushPull, AF4, AF5};

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
    pub acc: Lsm303dlhc,
    pub gyro: L3gd20,
}
