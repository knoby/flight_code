use lsm303dlhc;

type I2cSclPin = hal::gpio::PB6<
    hal::gpio::PullNone,
    hal::gpio::AltFn<hal::gpio::AF4, hal::gpio::PushPull, hal::gpio::HighSpeed>,
>;

type I2cSdaPin = hal::gpio::PB7<
    hal::gpio::PullNone,
    hal::gpio::AltFn<hal::gpio::AF4, hal::gpio::PushPull, hal::gpio::HighSpeed>,
>;

type I2c = hal::device::I2C1;

pub struct Sensors {
    //gyro: Type,
    pub acc: lsm303dlhc::Lsm303dlhc<hal::i2c::I2c<I2c, (I2cSclPin, I2cSdaPin)>>,
}
