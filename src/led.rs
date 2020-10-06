//! On-board user LEDs

#![allow(dead_code)]

use core::ops;

use hal::gpio::gpioe::{PEx, PE10, PE11, PE12, PE13, PE14, PE15, PE8, PE9};
use hal::gpio::{Output, PushPull};
use hal::prelude::*;

///  North LED
pub type LD3 = PE9<Output<PushPull>>;
pub type LedN = PE9<Output<PushPull>>;

/// Northeast LED
pub type LD5 = PE10<Output<PushPull>>;
pub type LedNE = PE10<Output<PushPull>>;

/// East LED
pub type LD7 = PE11<Output<PushPull>>;
pub type LedE = PE11<Output<PushPull>>;

/// Southeast LED
pub type LD9 = PE12<Output<PushPull>>;
pub type LedSE = PE12<Output<PushPull>>;

/// South LED
pub type LD10 = PE13<Output<PushPull>>;
pub type LedS = PE13<Output<PushPull>>;

/// Southwest LED
pub type LD8 = PE14<Output<PushPull>>;
pub type LedSW = PE14<Output<PushPull>>;

/// West LED
pub type LD6 = PE15<Output<PushPull>>;
pub type LedW = PE15<Output<PushPull>>;

/// Northwest LED
pub type LD4 = PE8<Output<PushPull>>;
pub type LedNW = PE8<Output<PushPull>>;

/// Cardinal directions. Each one matches one of the user LEDs.
pub enum Direction {
    /// North / LD3
    North,
    /// Northeast / LD5
    Northeast,
    /// East / LD7
    East,
    /// Southeast / LD9
    Southeast,
    /// South / LD10
    South,
    /// Southwest / LD8
    Southwest,
    /// West / LD6
    West,
    /// Northwest / LD4
    Northwest,
}

/// Array of all the user LEDs on the board
pub struct Leds {
    leds: [Led; 8],
}

impl ops::Deref for Leds {
    type Target = [Led];

    fn deref(&self) -> &[Led] {
        &self.leds
    }
}

impl ops::DerefMut for Leds {
    fn deref_mut(&mut self) -> &mut [Led] {
        &mut self.leds
    }
}

impl ops::Index<usize> for Leds {
    type Output = Led;

    fn index(&self, i: usize) -> &Led {
        &self.leds[i]
    }
}

impl ops::Index<Direction> for Leds {
    type Output = Led;

    fn index(&self, d: Direction) -> &Led {
        &self.leds[d as usize]
    }
}

impl ops::IndexMut<usize> for Leds {
    fn index_mut(&mut self, i: usize) -> &mut Led {
        &mut self.leds[i]
    }
}

impl ops::IndexMut<Direction> for Leds {
    fn index_mut(&mut self, d: Direction) -> &mut Led {
        &mut self.leds[d as usize]
    }
}

/// One of the on-board user LEDs
pub struct Led {
    state: bool,
    pex: PEx<Output<PushPull>>,
}

macro_rules! ctor {
    ($($ldx:ident),+) => {
        $(
            impl Into<Led> for $ldx {
                fn into(self) -> Led {
                    let mut led = self.downgrade();
                    led.set_low().unwrap();
                    Led {
                        state: false,
                        pex: led,
                    }
                }
            }
        )+
    }
}

ctor!(LD3, LD4, LD5, LD6, LD7, LD8, LD9, LD10);

impl Led {
    /// Turns the LED off
    pub fn off(&mut self) {
        self.state = false;
        self.set_output();
    }

    /// Turns the LED on
    pub fn on(&mut self) {
        self.state = true;
        self.set_output();
    }

    pub fn toggle(&mut self) {
        self.state = !self.state;
        self.set_output();
    }

    fn set_output(&mut self) {
        if self.state {
            self.pex.set_high().unwrap();
        } else {
            self.pex.set_low().unwrap();
        }
    }
}
