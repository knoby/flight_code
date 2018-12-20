#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
//extern crate panic_semihosting;

extern crate stm32f1;
use stm32f1::stm32f103;

use cortex_m_rt::entry;


#[entry]
fn main() -> ! {

    let p  = stm32f103::Peripherals::take().unwrap();

    //Enable Clock
    p.RCC.apb2enr.write( |w| w.iopaen().bit(true) );

    //Reset Peripherial Clock
    p.RCC.apb2rstr.write( |w| w.ioparst().bit(true) );
    p.RCC.apb2rstr.write( |w| w.ioparst().bit(false) );

    //Config GPIO
	p.GPIOA.crl.write( |w| unsafe{ w.cnf5().bits(0b00).mode5().bits(0b11) });

    //Set Output to high
    p.GPIOA.bsrr.write( |w| w.bs5().bit(true) ) ;


    loop {
        // your code goes here
    }
}