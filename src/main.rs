//! examples/interrupt.rs

#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m_semihosting::{hprintln};
use stm32f103xx_hal::{
    prelude::*,
    device,
	timer::Timer,
};
use rtfm::app;
use nb::block;

#[app(device = stm32f103xx_hal::device)]
const APP: () = {

	static mut LD2: stm32f103xx_hal::gpio::gpioa::PA5<
		stm32f103xx_hal::gpio::Output<
			stm32f103xx_hal::gpio::PushPull>> = ();

	static mut timer: Timer<stm32f103xx_hal::stm32f103xx::TIM1> = ();


    #[init]
    fn init() {

        hprintln!("init").unwrap();

        // Cortex-M peripherals
        let core: rtfm::Peripherals = core;

        // Device specific peripherals
        let device: device::Peripherals = device;

		let mut rcc = device.RCC.constrain();
		let mut gpioa = device.GPIOA.split(&mut rcc.apb2);		
		

 		let mut flash = device.FLASH.constrain();
		let clocks = rcc.cfgr
			.sysclk(64.mhz())
			.pclk1(32.mhz())
			.pclk2(32.mhz())
			.freeze(&mut flash.acr);

		timer = Timer::tim1( device.TIM1, 1.hz(), clocks, &mut rcc.apb2);
		LD2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);


    }

    #[idle(resources = [LD2, timer])]
    fn idle() -> ! {

        hprintln!("idle").unwrap();

        

        loop {
        	resources.LD2.set_high();
        	resources.timer.start(20.hz());
        	block!(resources.timer.wait()).unwrap();
        	resources.LD2.set_low();
        	resources.timer.start(10.hz());
        	block!(resources.timer.wait()).unwrap();
        }
    }

};
