//! examples/interrupt.rs

#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m_semihosting::{hprintln};
use stm32f103xx_hal::{
    prelude::*,
    device,
    gpio,
};


use rtfm::{app, Instant};

const PERIOD: u32 = 64_000_000;


#[app(device = stm32f103xx_hal::device)]
const APP: () = {

	//Resourcen
	static mut LD2: gpio::gpioa::PA5<gpio::Output<gpio::PushPull>> = ();
	static mut PB: gpio::gpioc::PC13<gpio::Input<gpio::Floating>> = ();


    #[init(schedule = [hightask])]
    fn init() {
    	hprintln!("Init").unwrap();

     	// Cortex-M peripherals
        let _core: rtfm::Peripherals = core;

        // Device specific peripherals
        let device: device::Peripherals = device;

        //Freeze clock frequencies
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let _clocks = rcc.cfgr
        	.sysclk(64.mhz())
        	.pclk1(32.mhz())
        	.pclk2(64.mhz())
        	.freeze(&mut flash.acr);

        //Configuration of PA5 as output and PC13 as Input
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);      

        //Schedule the Hight Task
        let now = Instant::now();
        schedule.hightask(now+PERIOD.cycles()).unwrap();


    	//Assign late resources
    	LD2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);
    	PB = gpioc.pc13.into_floating_input(&mut gpioc.crh);
    	
    }


    #[idle(resources = [LD2, PB])]
    fn idle() -> ! {
    	hprintln!("Idle").unwrap();

        loop {
        	if resources.PB.is_high() {
        		resources.LD2.lock(|led| led.set_high());
        	} else {
        		resources.LD2.lock(|led| led.set_low());
        	}
        }
    }


    #[task(schedule = [hightask], resources = [LD2])]
    fn hightask() {
		hprintln!("HighTask").unwrap();
		resources.LD2.toggle();
		schedule.hightask(scheduled+PERIOD.cycles()).unwrap();
    }


    extern "C" {
    	//fn EXTI0();
    	fn USART1();
    }


};
