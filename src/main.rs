//! examples/interrupt.rs

#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m_semihosting::{hprintln};
use stm32f103xx_hal::{
    prelude::*,
    device,
    serial::Serial,
    gpio,
};

use nb::block;

use rtfm::{app, Instant};

const PERIOD: u32 = 64_000_000;


#[app(device = stm32f103xx_hal::device)]
const APP: () = {

	//Resourcen
	static mut LD2: gpio::gpioa::PA5<gpio::Output<gpio::PushPull>> = ();


    #[init(schedule = [hightask])]
    fn init() {
    	hprintln!("Init");

     	// Cortex-M peripherals
        let _core: rtfm::Peripherals = core;

        // Device specific peripherals
        let device: device::Peripherals = device;

        //Freeze clock frequencies
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let clocks = rcc.cfgr
        	.sysclk(64.mhz())
        	.pclk1(32.mhz())
        	.pclk2(64.mhz())
        	.freeze(&mut flash.acr);

        //Configuration of PA5 as output
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        

    	//Start schedulinig of high prio task
    	let now = Instant::now();
    	schedule.hightask(now+PERIOD.cycles()).unwrap();       

    	//Assign late resources
    	LD2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);
    }


    #[idle]
    fn idle() -> ! {
    	hprintln!("Idle");
        loop {
        }
    }


    #[task(schedule = [hightask], resources = [LD2])]
    fn hightask() {
		hprintln!("HighTask");
		schedule.hightask(scheduled+PERIOD.cycles()).unwrap();
		resources.LD2.toggle();
    }


    extern "C" {
    	//fn EXTI0();
    	fn USART1();
    }


};
