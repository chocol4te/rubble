#![no_main]
#![no_std]

#[allow(unused_imports)]
use panic_semihosting;

use cortex_m_semihosting::{debug, hprintln};
use rtic::app;

use nrf51_hal as hal;

#[app(device = crate::hal::pac)]
const APP: () = {
    #[init]
    fn init(_: init::Context) {
        hprintln!("init").unwrap();
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        hprintln!("idle").unwrap();

        loop {
            hprintln!("loop").unwrap();
        }
    }
};
