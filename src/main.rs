#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use panic_probe as _;

use cortex_m::delay::Delay;
use pcf857x::{Pcf8574, SlaveAddr};
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{Output, Pin, PushPull},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use fugit::RateExtU32;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut led_pin = pins.led.into_push_pull_output();

    debug_toggle(&mut led_pin, &mut delay);

    let i2c = bsp::hal::i2c::I2C::i2c0(
        pac.I2C0,
        pins.gpio4.into_mode(),
        pins.gpio5.into_mode(),
        100.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    debug_toggle(&mut led_pin, &mut delay);

    let mut pcf8574a = Pcf8574::new(i2c, SlaveAddr::Default);
    debug_toggle(&mut led_pin, &mut delay);
    pcf8574a.write_array(b"Hello I2C!\r\n").unwrap();

    debug_toggle(&mut led_pin, &mut delay);

    loop {}
}

fn debug_toggle(
    led: &mut Pin<bsp::hal::gpio::pin::bank0::Gpio25, Output<PushPull>>,
    delay: &mut Delay,
) {
    led.toggle();
    delay.delay_ms(1000);
}
