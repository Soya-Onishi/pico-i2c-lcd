#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use panic_probe as _;

use lcd_lcm1602_i2c::Lcd;
use rp_pico as bsp;

use embedded_hal::timer::CountDown;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
    watchdog::Watchdog,
};

use fugit::ExtU32;
use fugit::RateExtU32;

#[entry]
fn main() -> ! {
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let timer = bsp::hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Construct UART
    let uart_pins = (pins.gpio0.into_mode(), pins.gpio1.into_mode());
    let uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    defmt_serial::defmt_serial(uart);
    info!("ready to logging\r\n");

    let mut i2c = bsp::hal::i2c::I2C::i2c0(
        pac.I2C0,
        pins.gpio4.into_mode(),
        pins.gpio5.into_mode(),
        100.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let lcd = match Lcd::new(&mut i2c, &mut delay)
        .address(0x27)
        .cursor_on(false)
        .rows(2)
        .init()
    {
        Ok(lcd) => lcd,
        Err(e) => {
            info!("error: {:?}", defmt::Debug2Format(&e));
            loop {}
        }
    };

    if let Err(e) = lcd_manipulation(lcd, timer) {
        debug!("{:?}\r\n", defmt::Debug2Format(&e));
    }

    loop {}
}

fn lcd_manipulation<'a, I, D>(
    mut lcd: Lcd<'a, I, D>,
    timer: bsp::hal::Timer,
) -> Result<(), <I as embedded_hal::blocking::i2c::Write>::Error>
where
    I: embedded_hal::blocking::i2c::Write,
    I::Error: core::fmt::Debug,
    D: embedded_hal::blocking::delay::DelayMs<u8>,
{
    let mut count_down = timer.count_down();
    loop {
        lcd.clear()?;
        lcd.return_home()?;
        lcd.write_str("Hello World")?;
        lcd.set_cursor(1, 0)?;
        lcd.write_str("From Rust")?;

        debug!("Hello World From Rust\r\n");

        count_down.start(1_000.millis());
        let _ = nb::block!(count_down.wait());

        lcd.clear()?;
        lcd.return_home()?;
        lcd.write_str("This is a Test")?;
        lcd.set_cursor(1, 0)?;
        lcd.write_str("Message For You")?;

        debug!("This is a Test Message For You\r\n");

        count_down.start(1_000.millis());
        let _ = nb::block!(count_down.wait());
    }
}
