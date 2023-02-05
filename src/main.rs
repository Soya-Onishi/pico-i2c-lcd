#![no_std]
#![no_main]

use bsp::{entry, hal::uart::Enabled};
use core::{
    borrow::Borrow,
    fmt::{self, Write},
};
use defmt::*;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use panic_probe as _;

use cortex_m::delay::Delay;
use pcf857x::{Pcf8574, SlaveAddr};
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{
        bank0::{Gpio0, Gpio1},
        Function, Output, Pin, PushPull, Uart,
    },
    pac::{self, UART0},
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
    watchdog::Watchdog,
};

use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};

use fugit::RateExtU32;

static UART: Mutex<
    RefCell<
        Option<
            UartPeripheral<
                Enabled,
                UART0,
                (Pin<Gpio0, Function<Uart>>, Pin<Gpio1, Function<Uart>>),
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

struct UartWriter<'a>(
    &'a UartPeripheral<Enabled, UART0, (Pin<Gpio0, Function<Uart>>, Pin<Gpio1, Function<Uart>>)>,
);

macro_rules! print {
    ($($arg:tt)*) => {
        $crate::_print(format_args!($($arg)*));
    };
}

fn _print(args: fmt::Arguments) {
    free(|cs| {
        let uart = UART.borrow(cs).borrow();
        let uart = uart.as_ref().unwrap();
        let mut writer = UartWriter(uart);
        writer.write_fmt(args).unwrap();
    });
}

impl Write for UartWriter<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.0.write_full_blocking(s.as_bytes());
        Ok(())
    }
}

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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut led_pin = pins.led.into_push_pull_output();

    // Construct UART
    let uart_pins = (pins.gpio0.into_mode(), pins.gpio1.into_mode());
    let uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    defmt_serial::defmt_serial(uart);
    defmt::info!("ready to logging");

    let i2c = bsp::hal::i2c::I2C::i2c0(
        pac.I2C0,
        pins.gpio4.into_mode(),
        pins.gpio5.into_mode(),
        100.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    delay.delay_ms(500);

    let mut pcf8574a = Pcf8574::new(i2c, SlaveAddr::Alternative(true, true, true));
    let init_commands = [
        0b0011_0000,
        0b0011_0000,
        0b0011_0000,
        0b0010_0000,
        //function set command
        0b0010_1000,
        0b0000_1000,
        // display command
        0b0000_1000,
        0b1110_1000,
        // entry mode set
        0b0000_1000,
        0b0110_1000,
        //write `A`
        0b0100_1001,
        0b1000_1001,
    ];
    for c in init_commands {
        if let Err(e) = pcf8574a.set(c | 0b0000_0100) {
            print!("{:?}\r\n", e);
            led_pin.set_high().unwrap();
        }

        delay.delay_ms(20);

        if let Err(e) = pcf8574a.set(c) {
            print!("{:?}\r\n", e);
            led_pin.set_high().unwrap();
        }

        delay.delay_ms(20);
    }

    loop {}
}

fn debug_toggle(
    led: &mut Pin<bsp::hal::gpio::pin::bank0::Gpio25, Output<PushPull>>,
    delay: &mut Delay,
) {
    led.toggle();
    delay.delay_ms(1000);
}
