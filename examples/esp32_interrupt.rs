#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use esp32s3_hal::clock::ClockControl;
use esp32s3_hal::delay::Delay;
use esp32s3_hal::entry;
use esp32s3_hal::gpio::Event;
use esp32s3_hal::gpio::Flex as Flex_;
use esp32s3_hal::gpio::Io;
use esp32s3_hal::gpio::Level;
use esp32s3_hal::gpio::Output;
use esp32s3_hal::gpio::Pull;
use esp32s3_hal::i2c::I2C;
use esp32s3_hal::interrupt;
use esp32s3_hal::interrupt::Priority;
use esp32s3_hal::peripheral::Peripheral;
use esp32s3_hal::peripherals::Interrupt;
use esp32s3_hal::peripherals::Peripherals;
use esp32s3_hal::peripherals::I2C0;
use esp32s3_hal::prelude::*;
use esp32s3_hal::rtc_cntl::Rtc;
use esp32s3_hal::system::SystemControl;
use esp32s3_hal::timer::timg::TimerGroup;
use esp32s3_hal::Blocking;
use esp_backtrace as _;
use gt911::esp32::Flex;
use gt911::Address;
use gt911::FlexPin;
use gt911::GT911;
use heapless::Vec;

// impl<I2C, IRQ> GT911<esp32s3_hal::i2c::I2C, Flex>

static TOUCH: Mutex<
    RefCell<
        Option<
            GT911<
                esp32s3_hal::i2c::I2C<'static, I2C0, Blocking>,
                Flex<'static, esp32s3_hal::gpio::Gpio3>,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));
#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // TODO this doesn't seem to be necessary

    // let mut rtc = Rtc::new(peripherals.LPWR, None);
    // let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    // let mut wdt0 = timer_group0.wdt;
    // let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks, None);
    // let mut wdt1 = timer_group1.wdt;
    // rtc.swd.disable();
    // rtc.rwdt.disable();
    // wdt0.disable();
    // wdt1.disable();

    // timer_group0.timer0.unlisten();
    // timer_group0.timer1.unlisten();
    // timer_group1.timer0.unlisten();
    // timer_group1.timer1.unlisten();

    // rtc.rwdt.unlisten();

    let mut delay = Delay::new(&clocks);
    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    io.set_interrupt_handler(GPIO);

    let mut sda = io.pins.gpio8;
    let mut scl = io.pins.gpio18;

    // let mut backlight = Output::new(io.pins.gpio47, Level::High);

    interrupt::disable(esp32s3_hal::Cpu::ProCpu, Interrupt::GPIO);

    let mut irq_pin = Flex::new(io.pins.gpio3);
    let mut rst = Flex::new(io.pins.gpio48);

    let mut i2c = I2C::new(
        peripherals.I2C0,
        sda,
        scl,
        // 400u32.kHz(),
        // Interrupt reset only somewhat works with slower bus frequency
        1u32.kHz(),
        &clocks,
        None,
    );

    let mut touch = gt911::GT911::new(i2c, Some(irq_pin), &mut rst, &mut delay, Address::One)
        .expect("Initialize the touch device");

    critical_section::with(|cs| {
        esp_println::println!("Initialized touch device");

        esp_println::println!("product id: {}", touch.product_id().unwrap());
        esp_println::println!("firmware version: {}", touch.firmware_version().unwrap());

        TOUCH.borrow_ref_mut(cs).replace(touch)
    });

    interrupt::enable(Interrupt::GPIO, Priority::Priority3).unwrap();

    loop {
        delay.delay_millis(1000);
        esp_println::println!("loop");
    }
}

#[handler]
fn GPIO() {
    // Print for sanity to confirm interrupt is detecte
    esp_println::println!("GPIO  Interrupt!");
    critical_section::with(|cs| {
        let mut touches = Vec::<_, 5>::new();
        TOUCH.borrow_ref_mut(cs).as_mut().map(|touch| {
            let mut touch_handle = touch.read_handle().unwrap();
            let n_touches = touch_handle.status().get_touches();
            let n_keys = touch_handle.status().is_pressed();

            if n_touches > 0 {
                let _ = touch_handle.read_touches_into(&mut touches).unwrap();
                esp_println::println!(
                    "touches registered: {:}\ncoordinates: {:?}",
                    n_touches,
                    &touches
                );
            }
            if n_keys {
                if let Some(key) = touch_handle.read_key_value().unwrap() {
                    esp_println::println!(
                        "key presses registered: {:}\n key-id: {:?}",
                        n_keys,
                        key
                    );
                }
            }
            // touch_handle.close();
        })
    });
}
