#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::gpio::Event;
use esp_hal::gpio::Flex as Flex_;
use esp_hal::gpio::Io;
use esp_hal::gpio::Level;
use esp_hal::gpio::Output;
use esp_hal::gpio::Pull;
use esp_hal::i2c::master::BusTimeout;
use esp_hal::i2c::master::Config;
use esp_hal::i2c::master::I2c;
use esp_hal::interrupt;
use esp_hal::interrupt::InterruptHandler;
use esp_hal::interrupt::Priority;
use esp_hal::main;
use esp_hal::ram;
use esp_hal::handler;
use esp_hal::peripheral::Peripheral;
use esp_hal::peripherals::Interrupt;
use esp_hal::peripherals::Peripherals;
use esp_hal::peripherals::I2C0;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Blocking;
use examples::Flex;
use gt911::Address;
use gt911::GT911;
use gt911::FlexPin;
use heapless::Vec;

static TOUCH: Mutex<
    RefCell<
        Option<
            GT911<I2c<'static, Blocking>, Flex<'static>>,
        >,
    >,
> = Mutex::new(RefCell::new(None));

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut delay = Delay::new();
    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(handler);

    let sda = peripherals.GPIO8;
    let scl = peripherals.GPIO18;

    interrupt::disable(esp_hal::system::Cpu::ProCpu, Interrupt::GPIO);

    let irq_pin: Option<Flex<'_>> = Some(Flex::new(peripherals.GPIO3));
    let mut rst = Flex::new(peripherals.GPIO48);

    let mut i2c = I2c::new(
        peripherals.I2C0,
        Config::default()
            .with_frequency(Rate::from_khz(400))
            .with_timeout(BusTimeout::Disabled),
    )
    .unwrap()
    .with_sda(sda)
    .with_scl(scl);

    let mut touch = gt911::GT911::new(i2c, irq_pin, &mut rst, &mut delay, Address::One)
        .expect("Initialize the touch device");

    critical_section::with(|cs| {
        esp_println::println!("Initialized touch device");

        esp_println::println!("product id: {}", touch.product_id().unwrap());
        esp_println::println!("firmware version: {}", touch.firmware_version().unwrap());

        TOUCH.borrow_ref_mut(cs).replace(touch)
    });

    interrupt::enable(Interrupt::GPIO, Priority::max()).unwrap();

    loop {
        delay.delay_millis(1000);
        esp_println::println!("loop");
    }
}

#[handler]
#[ram]
fn handler() {
    // Print for sanity to confirm interrupt is detected
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
