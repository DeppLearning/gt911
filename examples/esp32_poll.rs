#![no_std]
#![no_main]

use esp32s3_hal::delay::Delay;
use esp32s3_hal::entry;
use esp32s3_hal::gpio::Input;
use esp32s3_hal::gpio::Io;
use esp32s3_hal::gpio::Pull;
use esp32s3_hal::i2c::I2C;
use esp32s3_hal::peripherals::Peripherals;
use esp32s3_hal::prelude::*;
use esp32s3_hal::rtc_cntl::Rtc;
use esp32s3_hal::timer::timg::TimerGroup;
use esp_backtrace as _;
use gt911::esp32::Flex;
use gt911::Address;
use heapless::Vec;

#[entry]
fn main() -> ! {
    let peripherals = esp32s3_hal::init(esp32s3_hal::Config::default());

    let mut delay = Delay::new();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let sda = io.pins.gpio8;
    let scl = io.pins.gpio18;

    let irq_pin = Some(Flex::new(Input::new(io.pins.gpio3, Pull::Down)));
    let mut rst = Flex::new(Input::new(io.pins.gpio48, Pull::Down));

    let i2c = I2C::new(peripherals.I2C0, sda, scl, 1u32.kHz());

    let mut touch = gt911::GT911::new(i2c, irq_pin, &mut rst, &mut delay, Address::One)
        .expect("Initialize the touch device");

    esp_println::println!("Initialized touch device");

    esp_println::println!("product id: {}", touch.product_id().unwrap());
    esp_println::println!("firmware version: {}", touch.firmware_version().unwrap());
    let mut touches = Vec::<_, 5>::new();

    loop {
        if !touch.data_available().unwrap() {
            continue;
        }
        let mut touch_handle = touch.read_handle().unwrap();
        let status = touch_handle.status();
        if !status.is_ready() {
            continue;
        }
        let n_touches = status.get_touches();
        let n_keys = status.is_pressed();

        let _ = touch_handle.read_touches_into(&mut touches).unwrap();
        let key = touch_handle.read_key_value().unwrap();
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
                esp_println::println!("key presses registered: {:}\n key-id: {:?}", n_keys, key);
            }
        }
    }
}
