#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::gpio::Io;
use esp_hal::gpio::Level;
use esp_hal::gpio::Output;
use esp_hal::i2c::master::BusTimeout;
use esp_hal::i2c::master::Config;
use esp_hal::i2c::master::I2c;
use esp_hal::main;
use esp_hal::time::Rate;
use examples::Flex;
use gt911::Address;
use heapless::Vec;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut delay = Delay::new();
    let io = Io::new(peripherals.IO_MUX);

    let sda = peripherals.GPIO8;
    let scl = peripherals.GPIO18;

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
    // let reg: u16 = 0x8040;
    // i2c.write(0x5d, &reg.to_be_bytes()).unwrap();
    let mut touch = gt911::GT911::new(i2c, irq_pin, &mut rst, &mut delay, Address::One).unwrap();

    esp_println::println!("Initialized touch device");

    esp_println::println!("product id: {}", core::str::from_utf8(touch.product_id().unwrap().to_le_bytes().as_slice()).unwrap());
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
