//! GT911 Multi-Touch Touchscreen Controller

#![cfg_attr(not(test), no_std)]

use core::fmt::Debug;
use core::mem::size_of;

use bitfield_struct::bitfield;
use config::Config;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::ErrorType;
use embedded_hal::i2c::I2c;
use embedded_hal::i2c::Operation;
use zerocopy::little_endian::U16;
use zerocopy::AsBytes;
use zerocopy::FromBytes;
use zerocopy::FromZeroes;
use zerocopy::Unaligned;

#[macro_use]
extern crate getset;

pub mod config;

#[cfg(feature = "esp32s3")]
pub mod esp32;

const GT911_ADDR_1: u8 = 0x5D;
const GT911_ADDR_2: u8 = 0x14;

/// Command register
const GT911_REG_COMMAND: u16 = 0x8040;
/// Starting Product-id register
const GT911_REG_PROD_ID: u16 = 0x8140;
/// Starting Firmware version register
const GT911_REG_FIRM_VER: u16 = 0x8144;
/// Starting Touch status register
const GT911_REG_TOUCH_STATUS: u16 = 0x814E;
/// Starting First touch point register
const GT911_REG_COORD_POINT_1: u16 = 0x8150;
/// Starting Config register
const GT911_REG_CONFIG: u16 = 0x8047;

/// Any type of error which may occur while interacting with the device
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// Some error originating from the communication bus
    BusError(E),
    /// The alignment did not match the expectation
    InvalidAlignment,
    /// Reading a GPIO pin resulted in an error
    IOError,
    /// Tried to read a touch point, but no data was available
    NoDataAvailable,
}

impl<I2C: ErrorType, IRQ> ErrorType for GT911<I2C, IRQ>
where
    Error<<I2C as embedded_hal::i2c::ErrorType>::Error>: embedded_hal::i2c::Error,
{
    type Error = Error<<I2C as ErrorType>::Error>;
}

/// A trait for pins that can be used dynamically either as input and output.
pub trait FlexPin: InputPin + OutputPin {
    /// Clears interrupt of GPIO pin
    fn clear_interrupt(&mut self) {}
    /// Sets GPIO pin to output mode
    fn set_as_output(&mut self) {}
    /// Sets GPIO pin to output mode
    fn set_as_input(&mut self) {}
    /// Listens for interrupts on the rising pin edge
    fn listen_on_rising_edge(&mut self) {}
}

/// I2C address of the device.
///
/// The GT911 can utilize two addresses for its I2C communication. However, some
/// development boxes using the device, do not support both addresses. For
/// example, the ESP32-S3-BOX-3 only seems to support the `One` variant.
///
/// If you encounter I2C communication errors (i.e. `not acknowledged`) try
/// using another address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Address {
    One = 0x5d,
    Two = 0x14,
}

impl From<Address> for u8 {
    fn from(value: Address) -> Self {
        match value {
            Address::One => GT911_ADDR_1,
            Address::Two => GT911_ADDR_2,
        }
    }
}

/// Represents any errors that can occur when parsing [Address].
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AddressParsingError;

impl TryFrom<u8> for Address {
    type Error = AddressParsingError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x5d => Ok(Address::One),
            0x14 => Ok(Address::Two),
            _ => Err(AddressParsingError),
        }
    }
}

/// Status of the touch input
///
/// Represents the `0x814E` register, see section `3.3 Coordinate Information`
/// of the GT911 Programming Guide v.0.1.
#[bitfield(u8, order = Msb)]
#[derive(PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TouchStatus {
    /// Touch or key input is ready to be read
    buffer_status: bool,
    /// Large-area touch has been registered
    large_detect: bool,
    proximity_valid: bool,
    /// Key press has been registered
    have_key: bool,
    /// Number of touch points registered
    #[bits(4, access = RO)]
    touches: u8,
}

impl TouchStatus {
    /// Returns the number of touch points registered
    pub fn get_touches(&self) -> u8 {
        self.touches()
    }

    /// Returns whether touch points are ready to be read
    pub fn is_touched(&self) -> bool {
        self.touches() > 0
    }

    /// Returns whether key presses are ready to be read
    pub fn is_pressed(&self) -> bool {
        self.have_key()
    }

    /// Returns whether touch points or key presses are ready to be read
    pub fn is_ready(&self) -> bool {
        self.is_touched() || self.is_pressed()
    }
}

/// Represents a single touch point
#[derive(Debug, Default, Clone, FromZeroes, FromBytes, AsBytes, Unaligned, PartialEq, Eq)]
#[repr(C)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TouchPoint {
    /// X coordinate
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    x: U16,
    /// Y coordinate
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    y: U16,
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    /// size of the touch point
    size: U16,
}

impl TouchPoint {
    pub fn x(&self) -> u16 {
        self.x.into()
    }

    pub fn y(&self) -> u16 {
        self.y.into()
    }
}

/// A handle to read touch data.
///
/// Resets the `buffer-status` flag in the `0x814E` register on [Drop] (see
/// [Self::close]).
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ReadHandle<'a, I2C, IRQ>
where
    I2C: I2c,
    IRQ: FlexPin,
{
    status: TouchStatus,
    driver: &'a mut GT911<I2C, IRQ>,
}

impl<'a, I2C, IRQ> ReadHandle<'a, I2C, IRQ>
where
    I2C: I2c,
    IRQ: FlexPin,
{
    /// Reads the first touch event.
    ///
    /// Like [Self::read_touch_into] but reads the result into the provided
    /// buffer.
    pub fn read_touch(&mut self) -> Result<Option<TouchPoint>, Error<<I2C as ErrorType>::Error>> {
        let status = self.status;
        let touches = status.touches();
        if status.buffer_status() && touches > 0 {
            let mut buf = [0; size_of::<TouchPoint>()];
            self.driver.read(GT911_REG_COORD_POINT_1, &mut buf)?;
            let point = zerocopy::transmute!(buf);
            return Ok(Some(point));
        }

        Ok(None)
    }

    /// Reads the first touch event.
    pub fn read_touch_into<'buf>(
        &mut self,
        buf: &'buf mut [u8; size_of::<TouchPoint>()],
    ) -> Result<Option<&'buf TouchPoint>, Error<<I2C as ErrorType>::Error>> {
        let status = self.status;
        let touches = status.touches();
        if status.buffer_status() && touches > 0 {
            self.driver.read(GT911_REG_COORD_POINT_1, buf)?;
            let point = TouchPoint::slice_from(buf).ok_or(Error::InvalidAlignment)?;
            return Ok(Some(&point[0]));
        }

        Ok(None)
    }

    #[cfg(feature = "heapless")]
    /// Reads up to `N` touches.
    pub fn read_touches<const N: usize>(
        &mut self,
    ) -> Result<heapless::Vec<TouchPoint, N>, Error<<I2C as ErrorType>::Error>> {
        let mut out = heapless::Vec::new();
        self.read_touches_into(&mut out)?;

        Ok(out)
    }

    #[cfg(feature = "heapless")]
    /// Reads up to `N` touches.
    ///
    /// Like [Self::read_touches_into] but reads the result into the provided
    /// buffer.
    pub fn read_touches_into<const N: usize>(
        &mut self,
        buf: &mut heapless::Vec<TouchPoint, N>,
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        let status = self.status;
        let touches = status.touches();
        buf.clear();
        if status.buffer_status() && touches > 0 {
            buf.resize((touches as usize).min(N), TouchPoint::default())
                .unwrap();
            for (i, tp) in buf.iter_mut().enumerate() {
                let b = tp.as_bytes_mut();
                self.driver
                    .read(GT911_REG_COORD_POINT_1 + i as u16 * 8, b)?;
            }
        }

        Ok(())
    }

    /// Reads index of pressed key
    pub fn read_key_value(&mut self) -> Result<Option<u8>, Error<<I2C as ErrorType>::Error>> {
        let mut b = [0; 1];
        if self.read_key_value_into(&mut b)?.is_some() {
            Ok(Some(b[0]))
        } else {
            Ok(None)
        }
    }

    /// Reads index of pressed key
    ///
    /// Like [Self::read_key_value] but reads the result into the provided
    /// buffer.
    pub fn read_key_value_into(
        &mut self,
        buf: &mut [u8; 1],
    ) -> Result<Option<()>, Error<<I2C as ErrorType>::Error>> {
        let status = self.status;
        let mut out = None;
        if status.buffer_status() && status.have_key() {
            // the address of the key value depends on the number of touch points registered
            let touches = status.touches();
            self.driver
                .read(GT911_REG_COORD_POINT_1 + touches as u16 * 8 + 7, buf)?;
            out = Some(());
        }

        Ok(out)
    }

    /// Returns the [TouchStatus]
    pub fn status(&self) -> TouchStatus {
        self.status
    }

    /// Releases any allocated resources
    pub fn release(self) -> TouchStatus {
        self.status
    }

    /// Closes [ReadHandle]
    ///
    /// This resets the touch status of the device, such that it will continue
    /// to register new touch data. Use this only when you want to handle the
    /// error this might return explicitly.
    pub fn close(self) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        // only reset when there is touch data in the buffer, otherwise, the driver
        // might miss registered touches.
        // if self.status().buffer_status() {
        self.driver.reset_touch_status()?;
        // TODO prevent drop being called
        // reset buffer status to avoid the `self.driver.reset_touch_status` method
        // being called again (either here or in the drop impl)
        // self.status = self.status.with_buffer_status(false);
        // }
        Ok(())
    }
}

impl<'a, I2C, IRQ> Drop for ReadHandle<'a, I2C, IRQ>
where
    I2C: I2c,
    IRQ: FlexPin,
{
    fn drop(&mut self) {
        // only reset when there is touch data in the buffer, otherwise, the driver
        // might miss registered touches.
        // self.status().buffer_status() &&
        if self.driver.reset_touch_status().is_err() {
            #[cfg(feature = "defmt")]
            defmt::error!("failed to reset device touch status");
        }
        if let Some(pin) = self.driver.int.as_mut() {
            pin.clear_interrupt();
            // pin.set_high();
        }
    }
}
/// Driver for the GOODIX GT911 touch screen controller
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GT911<I2C, IRQ> {
    /// Underlying I²C peripheral
    i2c: I2C,
    /// Interrupt pin
    int: Option<IRQ>,
    /// Address
    ///
    /// Address is determined on start up, so we need to handle two variants:
    /// 0x14 and 0x5d
    addr: Address,
}

impl<I2C, IRQ> GT911<I2C, IRQ>
where
    I2C: I2c,
    IRQ: FlexPin,
{
    /// Creates a new instance of the driver and initialize the device
    pub fn new(
        i2c: I2C,
        // TODO check if we really need to own this or even need this for the reset method.
        int: Option<IRQ>,
        rst: &mut impl FlexPin,
        delay: &mut impl embedded_hal::delay::DelayNs,
        // TODO Optional config instead that also takes resolution and rotation?
        addr: Address,
    ) -> Result<GT911<I2C, IRQ>, Error<<I2C as ErrorType>::Error>> {
        let mut this = Self { i2c, int, addr };
        this.reset(rst, delay)?;
        this.write(GT911_REG_COMMAND, &[Command::ReadCoordinatesStatus as u8])?;
        // TODO: read track id/screen resolution to make sure we are
        // talking to the correct device.
        Ok(this)
    }

    /// Returns `true` if new data is available for reading
    // TODO this now only seems useful for polling. if used in interrupt, the
    // interrupt keeps interrupting
    // TODO try to reset / unlisten from
    // interrupts? but then how to relisten again. this stuff needs a read_handler
    // TODO we could get rid of data_available? but it seems useful for polling if
    // we don't listen to interrupts, then the touch controller seems to drive the
    // int pin correctly
    // TODO the problem is that we need to call reset when data_available == true
    // and buffer_status = 0, even though the spec said we only need to reset on
    // buffer_status = 1.
    pub fn data_available(&mut self) -> Result<bool, Error<<I2C as ErrorType>::Error>>
    where
        IRQ: InputPin,
    {
        if let Some(pin) = self.int.as_mut() {
            Ok(pin.is_low().map_err(|_| Error::IOError)?)
        } else {
            Ok(true)
        }
    }

    /// Reads config from device
    pub fn read_config(&mut self) -> Result<Config, Error<<I2C as ErrorType>::Error>> {
        let mut config = [0; size_of::<Config>()];
        self.read_config_into(&mut config)?;

        Ok(zerocopy::transmute!(config))
    }

    /// Reads config from device
    ///
    /// Like [Self::read_config] but reads the result into the provided buffer.
    pub fn read_config_into<'a>(
        &mut self,
        config: &'a mut [u8; size_of::<Config>()],
    ) -> Result<&'a Config, Error<<I2C as ErrorType>::Error>> {
        self.read(GT911_REG_CONFIG, config)?;
        Ok(
            Config::ref_from(config)
                .expect("any u8 array with correct length to be a valid config"),
        )
    }

    /// Writes config to the device
    ///
    /// The version field of the [Config] needs to be bigger than the device
    /// config version, otherwise the config will not be updated properly.
    /// 
    // TODO check that config has been applied
    pub fn write_config(
        &mut self,
        config: &mut Config,
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        config.finalize();

        self.write(GT911_REG_CONFIG, config.as_bytes())?;
        Ok(())
    }

    /// Returns a [ReadHandle] for reading from the device
    ///
    /// Read the status of the touch data from the `0x814e` register and return
    /// a [ReadHandler] for reading the registered data.
    pub fn read_handle(
        &mut self,
    ) -> Result<ReadHandle<I2C, IRQ>, Error<<I2C as ErrorType>::Error>> {
        let buf = [0; 1];
        self.read_handle_into(buf)
    }

    /// Returns a [ReadHandle] for reading from the device
    ///
    /// Like [Self::read_handle_mut] but reads the result into the provided
    /// buffer.
    pub fn read_handle_into(
        &mut self,
        buf: [u8; 1],
    ) -> Result<ReadHandle<I2C, IRQ>, Error<<I2C as ErrorType>::Error>> {
        let touch_status = self.read_touch_status_into(buf)?;

        Ok(ReadHandle {
            status: touch_status,
            driver: self,
        })
    }

    /// Returns the product id
    pub fn product_id(&mut self) -> Result<u32, Error<<I2C as ErrorType>::Error>> {
        let rx = [0; 4];
        self.product_id_into(rx)
    }

    /// Returns the product id
    ///
    /// Like [Self::product_id] but reads the result into the provided buffer.
    pub fn product_id_into(
        &mut self,
        mut rx: [u8; 4],
    ) -> Result<u32, Error<<I2C as ErrorType>::Error>> {
        self.read(GT911_REG_PROD_ID, &mut rx)?;
        let id = u32::from_le_bytes(rx);

        Ok(id)
    }

    /// Returns the firmware version
    // TODO: This might not be accurate, its supposed to return 1060 for my device
    // but now returns 4192.
    pub fn firmware_version(&mut self) -> Result<u16, Error<<I2C as ErrorType>::Error>> {
        let rx = [0; 2];
        self.firmware_version_into(rx)
    }

    /// Returns the firmware version
    ///
    /// Like [Self::firmware_version] but reads the result into the provided
    /// buffer.
    pub fn firmware_version_into(
        &mut self,
        mut rx: [u8; 2],
    ) -> Result<u16, Error<<I2C as ErrorType>::Error>> {
        self.read(GT911_REG_FIRM_VER, &mut rx)?;
        let version = u16::from_le_bytes(rx);
        Ok(version)
    }

    /// Releases the owned bus and pin
    pub fn release(self) -> (I2C, Option<IRQ>) {
        (self.i2c, self.int)
    }

    // -----------------------------------------------------------------------
    // PRIVATE
    /// Reads from device starting at register `reg` until `rx_buf` is full
    ///
    /// See section `2.2 Timing for Read Operation` of the GT911 Programming
    /// Guide v.0.1.
    fn read(
        &mut self,
        reg: u16,
        rx_buf: &mut [u8],
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        self.i2c
            .write_read(self.addr.into(), &reg.to_be_bytes(), rx_buf)
            .map_err(Error::BusError)
    }

    fn read_touch_status_into(
        &mut self,
        mut buf: [u8; 1],
    ) -> Result<TouchStatus, Error<<I2C as ErrorType>::Error>> {
        self.read(GT911_REG_TOUCH_STATUS, &mut buf)?;
        let touch_status = TouchStatus::from_bits(buf[0]);
        Ok(touch_status)
    }

    /// Writes all bytes from `tx_buf` to device starting at `reg`
    ///
    /// See section `2.1 Timing for Write Operation` of the GT911 Programming
    /// Guide v.0.1.
    fn write(&mut self, reg: u16, tx_buf: &[u8]) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        // TODO No idea why this is necessary
        let reg = reg - 1;
        self.i2c
            .transaction(
                self.addr.into(),
                &mut [
                    Operation::Write(&reg.to_be_bytes()[..]),
                    Operation::Write(tx_buf),
                ],
            )
            .map_err(Error::BusError)?;
        Ok(())
    }

    /// Reset the device
    ///
    /// This must happen at initialization of the device.
    ///
    /// There is various examples of cpp drivers, however, this one is the only
    /// one I found that seemed to work (somewhat) with my setup:
    ///
    /// https://github.com/blackketter/GT911/blob/dcadd293d964f397bc8fb81f867d58f08531d729/src/GT911.cpp#L42
    fn reset(
        &mut self,
        rst: &mut impl FlexPin,
        delay: &mut impl embedded_hal::delay::DelayNs,
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        if let Some(pin) = self.int.as_mut() {
            pin.set_as_output();
            pin.set_low().map_err(|_| Error::IOError)?;
        }
        rst.set_as_output();
        rst.set_low().map_err(|_| Error::IOError)?;

        // BEGIN select i2c slave addr
        // TODO address select doesn't work
        // T2 > 10ms
        delay.delay_ms(11);

        if let Some(pin) = self.int.as_mut() {
            pin.set_state((matches!(self.addr, Address::Two)).into())
                .map_err(|_| Error::IOError)?;
        }
        // T7 > 100us
        delay.delay_us(110);
        rst.set_as_input();
        // TODO this is documented but doesn't work. If commented in, the device is not
        // found on either address.

        //rst.set_high().map_err(|_| Error::IOError)?;
        // T8 > 5ms
        delay.delay_ms(6);
        if let Some(pin) = self.int.as_mut() {
            pin.set_low().map_err(|_| Error::IOError)?;
        }
        // T3 > 50ms
        delay.delay_ms(51);
        if let Some(pin) = self.int.as_mut() {
            pin.set_as_input();
            pin.listen_on_rising_edge();
        }

        Ok(())
    }

    /// Resets the `0x814E` register
    ///
    /// This must be invoked after the host read touch data from the `GT911`, to
    /// inform the `GT911` that it should continue registering new touch data.
    fn reset_touch_status(&mut self) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        self.write(GT911_REG_TOUCH_STATUS, &[0])?;
        Ok(())
    }
}

/// Represents the Command Register (0x8040).
///
/// See section `3.1 Register Map` of the GT911 Programming Guide v.0.1.
#[repr(u8)]
enum Command {
    ReadCoordinatesStatus = 0x00,
}
