use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use esp32s3_hal::gpio::Event;
use esp32s3_hal::gpio::Flex as Flex_;
use esp32s3_hal::gpio::Pull;
use esp32s3_hal::peripheral::Peripheral;
use esp_backtrace as _;

use crate::FlexPin;

/// Flexible GPIO pin usable as input and output
pub struct Flex<'d, P>(Flex_<'d, P>);

impl<'d, P: esp32s3_hal::gpio::InputPin + esp32s3_hal::gpio::OutputPin> Flex<'d, P> {
    pub fn new(pin: impl Peripheral<P = P> + 'd) -> Self {
        Self(Flex_::new(pin))
    }
}

impl<'d, P> embedded_hal::digital::ErrorType for Flex<'d, P> {
    type Error = embedded_hal::digital::ErrorKind;
}

impl<'d, P: esp32s3_hal::gpio::InputPin + esp32s3_hal::gpio::OutputPin> FlexPin for Flex<'d, P> {
    fn set_as_input(&mut self) {
        self.0.set_as_input(Pull::None)
    }

    fn set_as_output(&mut self) {
        self.0.set_as_output()
    }

    fn listen_on_rising_edge(&mut self) {
        // TODO we don't want this in polling mode for the data_available method
        // to work

        // self.0.listen(Event::RisingEdge);
    }

    fn clear_interrupt(&mut self) {
        self.0.clear_interrupt()
    }
}

impl<'d, P: esp32s3_hal::gpio::InputPin + esp32s3_hal::gpio::OutputPin> InputPin for Flex<'d, P> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_low())
    }
}

impl<'d, P: esp32s3_hal::gpio::InputPin + esp32s3_hal::gpio::OutputPin> OutputPin for Flex<'d, P> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_high();
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_low();
        Ok(())
    }
}
