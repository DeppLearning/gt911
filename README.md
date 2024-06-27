# GT911

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/depplearning/gt911/ci.yml?label=CI&logo=github&style=flat-square)
[![Crates.io](https://img.shields.io/crates/v/gt911?logo=Rust&style=flat-square)](https://crates.io/crates/gt911)
[![docs.rs](https://img.shields.io/docsrs/gt911?logo=rust&style=flat-square)](https://docs.rs/gt911)
![MSRV](https://img.shields.io/badge/MSRV-1.58-blue?style=flat-square)
![Crates.io](https://img.shields.io/crates/l/gt911?style=flat-square)

An `embedded-hal` driver for the GT911 touchscreen controller.

## Current State

The current state is pretty bare-bones. If you are missing a feature, please open an issue and/or MR.

- [ ] init device
  - [X] wake up
  - [ ] address selection (might work already)
- [X] configure device
- [ ] polling mode: 
  - [X] read touch data
  - [X] read key presses
  - [ ] read gestures
- [ ] interrupt mode (might work already)
- [ ] Alloc API
- [ ] Async API
- [ ] rotation
- [ ] more default configurations for popular development boards
  
## Supported Development Boards

I only have access to a single device using this controller. Other devices probably will work as well, however, probably require different configuration.

If you successfully managed to use this crate with other development boards, then please open a MR to add the configuration you used and add an entry in the list of supported devices.

Supported Devices:

- [ESP32-S3-BOX-3](https://github.com/espressif/esp-box/blob/master/docs/hardware_overview/esp32_s3_box_3/hardware_overview_for_box_3.md)
  - Limitations: 
    - The `ESP32-S3-BOX-3` seems to wire the `GT911` in such a way, that the latter operates on 1.8v logic level. If this is the case, the GT911 would be to weak to drive against the `ESP32-S3`s 3.3v. As a consequence, the `ESP32-S3-BOX-3` might not support interrupt mode fully. Lowering the I2C bus frequency to i.e. 1kHz might help.
    - The address selection procedure does not work and the device seems to be limited to using the `0x5D` address.

## Resources

- [GT911 Datasheet](https://www.crystalfontz.com/controllers/GOODIX/GT911/)
- [GT911 Programming Guide](https://www.crystalfontz.com/controllers/GOODIX/GT911ProgrammingGuide/478/)

## Related Works

- [jessebrahm/tt21100](https://github.com/jessebraham/tt21100)
  - Used as a reference rust touchscreen driver
- [blackketter/gt911](https://github.com/blackketter/GT911)
  - Used as a reference C-implementation for the GT911

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.