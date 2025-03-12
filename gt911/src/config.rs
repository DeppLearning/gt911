use zerocopy::little_endian::U16;
use zerocopy::little_endian::U64;
use zerocopy::AsBytes;
use zerocopy::FromBytes;
use zerocopy::FromZeroes;
use zerocopy::Unaligned;

/// Default config for the ESP32-S3-BOX-3
///
/// Module_switch_1 seems to be configured to falling, instead of rising edge
pub const ESP32_S3_BOX_3_DEFAULT_CONFIG: Config = zerocopy::transmute!([
    0x41u8, 0x40, 0x01, 0xf0, 0x00, 0x05, 0x0d, 0x01, 0x01, 0x08, 0x28, 0x05, 0x50, 0x32, 0x03,
    0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x89, 0x00, 0x06, 0x17,
    0x15, 0x31, 0x0d, 0x00, 0x00, 0x01, 0xba, 0x04, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x64,
    0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00,
    0x00, 0xf7, 0x4a, 0x3a, 0xff, 0xff, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0c, 0x0a, 0x08, 0x06, 0x04, 0x02, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
    0x02, 0x04, 0x06, 0x08, 0x0a, 0x0c, 0x0f, 0x10, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
]);

/// Device configuration
///
/// See section `3.2 Configuration Information` of the GT911 Programming
/// Guide v.0.1.
///
/// Note that this crate does not perform any validation of the config - any
/// byte sequence of the correct length will be accepted as valid configuration
/// data. Writing invalid configuration data to the device might cause
/// irresversible damage, so use with caution.
#[derive(
    Clone,
    Debug,
    FromZeroes,
    FromBytes,
    AsBytes,
    Unaligned,
    CopyGetters,
    MutGetters,
    Setters,
    PartialEq,
    Eq,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C)]
#[getset(get_copy = "pub", get_mut = "pub", set = "pub")]
pub struct Config {
    version: u8,
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    x_output_max: U16,
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    y_output_max: U16,
    touch_number: u8,
    module_switch_1: u8,
    module_switch_2: u8,
    shake_count: u8,
    filter: u8,
    large_touch: u8,
    noise_reduction: u8,
    screen_touch_level: u8,
    screen_leave_level: u8,
    low_power_control: u8,
    refresh_rate: u8,
    x_threshold: u8,
    y_threshold: u8,
    #[getset(skip)]
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    _nc0: U16,
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    space: U16,
    mini_filter: u8,
    stretch_r0: u8,
    stretch_r1: u8,
    stretch_r2: u8,
    stretch_rm: u8,
    drv_group_a_num: u8,
    drv_group_b_num: u8,
    sensor_num: u8,
    freq_a_factor: u8,
    freq_b_factor: u8,
    pannel_bit_freq_l: u8,
    pannel_bit_freq_h: u8,
    pannel_sensor_time_l: u8,
    pannel_sensor_time_h: u8,
    pannel_tx_gain: u8,
    pannel_rx_gain: u8,
    pannel_dump_shift: u8,
    drv_frame_control: u8,
    charging_lvl_up: u8,
    module_switch_3: u8,
    gesture_dis: u8,
    gesture_long_press_time: u8,
    x_y_slope_adjust: u8,
    gesture_control: u8,
    gesture_switch_1: u8,
    gesture_switch_2: u8,
    gesure_refresh_rate: u8,
    gesture_touch_level: u8,
    new_green_wake_up_level: u8,
    freq_hopping_start: u8,
    freq_hopping_end: u8,
    noise_detect_times: u8,
    hopping_flag: u8,
    hopping_threshold: u8,
    noise_threshold: u8,
    noise_min_threshold: u8,
    #[getset(skip)]
    _nc1: u8,
    hopping_sensor_group: u8,
    hopping_seg1_normalize: u8,
    hopping_seg1_factor: u8,
    main_clock_adjust: u8,
    hopping_seg2_normalize: u8,
    hopping_seg2_factor: u8,
    #[getset(skip)]
    _nc2: u8,
    hopping_seg3_normalize: u8,
    hopping_seg3_factor: u8,
    #[getset(skip)]
    _nc3: u8,
    hopping_seg4_normalize: u8,
    hopping_seg4_factor: u8,
    #[getset(skip)]
    _nc4: u8,
    hopping_seg5_normalize: u8,
    hopping_seg5_factor: u8,
    #[getset(skip)]
    _nc5: u8,
    hopping_seg6_normalize: u8,
    key_1: u8,
    key_2: u8,
    key_3: u8,
    key_4: u8,
    key_area: u8,
    key_touch_level: u8,
    key_leave_level: u8,
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    key_sens: U16,
    key_restrain: u8,
    key_restrain_time: u8,
    gesture_large_touch: u8,
    #[getset(skip)]
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    _nc6: U16,
    hotknot_noise_map: u8,
    link_threshold: u8,
    pxy_threshold: u8,
    g_hot_dump_shift: u8,
    g_hot_rx_gain: u8,
    freq_gain_0: u8,
    freq_gain_1: u8,
    freq_gain_2: u8,
    freq_gain_3: u8,
    #[getset(skip)]
    _nc7: [u8; 9],
    combine_dis: u8,
    split_set: u8,
    #[getset(skip)]
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    _nc8: U16,
    /// Sensor channels 0-13
    sensor_channels: [u8; 14],
    #[getset(skip)]
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    _nc9: [U64; 2],
    /// Driver channels 0-25
    driver_channels: [u8; 26],
    #[getset(skip)]
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    _nc10: [U64; 2],
    config_chksum: u8,
    config_fresh: u8,
}

impl Config {
    /// Finalize the config
    ///
    /// This MUST be called after updating a config and before writing the
    /// updated config back to the device. The method will calculate and set
    /// the checksum as well as set the `config_fresh` register at `0x8100`
    /// to `1`, to indicate to the device that the config has been updated by
    /// the host.
    pub fn finalize(&mut self) {
        self.calc_checksum();
        self.mark_updated();
    }

    /// Set the resolution in the config
    pub fn with_resolution(&mut self, x: u16, y: u16) -> &mut Self {
        self.x_output_max = x.into();
        self.y_output_max = y.into();

        self
    }

    /// Calculates the checksum value
    ///
    /// Returns the sum of the bytes from `0x8047` to `0x80FE`
    fn calc_checksum(&mut self) {
        let bytes = self.as_bytes();
        let csum: u8 = bytes
            .iter()
            .take(bytes.len() - 2)
            .fold(0u8, |sum, item| sum.wrapping_add(*item));
        self.config_chksum = (!csum).wrapping_add(1);
    }

    fn mark_updated(&mut self) {
        self.config_fresh = 1;
    }
}

#[cfg(test)]
mod test {
    use zerocopy::AsBytes;

    use crate::config::ESP32_S3_BOX_3_DEFAULT_CONFIG;

    #[test]
    fn parse_config() {
        let bytes = [
            0x5A, 0x20, 0x03, 0xE0, 0x01, 0x05, 0x0D, 0x00, 0x01, 0x08, 0x28, 0x08, 0x50, 0x32,
            0x03, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88,
            0x29, 0x0A, 0x35, 0x37, 0xD3, 0x07, 0x00, 0x00, 0x01, 0x81, 0x02, 0x1D, 0x00, 0x01,
            0x00, 0x00, 0x00, 0x03, 0x64, 0x32, 0x00, 0x00, 0x00, 0x28, 0x5A, 0x94, 0xC5, 0x02,
            0x00, 0x00, 0x00, 0x00, 0x98, 0x2B, 0x00, 0x84, 0x33, 0x00, 0x74, 0x3C, 0x00, 0x67,
            0x46, 0x00, 0x5C, 0x53, 0x00, 0x5C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14, 0xFF, 0xFF, 0xFF, 0xFF,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0F, 0x10, 0x12, 0x16, 0x18, 0x1C, 0x1D,
            0x1E, 0x1F, 0x20, 0x21, 0x22, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x25, 0x01,
        ];
        // let mut config = Config::read_from(&bytes).unwrap();
        let mut config = ESP32_S3_BOX_3_DEFAULT_CONFIG.clone();
        config.finalize();
        println!("{:#04x?}", &config.as_bytes());
        assert_eq!(config.config_chksum(), 0x25);
        assert_eq!(config.as_bytes(), bytes);
    }
}
