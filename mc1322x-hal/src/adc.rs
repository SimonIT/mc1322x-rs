use mc1322x_sys::{ADC_READ, ADC_flush, adc_init, adc_setup_chan};

/// Internal 1.2 V reference used to convert raw readings to millivolts.
const INTERNAL_REFERENCE_MV: u32 = 1200;

/// Per-unit offset applied to the battery reading (see `adc.h`).
const ADC_VBATT_TRIM: u32 = 183;

/// Number of ADC channels: GPIO0-7 plus the internal reference on channel 8.
pub const ADC_CHANNELS: u8 = 9;

/// MC1322x 12-bit ADC.
///
/// The ADC runs in automatic mode: every enabled channel plus the internal
/// reference is converted in a repeating sequence and pushed to the FIFO,
/// tagged with its channel number in the upper four bits. Sampling is started
/// and calibrated by [`Adc::new`]; [`Adc::read`] then blocks until a fresh
/// sample tagged with the requested channel arrives.
///
/// Channel 8 measures the internal 1.2 V reference against the ADC reference
/// rail and is always enabled; it is the denominator for the millivolt
/// conversions in [`Adc::voltage`] and [`Adc::battery_voltage`].
pub struct Adc;

impl Adc {
    /// Power up and calibrate the ADC.
    ///
    /// Enables the ADC clock and timings and starts the conversion sequence
    /// (channel 8 / internal reference included).
    pub fn new() -> Self {
        unsafe { adc_init() }
        Adc
    }

    /// Include `channel` (0-7) in the conversion sequence and mux its GPIO pad
    /// to the ADC. The internal reference on channel 8 is always enabled.
    pub fn enable_channel(&mut self, channel: u8) {
        assert!(channel < 8, "ADC channel {} out of range 0-7", channel);
        unsafe { adc_setup_chan(channel) }
    }

    /// Read one raw 12-bit sample (0-4095) from `channel`.
    ///
    /// Blocks until a sample tagged with `channel` appears in the FIFO,
    /// discarding samples from other channels. The FIFO is flushed first so
    /// the returned value is fresh.
    pub fn read(&mut self, channel: u8) -> u16 {
        unsafe { ADC_flush() }
        loop {
            let sample = unsafe { ADC_READ() };
            if (sample >> 12) as u8 == channel {
                return sample & 0x0FFF;
            }
        }
    }

    /// Read `channel` (0-8) in millivolts, referenced to the internal 1.2 V
    /// reference. Requires the reference on channel 8 to be enabled, which
    /// [`Adc::new`] does.
    pub fn voltage(&mut self, channel: u8) -> u32 {
        let reference = self.read(8) as u32;
        (self.read(channel) as u32) * INTERNAL_REFERENCE_MV / reference
    }

    /// Battery voltage in millivolts, derived from the full-scale ADC reading
    /// and the internal reference, with the default per-unit trim applied.
    pub fn battery_voltage(&mut self) -> u32 {
        let reference = self.read(8) as u32;
        (4095 * INTERNAL_REFERENCE_MV / reference) + ADC_VBATT_TRIM
    }
}

impl Default for Adc {
    fn default() -> Self {
        Self::new()
    }
}
