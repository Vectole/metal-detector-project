use embedded_hal::{digital::v2::OutputPin, PwmPin, adc::{OneShot, Channel}};
use stm32f1xx_hal::{prelude::*, delay::Delay};
use void::Void;

/// Contains a PWM pin and the maximum allowed duty cycle.
pub struct ModulatedOutput<'a> {
    pub pin: &'a mut dyn PwmPin<Duty = u16>,
    pub max_value: u16,
}

impl ModulatedOutput<'_> {
    pub fn pulse(&mut self, measurement: &Measurement) {
        self.pin.set_duty(map(
            measurement.data.value,
            measurement.range_min,
            measurement.range_max,
            0,
            self.max_value,
            false,
        ));
    }
}

/// Controls the inductor.
pub struct InductorController<'a> {
    pub pin: &'a mut dyn OutputPin<Error = Void>,
}

impl InductorController<'_> {
    /// Pulses the inductor for given time in microseconds (μs).
    pub fn pulse_for(&mut self, delay_provider: &mut Delay, pulse_width: u16) {
        self.pin.set_high();
        delay_provider.delay_us(pulse_width);
        self.pin.set_low();
    }
}

/// Contains the measurement samples and the calculated output value (average) of the sampels.
pub struct MeasurementData {
    pub samples: [u16; 15],
    pub value: u16,
    pub current_index: usize,
}

impl MeasurementData {
    /// Reads and adds a sample to internal array, calculates average value.
    pub fn update(&mut self, input: u16) {
        self.samples[self.current_index] = input;
        self.value = self.samples.iter().sum::<u16>() / self.samples.len() as u16;

        self.current_index += 1;
        if self.current_index == self.samples.len() {
            self.current_index = 0;
        }
    }
}

/// Contains the measurement and its range.
pub struct Measurement {
    pub data: MeasurementData,
    pub range_min: u16,
    pub range_max: u16,
}

/// Controls the measurement taking ADC pin.
pub struct MeasurementTaker<'a, ADC, PIN>
where
    PIN: Channel<ADC, ID = u8>,
{
    pub adc: &'a mut dyn OneShot<ADC, u16, PIN, Error = ()>,
    pub pin: PIN,
    pub output: Measurement,
}

impl<'a, ADC, PIN> MeasurementTaker<'_, ADC, PIN>
where
    PIN: Channel<ADC, ID = u8>,
{
    /// Reads the adc pin value after given time in microseconds (μs).
    pub fn read_after_waiting(&mut self, delay_provider: &mut Delay, wait_time: u16) {
        delay_provider.delay_us(wait_time);
        self.output.data.update(self.adc.read(&mut self.pin).unwrap());
    }
}

/// Controls the 3-digit 7(8) - segment display using the HC595 chip.
pub struct DisplayController<'a> {
    pub eight_segment_display_pin1: &'a mut dyn OutputPin<Error = Void>,
    pub eight_segment_display_pin2: &'a mut dyn OutputPin<Error = Void>,
    pub eight_segment_display_pin3: &'a mut dyn OutputPin<Error = Void>,
    pub shift_register_data_pin: &'a mut dyn OutputPin<Error = Void>,
    pub shift_register_data_clock_pin: &'a mut dyn OutputPin<Error = Void>,
    pub shift_register_storage_clock_pin: &'a mut dyn OutputPin<Error = Void>,
    pub shift_register_enable_pin: &'a mut dyn OutputPin<Error = Void>,
    pub shift_register_clear_pin: &'a mut dyn OutputPin<Error = Void>,
    pub current_display: u8,
    pub current_tick: u8,
    pub current_number: u16,
    pub digit_bytes: [u8; 10],
}

impl DisplayController<'_> {
    /// Controls the ŌĒ pin of the chip, clears the storage register on enable.
    pub fn set_enabled(&mut self, enable: bool) {
        // Active LOW
        if enable {
            self.shift_register_enable_pin.set_low();
            self.shift_register_clear_pin.set_low();
            self.shift_register_clear_pin.set_high();
            self.shift_register_data_clock_pin.set_low();
        } else {
            self.shift_register_enable_pin.set_high();
        }
    }

    /// Displays the input measurement as a 3 digit number.
    pub fn display_data(&mut self, measurement: &Measurement) {
        if self.current_tick == 0 {
            self.current_number = map(
                measurement.data.value,
                measurement.range_min,
                measurement.range_max,
                0,
                999,
                false,
            );
            self.current_tick = 25;
        } else {
            self.current_tick -= 1;
        }
        let mut displayed_digit: u16 = 0;
        match self.current_display {
            0 => {
                self.eight_segment_display_pin1.set_high();
                self.eight_segment_display_pin2.set_low();
                self.eight_segment_display_pin3.set_low();
                displayed_digit = self.current_number / 100;
                self.current_display += 1;
            }
            1 => {
                self.eight_segment_display_pin1.set_low();
                self.eight_segment_display_pin2.set_high();
                self.eight_segment_display_pin3.set_low();
                displayed_digit = (self.current_number / 10) % 10;
                self.current_display += 1;
            }
            2 => {
                self.eight_segment_display_pin1.set_low();
                self.eight_segment_display_pin2.set_low();
                self.eight_segment_display_pin3.set_high();
                displayed_digit = self.current_number % 10;
                self.current_display = 0;
            }
            _ => {}
        }

        for i in 0..8 {
            let value = !self.digit_bytes[displayed_digit as usize] >> i & 0b0000_0001;
            if value == 0 {
                self.shift_register_data_pin.set_low();
            } else {
                self.shift_register_data_pin.set_high();
            }
            self.shift_register_data_clock_pin.set_high();
            self.shift_register_data_clock_pin.set_low();
        }
        self.shift_register_storage_clock_pin.set_high();
        self.shift_register_storage_clock_pin.set_low();
    }
}

/// Maps the input value from a given range to a new output range.
///
/// Output can be reversed to map from max to min.
pub fn map(
    value: u16,
    input_range_min: u16,
    input_range_max: u16,
    output_range_min: u16,
    output_range_max: u16,
    reverse: bool,
) -> u16 {
    let buffer: u32;
    if value >= input_range_max {
        buffer = output_range_max as u32;
    } else if value <= input_range_min {
        buffer = output_range_min as u32;
    } else {
        buffer = (output_range_max - output_range_min) as u32 * (value - input_range_min) as u32
            / (input_range_max - input_range_min) as u32
            + output_range_min as u32;
    }
    if reverse {
        output_range_max - buffer as u16
    } else {
        buffer as u16
    }
}