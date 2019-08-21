#![no_std]
#![no_main]
#![allow(unused_must_use)]

/// Microcontroller Datasheet:
/// https://www.st.com/resource/en/datasheet/stm32f103c8.pdf
extern crate panic_halt;
#[macro_use()]
use cortex_m_rt::entry;
use embedded_hal::{digital::v2::OutputPin, PwmPin};
#[macro_use(stm32f1xx_hal::gpio)]
use stm32f1xx_hal::{adc::Adc, pac, prelude::*, delay::Delay};
use void::Void;

#[entry]

fn main() -> ! {
    let core_peripherals = cortex_m::Peripherals::take().unwrap(); // all generic Cortex peripherals
    let device_peripherals = pac::Peripherals::take().unwrap(); // all peripherals specific to stm32f1xx series
    let mut flash = device_peripherals.FLASH.constrain();
    let mut rcc = device_peripherals.RCC.constrain(); // reset & clock control

    let mut afio = device_peripherals.AFIO.constrain(&mut rcc.apb2);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay_provider = Delay::new(core_peripherals.SYST, clocks);

    let mut gpioa = device_peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = device_peripherals.GPIOB.split(&mut rcc.apb2);
    //let mut gpioc = device_peripherals.GPIOC.split(&mut rcc.apb2);

    let mut measurement_taker = MeasurementTaker {
        adc: &mut Adc::adc1(device_peripherals.ADC1, &mut rcc.apb2, clocks),
        pin: &mut gpioa.pa5.into_analog(&mut gpioa.crl),
        output: 0,
    };

    let pwm_pins_1 = (
        gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl),
        gpiob.pb7.into_alternate_push_pull(&mut gpiob.crl),
        gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh),
        gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh),
    );

    let (mut pwm_channel1, _, mut pwm_channel3, mut pwm_channel4) =
        device_peripherals
            .TIM4
            .pwm(pwm_pins_1, &mut afio.mapr, 1.khz(), clocks, &mut rcc.apb1);

    pwm_channel1.enable();
    pwm_channel3.enable();
    pwm_channel4.enable();

    // TODO: find the values for the outputs
    let mut bar_graph_controller = ModulatedOutput {
        pin: &mut pwm_channel1,
        minimum_value: 1500,
    };

    let mut voltmeter_controller = ModulatedOutput {
        pin: &mut pwm_channel3,
        minimum_value: 3000,
    };

    let mut speaker_controller = ModulatedOutput {
        pin: &mut pwm_channel4,
        minimum_value: 160,
    };

    let mut inductor_controller = InductorController {
        pin: &mut gpiob.pb1.into_push_pull_output(&mut gpiob.crl),
    };

    let mut display_controller = DisplayController {
        eight_segment_display_pin1: &mut gpioa.pa6.into_push_pull_output(&mut gpioa.crl),
        eight_segment_display_pin2: &mut gpioa.pa7.into_push_pull_output(&mut gpioa.crl),
        eight_segment_display_pin3: &mut gpiob.pb0.into_push_pull_output(&mut gpiob.crl),
        shift_register_data_pin: &mut gpioa.pa0.into_push_pull_output(&mut gpioa.crl),
        shift_register_data_clock_pin: &mut gpioa.pa3.into_push_pull_output(&mut gpioa.crl),
        shift_register_storage_clock_pin: &mut gpioa.pa2.into_push_pull_output(&mut gpioa.crl),
        shift_register_enable_pin: &mut gpioa.pa1.into_push_pull_output(&mut gpioa.crl),
        shift_register_clear_pin: &mut gpioa.pa4.into_push_pull_output(&mut gpioa.crl),
        current_display: 0,
        current_tick: 0,
        current_number: 0,
        digit_bytes: &DIGIT_BYTES,
    };
    display_controller.set_enabled(true);

    loop {
        inductor_controller.pulse_for(&mut delay_provider, 700);
        measurement_taker.read_after_waiting(&mut delay_provider, 40);

        voltmeter_controller.pulse(measurement_taker.output);
        display_controller.display_number(measurement_taker.output);
        speaker_controller.pulse(measurement_taker.output);
        bar_graph_controller.pulse(measurement_taker.output);
        delay_provider.delay_us(6666 as u32);
    }
}

fn set_pwm<T: ?Sized>(pin: &mut T, duty: u16, starting_point: u16)
where
    T: PwmPin<Duty = u16>,
{
    let fill = map(duty, 0, 4095, 0, starting_point, false);
    pin.set_duty(fill);
}

struct ModulatedOutput<'a> {
    pin: &'a mut dyn PwmPin<Duty = u16>,
    minimum_value: u16,
}

impl ModulatedOutput<'_> {
    fn pulse(&mut self, duty: u16) {
        set_pwm(self.pin, duty, self.minimum_value);
    }
}

struct InductorController<'a> {
    pin: &'a mut dyn OutputPin<Error = Void>,
}

impl InductorController<'_> {
    fn pulse_for(&mut self, delay_provider: &mut Delay, pulse_width: u16) {
        self.pin.set_high();
        delay_provider.delay_us(pulse_width);
        self.pin.set_low();
    }
}

struct MeasurementTaker<'a> {
    adc: &'a mut Adc<pac::ADC1>,
    // TODO: find a way to make this more generic:
    pin: &'a mut stm32f1xx_hal::gpio::gpioa::PA5<stm32f1xx_hal::gpio::Analog>,
    output: u16,
}

impl MeasurementTaker<'_> {
    fn read_after_waiting(&mut self, delay_provider: &mut Delay, wait_time: u16) {
        delay_provider.delay_us(wait_time);
        let mut output: u16 = self.adc.read(self.pin).unwrap();
        output = map(output, 2036, 2654, 0, 4095, false);
        self.output = output;
    }
}

const DIGIT_BYTES: [u8; 10] = [
    0b10111110, //0
    0b10000010, //1
    0b11101100, //2
    0b11100110, //3
    0b11010010, //4
    0b01110110, //5
    0b01111110, //6
    0b10100010, //7
    0b11111110, //8
    0b11110110, //9
];

struct DisplayController<'a> {
    eight_segment_display_pin1: &'a mut dyn OutputPin<Error = Void>,
    eight_segment_display_pin2: &'a mut dyn OutputPin<Error = Void>,
    eight_segment_display_pin3: &'a mut dyn OutputPin<Error = Void>,
    shift_register_data_pin: &'a mut dyn OutputPin<Error = Void>,
    shift_register_data_clock_pin: &'a mut dyn OutputPin<Error = Void>,
    shift_register_storage_clock_pin: &'a mut dyn OutputPin<Error = Void>,
    shift_register_enable_pin: &'a mut dyn OutputPin<Error = Void>,
    shift_register_clear_pin: &'a mut dyn OutputPin<Error = Void>,
    current_display: u8,
    current_tick: u8,
    current_number: u16,
    digit_bytes: &'static [u8; 10],
}

impl DisplayController<'_> {
    fn set_enabled(&mut self, enable: bool) {
        // Active LOW (ŌĒ)
        if enable {
            self.shift_register_enable_pin.set_low();
            self.shift_register_clear_pin.set_low();
            self.shift_register_clear_pin.set_high();
            self.shift_register_data_clock_pin.set_low();
        } else {
            self.shift_register_enable_pin.set_high();
        }
    }
    fn display_number(&mut self, input_value: u16) {
        if self.current_tick == 0 {
            self.current_number = map(input_value, 0, 4095, 0, 999, false);
            self.current_tick = 50;
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
            digital_write(
                self.shift_register_data_pin,
                WriteState::Numeric(!self.digit_bytes[displayed_digit as usize] >> i & 0b0000_0001),
            );
            self.shift_register_data_clock_pin.set_high();
            self.shift_register_data_clock_pin.set_low();
        }
        self.shift_register_storage_clock_pin.set_high();
        self.shift_register_storage_clock_pin.set_low();
    }
}

enum WriteState {
    HIGH,
    LOW,
    Numeric(u8),
}

fn digital_write<T: ?Sized>(output_pin: &mut T, state: WriteState)
where
    T: OutputPin<Error = Void>,
{
    match state {
        WriteState::HIGH => {
            output_pin.set_high();
        }
        WriteState::LOW => {
            output_pin.set_low();
        }
        WriteState::Numeric(value) => {
            if value == 0 {
                output_pin.set_low();
            } else {
                output_pin.set_high();
            }
        }
    }
}

fn map(
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
