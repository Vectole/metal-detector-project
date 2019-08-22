#![no_std]
#![no_main]
#![allow(unused_must_use)]

/// Microcontroller Datasheet:
/// https://www.st.com/resource/en/datasheet/stm32f103c8.pdf
extern crate panic_halt;
#[macro_use()]
use cortex_m_rt::entry;
#[macro_use(stm32f1xx_hal::gpio)]
use stm32f1xx_hal::{adc::Adc, pac, prelude::*, delay::Delay};

mod metal_detector;
use metal_detector::*;

#[entry]

fn main() -> ! {
    let core_peripherals = cortex_m::Peripherals::take().unwrap(); // all generic Cortex peripherals
    let device_peripherals = pac::Peripherals::take().unwrap(); // all peripherals specific to stm32f1xx series
    let mut flash = device_peripherals.FLASH.constrain();
    let mut rcc = device_peripherals.RCC.constrain(); // reset & clock control

    let mut afio = device_peripherals.AFIO.constrain(&mut rcc.apb2);
    let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);
    let mut delay_provider = Delay::new(core_peripherals.SYST, clocks);

    let mut gpioa = device_peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = device_peripherals.GPIOB.split(&mut rcc.apb2);

    let mut measurement_taker = MeasurementTaker {
        adc: &mut Adc::adc1(device_peripherals.ADC1, &mut rcc.apb2, clocks),
        pin: gpioa.pa5.into_analog(&mut gpioa.crl),
        output: Measurement {
            data: MeasurementData {
                samples: [0; 15],
                value: 0,
                current_index: 0,
            },
            range_min: 2040,
            range_max: 2460,
        },
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

    let mut bar_graph_controller = ModulatedOutput {
        max_value: pwm_channel1.get_max_duty() / 3,
        pin: &mut pwm_channel1,
    };

    let mut voltmeter_controller = ModulatedOutput {
        max_value: pwm_channel3.get_max_duty(),
        pin: &mut pwm_channel3,
    };

    let mut speaker_controller = ModulatedOutput {
        max_value: pwm_channel4.get_max_duty() / 5,
        pin: &mut pwm_channel4,
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
        digit_bytes: [
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
        ],
    };
    display_controller.set_enabled(true);

    loop {
        inductor_controller.pulse_for(&mut delay_provider, 700);
        measurement_taker.read_after_waiting(&mut delay_provider, 40);

        bar_graph_controller.pulse(&measurement_taker.output);
        voltmeter_controller.pulse(&measurement_taker.output);
        speaker_controller.pulse(&measurement_taker.output);
        display_controller.display_data(&measurement_taker.output);

        delay_provider.delay_us(6666 as u32);
    }
}