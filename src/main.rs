#![no_std]
#![no_main]
#![allow(unused_must_use)]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate panic_halt;
//use nb::block;

#[macro_use()]
use cortex_m_rt::entry;
use embedded_hal::{digital::v2::OutputPin, PwmPin};
#[macro_use(stm32f1xx_hal::gpio)]
use stm32f1xx_hal::{adc::Adc, pwm::*, pac, prelude::*, timer::Timer, delay::Delay};

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
    //let mut gpioc = device_peripherals.GPIOC.split(&mut rcc.apb2);

    //let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    //let measurement: u16 = adc1.read(&mut measurement_pin).unwrap();

    let mut measurement_pin = MeasurementPin {
        adc: &mut Adc::adc1(device_peripherals.ADC1, &mut rcc.apb2, clocks),
        pin: &mut gpioa.pa5.into_analog(&mut gpioa.crl),
    };

    let pwm_pins = (
        gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
        gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl),
    );

    let (mut pwm_channel1, mut pwm_channel2, _, _) =
        device_peripherals
            .TIM2
            .pwm(pwm_pins, &mut afio.mapr, 10.khz(), clocks, &mut rcc.apb1);

    //c0.set_duty(c0.get_max_duty() / 2);
    //pulse_voltmeter(&mut c0, 51);
    pwm_channel1.enable();
    pwm_channel2.enable();
    //c0.set_duty(c0.get_max_duty() / 2);
    // alternative timer:
    // let mut timer = Timer::syst(core_peripherals.SYST, 2.hz(), clocks);
    // block!(timer.wait());

    //pulse_voltmeter(&mut c0, 51);

    let mut pwm_pin = FadingPin {
        fill: 255,
        incrementing: false,
        pin: &mut pwm_channel1,
    };
    let mut voltmeter_pin = VoltmeterPin {
        pin: &mut pwm_channel2,
    };
    loop {
        // flash_led(&mut led, &mut delay, 500);
        measurement_pin.read();
        delay_provider.delay_ms(1 as u16);
        pwm_pin.fade();
        voltmeter_pin.pulse(voltmeter_pin.pin.get_max_duty() / 3);
    }
}

fn flash_led<T>(pin: &mut T, delay: &mut Delay, amount: u16)
where
    T: OutputPin,
{
    pin.set_high();
    delay.delay_ms(amount);
    pin.set_low();
    delay.delay_ms(amount);
}

struct FadingPin<'a> {
    fill: u16,
    incrementing: bool,
    pin: &'a mut dyn PwmPin<Duty = u16>,
}

impl FadingPin<'_> {
    fn fade(&mut self) {
        let max_duty = self.pin.get_max_duty();
        if self.fill == max_duty {
            self.incrementing = false;
        } else if self.fill == 1 {
            self.incrementing = true;
        }
        if self.incrementing {
            self.fill += 1;
        } else {
            self.fill -= 1;
        }
        self.pin.set_duty(max_duty - self.fill);
    }
}

fn set_pwm<T: ?Sized>(pin: &mut T, duty: u16, starting_point: u16)
where
    T: PwmPin<Duty = u16>,
{
    let fill = map(duty, 0, 4095, starting_point, 0);
    pin.set_duty(fill);
}

struct VoltmeterPin<'a> {
    pin: &'a mut dyn PwmPin<Duty = u16>,
}

impl VoltmeterPin<'_> {
    fn pulse(&mut self, duty: u16) {
        //set_pwm(self.pin, duty, 255);
        self.pin.set_duty(duty);
        set_pwm(self.pin, duty, 255);
    }
}

struct MeasurementPin<'a> {
    adc: &'a mut Adc<pac::ADC1>,
    // TODO: find a way to make this more generic:
    pin: &'a mut stm32f1xx_hal::gpio::gpioa::PA5<stm32f1xx_hal::gpio::Analog>,
}

impl MeasurementPin<'_> {
    fn read(&mut self) -> u16 {
        self.adc.read(self.pin).unwrap()
    }
}

// fn pulse_voltmeter(pin: &mut Pwm<TIM2, C1>, input_value: u16) {
//     //let voltmeter_value: u16 = map(input_value, 0, 4095, pin.get_max_duty(), 0);
//     let input_range_min = 0;
//     let input_range_max = 4095;
//     let output_range_min = pin.get_max_duty();
//     pin.set_duty(output_range_min * (input_value - input_range_min) / (input_range_max - input_range_min) + output_range_min);
// }

fn map(
    value: u16,
    input_range_min: u16,
    input_range_max: u16,
    output_range_min: u16,
    output_range_max: u16,
) -> u16 {
    (output_range_max - output_range_min) * (value - input_range_min)
        / (input_range_max - input_range_min)
        + output_range_min
}
