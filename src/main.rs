#![no_std]
#![no_main]

mod ctrl;
mod current;
mod pwms;

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use crate::{
    current::RpCurrentSensorTrait,
    pwms::{RpPWMsTrait, PWM_TOP},
};

use rp2040_hal as hal;

use hal::clocks::Clock;
use hal::entry;
use hal::pac;
use hal::pio::PIOExt;

use core::iter::once;
use cortex_m::delay::Delay;
use hal::adc::AdcPin;
use hal::gpio::FunctionSpi;
use hal::Spi;
use mt6701::MT6701Spi;
use toyfoc::{current::InlineCurrentSensor, none::NONE_ADC_PIN, pwms::PWMs, ToyFOC};

use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

use fugit::RateExtU32;

use toyfoc::LoopMode;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

const I2C_ADDR: u16 = 0x68;

#[entry]
fn main() -> ! {
    info!("ToyFOC start!");

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.gpio20.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let i2c = hal::I2C::new_peripheral_event_iterator(
        pac.I2C0,
        pins.gpio12.reconfigure(),
        pins.gpio13.reconfigure(),
        &mut pac.RESETS,
        I2C_ADDR,
    );
    let mut i2c_ctrl = ctrl::I2Ctrl::new(i2c);

    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    pwm_slices.pwm3.set_ph_correct();
    pwm_slices.pwm3.set_div_int(1);
    pwm_slices.pwm3.set_div_frac(0);
    pwm_slices.pwm3.set_top(PWM_TOP);
    pwm_slices.pwm3.enable();

    let mut pwm_u = pwm_slices.pwm3.channel_a;
    pwm_u.output_to(pins.gpio6);

    pwm_slices.pwm4.set_ph_correct();
    pwm_slices.pwm4.set_div_int(1);
    pwm_slices.pwm4.set_div_frac(0);
    pwm_slices.pwm4.set_top(PWM_TOP);
    pwm_slices.pwm4.enable();

    let mut pwm_v = pwm_slices.pwm4.channel_a;
    pwm_v.output_to(pins.gpio8);

    pwm_slices.pwm5.set_ph_correct();
    pwm_slices.pwm5.set_div_int(1);
    pwm_slices.pwm5.set_div_frac(0);
    pwm_slices.pwm5.set_top(PWM_TOP);
    pwm_slices.pwm5.enable();
    let mut pwm_w = pwm_slices.pwm5.channel_a;
    pwm_w.output_to(pins.gpio10);

    let en_pin = pins.gpio4.into_push_pull_output();
    let foc_pwms = PWMs::new(pwm_u, pwm_v, pwm_w, en_pin);

    let spi = Spi::<_, _, _, 16>::new(
        pac.SPI0,
        (
            pins.gpio19.into_function::<FunctionSpi>(),
            pins.gpio16.into_function::<FunctionSpi>(),
            pins.gpio18.into_function::<FunctionSpi>(),
        ),
    )
    .init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        1.MHz(),
        &embedded_hal::spi::MODE_0,
    );
    let csn_pin = pins.gpio17.into_push_pull_output();
    let sensor = MT6701Spi::new(spi, csn_pin);

    // Enable ADC
    let adc: hal::Adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let adc_u = AdcPin::new(pins.gpio26);
    let adc_v = AdcPin::new(pins.gpio27);
    // let adc_w = AdcPin::new(pins.gpio28);

    let current_sensor = InlineCurrentSensor::new(adc, Some(adc_u), Some(adc_v), NONE_ADC_PIN);

    let _voltage_main = 0.0;

    let mut foc = ToyFOC::new(foc_pwms);

    foc.sensor = Some(sensor);
    foc.current_sensor = Some(current_sensor);
    // foc.sensor = NONE_SENSOR;
    // foc.current_sensor = NONE_CURRENT_SENSOR;

    foc.voltage_power = 5.0;
    foc.voltage_limit = 1.5;
    foc.phase_resistance = 6.0;

    foc.torque.p = 1.;
    foc.torque.i = 0.1;
    foc.torque.d = 0.0;

    foc.velocity.p = 0.01;
    foc.velocity.i = 1.5;
    foc.velocity.d = 0.0;

    foc.position.p = 30.;
    foc.position.i = 0.01;
    foc.position.d = 0.0;

    foc.torque_limit = 0.15;
    foc.velocity_limit = 2.;

    foc.init(delay, 2000).unwrap();

    foc.use_svpwm = true;
    foc.mode = LoopMode::Position;
    foc.target = 10.0;

    loop {
        let now_us = timer.get_counter().ticks();

        foc.run(now_us, i2c_ctrl.cmd).unwrap();

        i2c_ctrl.exchange(foc.get_states());

        if let Some(_) = foc.sensor {
            let color: RGB8 = (50, 100, 150).into();
            let bn = foc.angle_val as u8 * 5 + 1;
            ws.write(brightness(once(color), bn)).unwrap();
        }
    }
}
