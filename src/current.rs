use embedded_hal::adc::{Channel, OneShot};
use toyfoc::current::InlineCurrentSensor;

const ADC_VOLT: f32 = 3.0;
const ADC_MAX: u32 = 4095;
pub const SHUNT_RESISTOR: f32 = 0.03;
const AMP_GAIN: f32 = 50.0;
const _1_SQRT3: f32 = 0.57735026919;
const _2_SQRT3: f32 = 1.15470053838;

pub trait RpCurrentSensorTrait<ADC, UP, VP, WP> {
    fn new(adc_handle: ADC, u_pin: Option<UP>, v_pin: Option<VP>, w_pin: Option<WP>) -> Self;
}

impl<ADC, UP, VP, WP> RpCurrentSensorTrait<ADC, UP, VP, WP> for InlineCurrentSensor<ADC, UP, VP, WP>
where
    UP: Channel<ADC>,
    VP: Channel<ADC>,
    WP: Channel<ADC>,
    ADC: OneShot<ADC, u32, UP> + OneShot<ADC, u32, VP> + OneShot<ADC, u32, WP>,
{
    fn new(adc_handle: ADC, u_pin: Option<UP>, v_pin: Option<VP>, w_pin: Option<WP>) -> Self {
        InlineCurrentSensor {
            adc_handle,
            adc_u_pin: u_pin,
            adc_v_pin: v_pin,
            adc_w_pin: w_pin,
            iu: 0.0,
            iv: 0.0,
            iw: 0.0,
            id: 0.0,
            iq: 0.0,
            offset_adc_u: 0.0,
            offset_adc_v: 0.0,
            offset_adc_w: 0.0,
            gain_u: 1.0 / AMP_GAIN / SHUNT_RESISTOR,
            gain_v: 1.0 / AMP_GAIN / SHUNT_RESISTOR,
            gain_w: 1.0 / AMP_GAIN / SHUNT_RESISTOR,
            adc_conversion_factor: ADC_VOLT / ADC_MAX as f32,
        }
    }
}
