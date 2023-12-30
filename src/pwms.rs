use embedded_hal::{digital::v2::OutputPin, PwmPin};
use toyfoc::pwms::PWMs;

// PWM_TOP = 125,000,000 / 2 / feq -1
pub const PWM_TOP: u16 = 2082; //10K=6249,25K=2499,30K=2082,50K=1249

pub trait RpPWMsTrait<CHU, CHV, CHW, P, E>
where
    CHU: PwmPin<Duty = u16>,
    CHV: PwmPin<Duty = u16>,
    CHW: PwmPin<Duty = u16>,
    P: OutputPin<Error = E>,
{
    fn new(pwm_u: CHU, pwm_v: CHV, pwm_w: CHW, enable_pin: P) -> Self;
}

impl<CHU, CHV, CHW, P, E> RpPWMsTrait<CHU, CHV, CHW, P, E> for PWMs<CHU, CHV, CHW, P>
where
    CHU: PwmPin<Duty = u16>,
    CHV: PwmPin<Duty = u16>,
    CHW: PwmPin<Duty = u16>,
    P: OutputPin<Error = E>,
{
    fn new(pwm_u: CHU, pwm_v: CHV, pwm_w: CHW, enable_pin: P) -> Self {
        PWMs {
            duty_max: PWM_TOP,
            u: pwm_u,
            v: pwm_v,
            w: pwm_w,
            en_pin: Some(enable_pin),
        }
    }
}
