use esp_hal::ledc::{channel::Channel, HighSpeed, LowSpeed};
use fugit::Hertz;
use log::info;

use embedded_hal::pwm::SetDutyCycle;

pub struct Servo<PWM> {
    pwm: PWM,
    angle: u8,
    max_duty: u32,
    frequency: Hertz<u32>,
}

pub enum AnyServo {
    Low(Servo<Channel<'static, LowSpeed>>),
    High(Servo<Channel<'static, HighSpeed>>),
}

impl<PWM> Servo<PWM>
where
    PWM: SetDutyCycle,
{
    pub fn new(pwm: PWM, max_duty: u32, period: Hertz<u32>) -> Self {
        Self {
            pwm,
            angle: 0,
            max_duty,
            frequency: period,
        }
    }

    fn set_angle(&mut self, angle: u8) -> Result<(), ()> {
        let angle = angle.clamp(0, 180);
        self.angle = angle;

        // Convert angle (0–180) to pulse width (500–2500 us)
        let min_pulse = 500;
        let max_pulse = 2400;
        let pulse = min_pulse + ((angle as u32 * (max_pulse - min_pulse)) as u32 / 180);
        let period_us = 1_000_000 / self.frequency.raw();

        let duty = ((pulse * self.max_duty) / period_us).min(self.max_duty) as u16;
        info!("duty: {duty:?}");
        self.pwm.set_duty_cycle(duty).map_err(|_| ())
    }

    fn angle(&self) -> u8 {
        self.angle
    }
}

impl AnyServo {
    pub fn set_angle(&mut self, angle: u8) -> Result<(), ()> {
        match self {
            AnyServo::Low(channel) => channel.set_angle(angle),
            AnyServo::High(channel) => channel.set_angle(angle),
        }
    }

    fn angle(&self) -> u8 {
        match self {
            AnyServo::Low(channel) => channel.angle(),
            AnyServo::High(channel) => channel.angle(),
        }
    }
}
