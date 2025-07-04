use esp_hal::ledc::{channel::Channel, HighSpeed, LowSpeed};
use fugit::Hertz;
use log::debug;

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

    /// Sets the servo angle in degrees.
    ///
    /// # Arguments
    /// * `angle` - A value between 0 and 180 degrees. Values outside this range are clamped.
    ///
    /// # Returns
    /// * `Ok(())` on success
    /// * `Err(())` if the PWM driver fails to update the duty cycle
    pub fn set_angle(&mut self, angle: u8) -> Result<(), ()> {
        let angle = angle.clamp(0, 180);

        //Avoid setting the same angle again
        if self.angle == angle {
            return Ok(());
        }
        self.angle = angle;

        // Convert angle (0–180) to pulse width (500–2500 us)
        let min_pulse = 500; // microseconds (0°)
        let max_pulse = 2400; //microseconds (180°)

        // Linearly interpolate the pulse
        let pulse = min_pulse + ((angle as u32 * (max_pulse - min_pulse)) as u32 / 180);
        // e.g.: 90° -> 1450 µs

        // Scale pulse to PWM register resolution
        // Example: 1450 µs / 20000 µs * 255 ≈ 18
        // THE WIDTH OF THE PULSE DRIVES THE ANGLE, NOT FREQ
        let period_us = 1_000_000 / self.frequency.raw();
        let duty = ((pulse * self.max_duty) / period_us).min(self.max_duty) as u16;
        debug!("duty: {duty:?}");
        self.pwm.set_duty_cycle(duty).map_err(|_| ())
    }

    pub fn angle(&self) -> u8 {
        self.angle
    }
}

/// The AnyServo enum serves as a wrapper around servos that might old different underlying
/// channels (lowSpeed and HighSpeed). This way we can treat the servos as the same objects despite
/// their differences (e.g. to store in a vector)
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
