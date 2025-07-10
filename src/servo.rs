use core::fmt::Display;

use esp_hal::ledc::{channel::Channel, HighSpeed, LowSpeed};
use fugit::Hertz;
use log::{debug, error};

use embedded_hal::pwm::SetDutyCycle;

pub enum Leg {
    FrontLeft = 0,
    BottomLeft = 1,
    FrontRight = 2,
    BottomRight = 3,
}

pub enum Joint {
    Coxa = 0,
    Femur = 1,
    Tibia = 2,
}

impl Display for Leg {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Leg::FrontLeft => f.write_str("Front left"),
            Leg::FrontRight => f.write_str("Front right"),
            Leg::BottomLeft => f.write_str("Bottom left"),
            Leg::BottomRight => f.write_str("Bottom right"),
        }
    }
}

impl Display for Joint {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Joint::Coxa => f.write_str("coxa"),
            Joint::Femur => f.write_str("femur"),
            Joint::Tibia => f.write_str("tibia"),
        }
    }
}

// TODO: implement from usize for joint and leg

pub struct Servo<PWM> {
    pwm: PWM,
    angle: u8,
    max_duty: u32,
    frequency: Hertz<u32>,
    leg_id: Leg,
    joint_id: Joint,
}

pub enum AnyServo {
    Low(Servo<Channel<'static, LowSpeed>>),
    High(Servo<Channel<'static, HighSpeed>>),
}

impl<PWM> Servo<PWM>
where
    PWM: SetDutyCycle,
{
    pub fn new(pwm: PWM, max_duty: u32, period: Hertz<u32>, leg_id: Leg, joint_id: Joint) -> Self {
        Self {
            pwm,
            angle: 0,
            max_duty,
            frequency: period,
            leg_id,
            joint_id,
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
    pub fn set_angle(&mut self, angle: u8) {
        let angle = angle.clamp(0, 180);

        //Avoid setting the same angle again
        if self.angle == angle {
            return;
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
        if let Err(e) = self.pwm.set_duty_cycle(duty) {
            error!(
                "{} {} Error writing angle {:?}",
                self.leg_id, self.joint_id, e
            );
        }
    }

    pub fn angle(&self) -> u8 {
        self.angle
    }
}

/// The AnyServo enum serves as a wrapper around servos that might old different underlying
/// channels (lowSpeed and HighSpeed). This way we can treat the servos as the same objects despite
/// their differences (e.g. to store in a vector)
impl AnyServo {
    pub fn set_angle(&mut self, angle: u8) {
        match self {
            AnyServo::Low(servo) => servo.set_angle(angle),
            AnyServo::High(servo) => servo.set_angle(angle),
        }
    }

    fn angle(&self) -> u8 {
        match self {
            AnyServo::Low(servo) => servo.angle(),
            AnyServo::High(servo) => servo.angle(),
        }
    }
}
