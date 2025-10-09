//! Inverse kinematics and servo pulse conversion.
//!
//! Provides functions to convert between Cartesian coordinates and joint angles
//! for each leg, as well as mapping joint angles to servo pulse widths.
//!
//! Used by the gait engine (held by the motion task) to plan and execute leg movements.
use core::f32::consts::PI;
use micromath::F32Ext;

use crate::robot::{
    config::{LENGTH_A, LENGTH_B, LENGTH_C},
    joint::Joint,
    leg::Leg,
};
use esp_hal::{i2c::master::I2c, Async};
use log::error;
use pwm_pca9685::{Channel, Pca9685};

// --- Servo Configuration ---
const SERVO_MIN_PULSE_US: f32 = 544.0;
const SERVO_MAX_PULSE_US: f32 = 2400.0;
const SERVO_ANGLE_RANGE: f32 = 180.0;
const PCA_FREQUENCY_HZ: u32 = 50;
const PCA_PERIOD_US: f32 = 1_000_000.0 / PCA_FREQUENCY_HZ as f32; // 20000 µs
const PRESCALE_REG_SIZE: f32 = 4096.0;

//[femur, tibia, coxa]
static SERVO_CHANNEL_MAP: [[Channel; 3]; 4] = [
    [Channel::C0, Channel::C1, Channel::C2],   // front left
    [Channel::C3, Channel::C4, Channel::C5],   // bottom left
    [Channel::C6, Channel::C7, Channel::C8],   // front right
    [Channel::C9, Channel::C10, Channel::C11], // bottom right
];

fn angle_to_ticks(angle: f32) -> u16 {
    let pulse_width_range = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
    let pulse_us = SERVO_MIN_PULSE_US + (angle / SERVO_ANGLE_RANGE) * pulse_width_range;
    let tick = (pulse_us / PCA_PERIOD_US) * PRESCALE_REG_SIZE;
    // Clamp the value to the valid PCA9685 range
    tick.round().clamp(0.0, PRESCALE_REG_SIZE - 1.0) as u16
}

async fn set_leg_angles(
    pwm: &mut Pca9685<I2c<'static, Async>>,
    leg: Leg,
    alpha: f32,
    beta: f32,
    gamma: f32,
) {
    let femur_tick = angle_to_ticks(alpha);
    let tibia_tick = angle_to_ticks(beta);
    let coxa_tick = angle_to_ticks(gamma);

    let channels = SERVO_CHANNEL_MAP[leg as usize];
    if let Err(e) = pwm
        .set_channel_on_off(channels[Joint::Femur as usize], 0, femur_tick)
        .await
    {
        error!("{e}");
    }
    if let Err(e) = pwm
        .set_channel_on_off(channels[Joint::Tibia as usize], 0, tibia_tick)
        .await
    {
        error!("{e}");
    }
    if let Err(e) = pwm
        .set_channel_on_off(channels[Joint::Coxa as usize], 0, coxa_tick)
        .await
    {
        error!("{e}");
    }
}

/// transform in place alpha beta and gamma using mathematical model
pub fn cartesian_to_polar(x: f32, y: f32, z: f32) -> (f32, f32, f32) {
    let (mut alpha, mut beta, mut gamma);

    // Calculate w-z degree
    let w_sign = if x >= 0.0 { 1.0 } else { -1.0 };
    let w = w_sign * (x.powi(2) + y.powi(2)).sqrt();
    let v = w - LENGTH_C;

    let d_squared = v.powi(2) + z.powi(2);
    let d = d_squared.sqrt();

    alpha = z.atan2(v)
        + ((LENGTH_A.powi(2) - LENGTH_B.powi(2) + d_squared) / (2.0 * LENGTH_A * d)).acos();

    beta = ((LENGTH_A.powi(2) + LENGTH_B.powi(2) - d_squared) / (2.0 * LENGTH_A * LENGTH_B)).acos();

    // Calculate x-y-z degree
    gamma = if w >= 0.0 { y.atan2(x) } else { (-y).atan2(-x) };

    // Convert radians to degrees
    alpha = alpha * 180.0 / PI;
    beta = beta * 180.0 / PI;
    gamma = gamma * 180.0 / PI;

    (alpha, beta, gamma)
}

pub async fn polar_to_servo(
    pwm: &mut Pca9685<I2c<'static, Async>>,
    leg: Leg,
    mut alpha: f32,
    mut beta: f32,
    mut gamma: f32,
) {
    match leg {
        Leg::FrontLeft => {
            alpha = 90.0 - alpha;
            beta = beta;
            gamma += 90.0;
        }
        Leg::BottomLeft => {
            alpha += 90.0;
            beta = 180.0 - beta;
            gamma = 90.0 - gamma;
        }
        Leg::FrontRight => {
            alpha += 90.0;
            beta = 180.0 - beta;
            gamma = 90.0 - gamma;
        }
        Leg::BottomRight => {
            alpha = 90.0 - alpha;
            beta = beta;
            gamma += 90.0;
        }
    }

    set_leg_angles(pwm, leg, alpha, beta, gamma).await;
}
