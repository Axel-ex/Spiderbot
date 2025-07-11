use core::f32::consts::PI;
use micromath::F32Ext;

use crate::robot::{
    config::{LENGTH_A, LENGTH_B, LENGTH_C},
    leg::Leg,
    servo::AnyServo,
};

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

pub fn polar_to_servo(
    servos: &mut [[AnyServo; 3]; 4],
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

    servos[leg][0].set_angle(f32_to_u8(alpha));
    servos[leg][1].set_angle(f32_to_u8(beta));
    servos[leg][2].set_angle(f32_to_u8(gamma));
}

// Utility: Convert f32 to u8 safely
fn f32_to_u8(value: f32) -> u8 {
    value.round().clamp(0.0, 180.0) as u8
}
