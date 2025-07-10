use core::f32::consts::PI;
use log::error;
use micromath::F32Ext;

use crate::{
    commands::ServoCommand,
    config::{LENGTH_A, LENGTH_B, LENGTH_C},
    servo::AnyServo,
};

/// transform in place alpha beta and gamma using mathematical model
pub fn cartesian_to_polar(
    alpha: &mut f32,
    beta: &mut f32,
    gamma: &mut f32,
    x: f32,
    y: f32,
    z: f32,
) {
    // Calculate w-z degree
    let w_sign = if x >= 0.0 { 1.0 } else { -1.0 };
    let w = w_sign * (x.powi(2) + y.powi(2)).sqrt();
    let v = w - LENGTH_C;

    let d_squared = v.powi(2) + z.powi(2);
    let d = d_squared.sqrt();

    *alpha = z.atan2(v)
        + ((LENGTH_A.powi(2) - LENGTH_B.powi(2) + d_squared) / (2.0 * LENGTH_A * d)).acos();

    *beta =
        ((LENGTH_A.powi(2) + LENGTH_B.powi(2) - d_squared) / (2.0 * LENGTH_A * LENGTH_B)).acos();

    // Calculate x-y-z degree
    *gamma = if w >= 0.0 { y.atan2(x) } else { (-y).atan2(-x) };

    // Convert radians to degrees
    *alpha = *alpha * 180.0 / PI;
    *beta = *beta * 180.0 / PI;
    *gamma = *gamma * 180.0 / PI;
}

pub fn polar_to_servo(
    servos: &mut [[AnyServo; 3]; 4],
    leg_id: usize,
    mut alpha: f32,
    mut beta: f32,
    mut gamma: f32,
) {
    match leg_id {
        0 => {
            alpha = 90.0 - alpha;
            beta = beta;
            gamma += 90.0;
        }
        1 => {
            alpha += 90.0;
            beta = 180.0 - beta;
            gamma = 90.0 - gamma;
        }
        2 => {
            alpha += 90.0;
            beta = 180.0 - beta;
            gamma = 90.0 - gamma;
        }
        3 => {
            alpha = 90.0 - alpha;
            beta = beta;
            gamma += 90.0;
        }
        _ => {
            error!("[SERVO_TASK] Invalid leg id");
        }
    }
    //TODO:Handle error
    servos[leg_id][0].set_angle(f32_to_u8(alpha)).unwrap();
    servos[leg_id][1].set_angle(f32_to_u8(beta)).unwrap();
    servos[leg_id][2].set_angle(f32_to_u8(gamma)).unwrap();
}

pub fn movement_is_done(cmd: &ServoCommand) -> bool {
    for i in 0..4 {
        if !leg_movement_is_done(cmd, i) {
            return false;
        }
    }
    true
}

/// Check if the servos of a leg have reach their position
fn leg_movement_is_done(cmd: &ServoCommand, leg_id: usize) -> bool {
    cmd.current_pos[leg_id][0] == cmd.current_pos[leg_id][0]
        && cmd.current_pos[leg_id][1] == cmd.current_pos[leg_id][1]
        && cmd.current_pos[leg_id][2] == cmd.current_pos[leg_id][2]
}

// Utility: Convert f32 to u8 safely
fn f32_to_u8(value: f32) -> u8 {
    value.round().clamp(0.0, 180.0) as u8
}
