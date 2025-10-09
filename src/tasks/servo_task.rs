//! Servo control task for Spiderbot.
//!
//! Receives joint angle commands and drives the servo controller hardware to
//! move the robot's legs accordingly.
//!
//! Handles servo timing and error reporting.
extern crate alloc;

use crate::kinematics::{
    conversion::{cartesian_to_polar, polar_to_servo},
    gait_engine::MOVEMENT_COMPLETED,
};
use crate::robot::commands::ServoCommand;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Receiver};
use embassy_time::{Duration, Ticker};
use esp_hal::{i2c::master::I2c, Async};
use log::{debug, info};
use pwm_pca9685::Pca9685;

const UPDATE_PERIOD_MS: u64 = 20;

#[embassy_executor::task]
pub async fn servo_task(
    mut pwm: Pca9685<I2c<'static, Async>>,
    receiver: Receiver<'static, CriticalSectionRawMutex, ServoCommand, 3>,
) {
    pwm.set_prescale(121)
        .await
        .expect("Fail configurating pca driver"); //prescale=(25,000,000 / 4096×50) −1
    pwm.enable().await.expect("Fail enabling the pca driver");
    let mut ticker = Ticker::every(Duration::from_millis(UPDATE_PERIOD_MS));

    let mut cmd = receiver.receive().await;
    loop {
        update_position(&mut cmd, &mut pwm).await;
        if let Ok(new_cmd) = receiver.try_receive() {
            cmd = new_cmd;
        }
        debug!("[SERVO_TASK] Received a command!");
        ticker.next().await;
    }
}

pub async fn update_position(cmd: &mut ServoCommand, pwm: &mut Pca9685<I2c<'static, Async>>) {
    let mut ticker = Ticker::every(Duration::from_millis(UPDATE_PERIOD_MS));

    loop {
        let now = embassy_time::Instant::now();
        for leg in 0..4 {
            for pos in 0..3 {
                let diff = (cmd.current_pos[leg][pos] - cmd.expected_pos[leg][pos]).abs();
                let speed = cmd.temp_speed[leg][pos].abs();

                if diff >= speed {
                    cmd.current_pos[leg][pos] += cmd.temp_speed[leg][pos];
                } else {
                    cmd.current_pos[leg][pos] = cmd.expected_pos[leg][pos];
                }
            }
            let (alpha, beta, gamma) = cartesian_to_polar(
                cmd.current_pos[leg][0],
                cmd.current_pos[leg][1],
                cmd.current_pos[leg][2],
            );
            polar_to_servo(pwm, leg.into(), alpha, beta, gamma).await;
        }
        if movement_is_done(&cmd) {
            break;
        }
        info!("tick at: {}", now.as_micros());

        ticker.next().await;
    }
    MOVEMENT_COMPLETED.signal(());
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
    cmd.current_pos[leg_id][0] == cmd.expected_pos[leg_id][0]
        && cmd.current_pos[leg_id][1] == cmd.expected_pos[leg_id][1]
        && cmd.current_pos[leg_id][2] == cmd.expected_pos[leg_id][2]
}
