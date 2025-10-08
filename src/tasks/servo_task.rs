extern crate alloc;

use crate::kinematics::{
    conversion::{cartesian_to_polar, polar_to_servo},
    gait_engine::MOVEMENT_COMPLETED,
};
use crate::robot::commands::ServoCommand;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Receiver};
use embassy_time::{Duration, Ticker};
use esp_hal::{i2c::master::I2c, Async};
use log::debug;
use pwm_pca9685::Pca9685;

#[embassy_executor::task]
pub async fn servo_task(
    mut pwm: Pca9685<I2c<'static, Async>>,
    receiver: Receiver<'static, CriticalSectionRawMutex, ServoCommand, 3>,
) {
    pwm.set_prescale(100)
        .await
        .expect("Fail configurating pca driver"); //WARN: need 50Hz
    pwm.enable().await.expect("Fail enabling the pca driver");

    loop {
        let cmd = receiver.receive().await;
        debug!("[SERVO_TASK] Received a command!");
        update_position(cmd, &mut pwm).await;
    }
}

pub async fn update_position(mut cmd: ServoCommand, pwm: &mut Pca9685<I2c<'static, Async>>) {
    let mut ticker = Ticker::every(Duration::from_millis(20));

    loop {
        for leg in 0..4 {
            for joint in 0..3 {
                let diff = (cmd.current_pos[leg][joint] - cmd.expected_pos[leg][joint]).abs();
                let speed = cmd.temp_speed[leg][joint].abs();

                if diff >= speed {
                    cmd.current_pos[leg][joint] += cmd.temp_speed[leg][joint];
                } else {
                    cmd.current_pos[leg][joint] = cmd.expected_pos[leg][joint];
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
