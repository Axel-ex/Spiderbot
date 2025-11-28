//! High-level motion task for Spiderbot.
//!
//! Receives movement commands, computes gait steps, and coordinates leg positions
//! using the gait engine and kinematics modules.
//!
//! Communicates with the servo task to execute planned movements.
use crate::kinematics::gait_engine::GaitEngine;
use crate::robot::commands::{ServoCommand, TcpCommand};
use crate::{SERVOCMD_CHANNEL_SIZE, TCPCMD_CHANNEL_SIZE};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Receiver, Sender},
};
use log::{debug, info};

#[embassy_executor::task]
pub async fn motion_task(
    tcp_cmd_receiver: Receiver<'static, CriticalSectionRawMutex, TcpCommand, TCPCMD_CHANNEL_SIZE>,
    servo_cmd_sender: Sender<'static, CriticalSectionRawMutex, ServoCommand, SERVOCMD_CHANNEL_SIZE>,
) {
    let mut gait = GaitEngine::new(servo_cmd_sender);
    gait.init_positions().await;
    debug!("{:?}", gait.config());

    loop {
        let stamp = "[MOTION_TASK] received";
        match tcp_cmd_receiver.receive().await {
            TcpCommand::Test => {
                info!("{stamp} test");
                gait.do_test().await;
            }
            TcpCommand::CloseConnection => {
                info!("{stamp} close connection");
            }
            TcpCommand::StepForward(n) => {
                info!("{stamp} step forward {n}");
                gait.step_forward(n).await;
            }
            TcpCommand::StepBackward(n) => {
                info!("{stamp} step backward {n}");
                gait.step_backward(n).await;
            }
            TcpCommand::Wave(n) => {
                info!("{stamp} wave {n}");
                gait.wave(n).await;
            }
            TcpCommand::Sit => {
                info!("{stamp} sit command");
                gait.sit().await;
            }
            TcpCommand::Stand => {
                info!("{stamp} stand command");
                gait.stand().await;
            }
            TcpCommand::TurnLeft(n) => {
                info!("{stamp} turn left {n}");
                gait.turn_left(n).await;
            }
            TcpCommand::TurnRight(n) => {
                info!("{stamp} turn right {n}");
                gait.turn_right(n).await;
            }
            _ => info!("{stamp} unknown command"),
        }
    }
}
