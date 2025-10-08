use crate::kinematics::gait_engine::GaitEngine;
use crate::robot::commands::{ServoCommand, TcpCommand};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Receiver, Sender},
};
use embassy_time::{Duration, Ticker};
use log::{debug, info};

#[embassy_executor::task]
pub async fn motion_task(
    tcp_cmd_receiver: Receiver<'static, CriticalSectionRawMutex, TcpCommand, 3>,
    servo_cmd_sender: Sender<'static, CriticalSectionRawMutex, ServoCommand, 3>,
) {
    let mut ticker = Ticker::every(Duration::from_millis(20));
    let mut gait = GaitEngine::new(servo_cmd_sender);
    gait.init_positions().await;
    debug!("{:?}", gait.config());

    loop {
        info!("[MOTION_TASK] listening for command...");
        let stamp = "[MOTION_TASK] received";
        match tcp_cmd_receiver.receive().await {
            TcpCommand::Test => {
                info!("{stamp} test");
                gait.do_test().await;
            }
            TcpCommand::Calibrate => {
                info!("{stamp} calibrate");
                gait.calibrate().await;
            }
            TcpCommand::StepForward(n) => {
                info!("{stamp} step forward {n}");
                gait.step_forward(n).await;
            }
            TcpCommand::Wave(n) => {
                info!("{stamp} wave command");
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
        debug!("Gait engine state:\n {gait:?}");

        ticker.next().await;
    }
}
