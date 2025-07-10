use crate::{
    commands::{ServoCommand, TcpCommand},
    gait_engine::GaitEngine,
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Receiver, Sender},
};
use embassy_time::{Duration, Ticker};
use log::info;

#[embassy_executor::task]
pub async fn motion_task(
    tcp_cmd_receiver: Receiver<'static, CriticalSectionRawMutex, TcpCommand, 3>,
    servo_cmd_sender: Sender<'static, CriticalSectionRawMutex, ServoCommand, 3>,
) {
    let mut ticker = Ticker::every(Duration::from_millis(20));
    let mut gait = GaitEngine::new(servo_cmd_sender);
    gait.init_positions();

    loop {
        info!("[MOTION_TASK] listening for command...");
        match tcp_cmd_receiver.receive().await {
            TcpCommand::StepForward(n) => {
                gait.step_forward(n).await;
                info!("[MOTION_TASK] receive step forward {n}");
            }
            TcpCommand::Wave(n) => {
                gait.say_hi(n).await;
                info!("[MOTION_TASK] received wave command")
            }
            _ => info!("[MOTION_TASK] received other command"),
        }

        ticker.next().await;
    }
}
