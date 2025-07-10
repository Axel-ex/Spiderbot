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
        let stamp = "[MOTION_TASK] received";
        match tcp_cmd_receiver.receive().await {
            TcpCommand::StepForward(n) => {
                info!("{stamp} step forward {n}");
                gait.step_forward(n).await;
            }
            TcpCommand::Wave(n) => {
                info!("{stamp} wave command");
                gait.say_hi(n).await;
            }
            TcpCommand::Sit => {
                info!("{stamp} sit command");
                gait.sit().await;
            }
            TcpCommand::Stand => {
                info!("{stamp} stand command");
                gait.stand().await;
            }
            _ => info!("{stamp} other command"),
        }

        ticker.next().await;
    }
}
