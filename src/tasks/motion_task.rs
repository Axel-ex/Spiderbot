use crate::{commands::Command, gait_engine::GaitEngine};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Receiver};
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn motion_task(cmd_receiver: Receiver<'static, CriticalSectionRawMutex, Command, 3>) {
    // pass command channel to the task
    // pass channel for the servo angles
    // Instantiate the GaitEngine
    let mut gait = GaitEngine::new();
    gait.init_positions();

    loop {
        Timer::after_millis(20).await; //50Hz
    }
}
