use crate::constants::RobotConfig;

pub enum Command {
    StepForward(i32),
    TurnLeft(i32),
    TurnRight(i32),
    SayHi,
}
#[embassy_executor::task]
pub async fn motion_task() {}
