use embassy_time::Timer;

pub enum Command {
    StepForward(i32),
    TurnLeft(i32),
    TurnRight(i32),
    SayHi,
}

#[embassy_executor::task]
pub async fn motion_task() {
    // pass command channel to the task
    // pass channel for the servo angles
    // Instantiate the GaitEngine

    loop {
        todo!();

        Timer::after_millis(20); //50Hz
    }
}
