use core::cell::RefCell;
use core::fmt::{self, Display, Formatter};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

#[derive(Debug)]
pub struct RobotState {
    pub current_pos: [[f32; 3]; 4],
    pub expected_pos: [[f32; 3]; 4],
    pub temp_speed: [[f32; 3]; 4],
}

pub static ROBOT_STATE: Mutex<CriticalSectionRawMutex, RefCell<RobotState>> =
    Mutex::new(RefCell::new(RobotState {
        current_pos: [[0.0; 3]; 4],
        expected_pos: [[0.0; 3]; 4],
        temp_speed: [[0.0; 3]; 4],
    }));
