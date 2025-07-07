use core::f32;

use crate::config::*;

/// State machine that calculate movements and update its posisions and speed accordingly
pub struct GaitEngine {
    current_pos: [[f32; 3]; 4], // real time coordinates of the end of each leg
    expected_pos: [[f32; 3]; 4], // expected coordinates
    temp_speed: [[f32; 3]; 4],
    config: RobotConfig,
}

impl GaitEngine {
    pub fn new() -> Self {
        let current_pos = [[0.0; 3]; 4];
        let expected_pos = [[0.0; 3]; 4];
        let temp_speed = [[0.0; 3]; 4];
        let config = RobotConfig::new();

        Self {
            current_pos,
            expected_pos,
            temp_speed,
            config,
        }
    }

    /// Init position arrays with initial values
    pub fn init() {}

    /// Update expected site and temp_speed
    fn set_site(leg_nb: u8, x: f32, y: f32, z: f32) {}
}

