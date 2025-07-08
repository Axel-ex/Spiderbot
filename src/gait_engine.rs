use crate::config::*;
use core::f32;
use embassy_time::Timer;
use log::info;
use micromath::F32Ext;

/// State machine that calculate movements and update its posisions and speed accordingly
pub struct GaitEngine {
    current_pos: [[f32; 3]; 4], // real time coordinates of the end of each leg
    expected_pos: [[f32; 3]; 4], // expected coordinates
    temp_speed: [[f32; 3]; 4],  // Speed to reach expected pos.
    config: RobotConfig,
}

/// The enine can be in several possible state and transition between them. when it updates its
/// position, it simply add temp_speed to the coordinates and recalculates the speed (since the
/// distances change, the further, the faster). we can then wait forr all position to be reached by
/// checking if the current positions are the same as the expected ones
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
    pub fn init_positions(&mut self) {
        self.set_site(0, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_BOOT);
        self.set_site(1, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_BOOT);
        self.set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_BOOT);
        self.set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_BOOT);

        for leg_id in 0..4 {
            for i in 0..3 {
                self.current_pos[leg_id][i] = self.expected_pos[leg_id][i];
            }
        }
        info!("Spider robot initialized!");
    }

    /// Update expected site and temp_speed
    fn set_site(&mut self, leg_id: usize, x: f32, y: f32, z: f32) {
        let (mut length_x, mut length_y, mut length_z) = (0.0, 0.0, 0.0);

        if x != KEEP {
            length_x = x - self.current_pos[leg_id][0];
        }
        if y != KEEP {
            length_y = y - self.current_pos[leg_id][1];
        }
        if z != KEEP {
            length_z = z - self.current_pos[leg_id][2];
        }

        let length = (length_x.powi(2) + length_y.powi(2) + length_z.powi(2)).sqrt();

        self.temp_speed[leg_id][0] =
            length_x / length * self.config.move_speed * self.config.speed_multiple;
        self.temp_speed[leg_id][1] =
            length_y / length * self.config.move_speed * self.config.speed_multiple;
        self.temp_speed[leg_id][2] =
            length_z / length * self.config.move_speed * self.config.speed_multiple;

        if x != KEEP {
            self.expected_pos[leg_id][0] = x;
        }
        if y != KEEP {
            self.expected_pos[leg_id][1] = y;
        }
        if z != KEEP {
            self.expected_pos[leg_id][2] = z;
        }
    }

    pub async fn wait_reach(&mut self, leg_id: usize) {
        loop {
            if self.current_pos[leg_id][0] == self.expected_pos[leg_id][0]
                && self.current_pos[leg_id][1] == self.expected_pos[leg_id][1]
                && self.current_pos[leg_id][2] == self.expected_pos[leg_id][2]
            {
                break;
            }
            Timer::after_millis(1).await;
        }
    }

    pub async fn wait_all_reach(&mut self) {
        for leg_id in 0..4 {
            self.wait_reach(leg_id).await;
        }
    }
}
