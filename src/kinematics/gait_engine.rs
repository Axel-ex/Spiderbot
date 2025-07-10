use crate::robot::{commands::ServoCommand, config::*};
use core::f32;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Sender, signal::Signal};
use embassy_time::{with_timeout, Duration};
use log::{error, info};
use micromath::F32Ext;

pub static MOVEMENT_COMPLETED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// State machine that calculate movements and update its posisions and speed accordingly
pub struct GaitEngine {
    current_pos: [[f32; 3]; 4], // real time coordinates of the end of each leg
    expected_pos: [[f32; 3]; 4], // expected coordinates
    temp_speed: [[f32; 3]; 4],  // Speed to reach expected pos.
    config: RobotConfig,
    servo_cmd_sender: Sender<'static, CriticalSectionRawMutex, ServoCommand, 3>, //channel for ServoCommand
}

impl GaitEngine {
    pub fn new(
        servo_cmd_sender: Sender<'static, CriticalSectionRawMutex, ServoCommand, 3>,
    ) -> Self {
        let current_pos = [[0.0; 3]; 4];
        let expected_pos = [[0.0; 3]; 4];
        let temp_speed = [[0.0; 3]; 4];
        let config = RobotConfig::new();

        Self {
            servo_cmd_sender,
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

        for leg in 0..4 {
            for joint in 0..3 {
                self.current_pos[leg][joint] = self.expected_pos[leg][joint];
            }
        }
        info!("Spider robot initialized!");
    }

    /// Send the internal state of the gait engine to the servo task and update position (it
    /// assumes the command always succeed) //TODO: should it fail? watchdog? clean state?
    pub async fn send_cmd(&mut self) {
        let cmd = ServoCommand::new(self.current_pos, self.expected_pos, self.temp_speed);
        self.servo_cmd_sender.send(cmd).await;

        // wait for the notification from the servo task
        match with_timeout(Duration::from_secs(3), MOVEMENT_COMPLETED.wait()).await {
            Ok(_) => self.current_pos = self.expected_pos,
            Err(_) => error!("[MOTION_TASK] command timed out"),
        }
    }

    pub async fn sit(&mut self) {
        self.config.move_speed = self.config.stand_seat_speed; // WARN: not very nice pattern maybe
                                                               // passing the speed as argument for set_site is cleaner
        for leg in 0..4 {
            self.set_site(leg, KEEP, KEEP, Z_BOOT);
        }
        self.send_cmd().await;
    }

    pub async fn stand(&mut self) {
        self.config.move_speed = self.config.stand_seat_speed;
        for leg in 0..4 {
            self.set_site(leg, KEEP, KEEP, Z_DEFAULT);
        }
        self.send_cmd().await;
    }

    pub async fn step_forward(&mut self, _times: u8) {
        self.set_site(0, self.config.turn_x1, self.config.turn_y1, KEEP);
        self.send_cmd().await;
        info!("[MOTION_TASK] step_forward completed!");
    }

    /// Move front right leg or front left depending on?
    pub async fn say_hi(&mut self, times: u8) {
        let (x_tmp, y_tmp, z_tmp);

        if self.current_pos[3][1] == Y_START {
            // NOTE: WHat?
            self.body_right(15).await;
            x_tmp = self.current_pos[2][0];
            y_tmp = self.current_pos[2][1];
            z_tmp = self.current_pos[2][2];
            self.config.move_speed = self.config.body_move_speed; //update the move speed before
                                                                  //setting sites

            for _ in 0..times {
                self.set_site(2, self.config.turn_x1, self.config.turn_y1, 50.0);
                self.send_cmd().await;
                self.set_site(2, self.config.turn_x0, self.config.turn_y0, 50.0);
                self.send_cmd().await;
            }
            self.set_site(2, x_tmp, y_tmp, z_tmp);
            self.config.move_speed = 1.0;
            self.body_left(15).await;
        } else {
            self.body_left(15).await;
            x_tmp = self.current_pos[0][0];
            y_tmp = self.current_pos[0][1];
            z_tmp = self.current_pos[0][2];
            self.config.move_speed = self.config.body_move_speed; //update the move speed before
                                                                  //setting sites

            for _ in 0..times {
                self.set_site(0, self.config.turn_x1, self.config.turn_y1, 50.0);
                self.send_cmd().await;
                self.set_site(0, self.config.turn_x0, self.config.turn_y0, 50.0);
                self.send_cmd().await;
            }
            self.set_site(0, x_tmp, y_tmp, z_tmp);
            self.send_cmd().await;
            self.config.move_speed = 1.0;
            self.body_right(15).await;
        }
        info!("[MOTION TASK] say hi completed!")
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

    async fn body_left(&mut self, i: i32) {
        self.set_site(0, self.current_pos[0][0] + i as f32, KEEP, KEEP);
        self.set_site(1, self.current_pos[1][0] + i as f32, KEEP, KEEP);
        self.set_site(2, self.current_pos[2][0] - i as f32, KEEP, KEEP);
        self.set_site(3, self.current_pos[3][0] - i as f32, KEEP, KEEP);
        self.send_cmd().await;
        MOVEMENT_COMPLETED.wait().await;
    }

    async fn body_right(&mut self, i: i32) {
        self.set_site(0, self.current_pos[0][0] - i as f32, KEEP, KEEP);
        self.set_site(1, self.current_pos[1][0] - i as f32, KEEP, KEEP);
        self.set_site(2, self.current_pos[2][0] + i as f32, KEEP, KEEP);
        self.set_site(3, self.current_pos[3][0] + i as f32, KEEP, KEEP);
        self.send_cmd().await;
        MOVEMENT_COMPLETED.wait().await;
    }
}
