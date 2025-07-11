use crate::robot::{commands::ServoCommand, config::*, leg::Leg};
use core::f32;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Sender, signal::Signal};
use embassy_time::{with_timeout, Duration};
use log::{debug, error, info};
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

    pub fn config(&self) -> &RobotConfig {
        &self.config
    }

    /// Init position arrays with initial values
    pub async fn init_positions(&mut self) {
        let speed = self.config.move_speed;
        self.set_site(
            Leg::FrontLeft,
            X_DEFAULT - X_OFFSET,
            Y_START + Y_STEP,
            Z_BOOT,
            speed,
        );
        self.set_site(
            Leg::BottomLeft,
            X_DEFAULT - X_OFFSET,
            Y_START + Y_STEP,
            Z_BOOT,
            speed,
        );
        self.set_site(
            Leg::FrontRight,
            X_DEFAULT + X_OFFSET,
            Y_START,
            Z_BOOT,
            speed,
        );
        self.set_site(
            Leg::BottomRight,
            X_DEFAULT + X_OFFSET,
            Y_START,
            Z_BOOT,
            speed,
        );

        for leg in 0..4 {
            for joint in 0..3 {
                self.current_pos[leg][joint] = self.expected_pos[leg][joint];
            }
        }
        self.send_cmd().await;

        info!("Spider robot initialized!");
    }

    pub async fn calibrate(&mut self) {
        let speed = self.config.move_speed;
        self.set_site(Leg::FrontLeft, 0.0, 0.0, 0.0, speed);
        self.set_site(Leg::BottomLeft, 0.0, 0.0, 0.0, speed);
        self.set_site(Leg::FrontRight, 0.0, 0.0, 0.0, speed);
        self.set_site(Leg::BottomRight, 0.0, 0.0, 0.0, speed);
        self.send_cmd().await;
    }

    /// Send the internal state of the gait engine to the servo task and update position (it
    /// assumes the command always succeed) //TODO: should it fail? watchdog? clean state?
    pub async fn send_cmd(&mut self) {
        let cmd = ServoCommand::new(self.current_pos, self.expected_pos, self.temp_speed);
        self.servo_cmd_sender.send(cmd).await;

        // wait for the notification from the servo task
        match with_timeout(Duration::from_secs(10), MOVEMENT_COMPLETED.wait()).await {
            Ok(_) => self.current_pos = self.expected_pos,
            Err(_) => error!("[MOTION_TASK] command timed out"),
        }
    }

    pub async fn sit(&mut self) {
        for leg in 0..4 {
            self.set_site(leg.into(), KEEP, KEEP, Z_BOOT, self.config.stand_seat_speed);
        }
        self.send_cmd().await;
    }

    pub async fn stand(&mut self) {
        for leg in 0..4 {
            self.set_site(
                leg.into(),
                KEEP,
                KEEP,
                Z_DEFAULT,
                self.config.stand_seat_speed,
            );
        }
        self.send_cmd().await;
    }

    pub async fn step_forward(&mut self, times: u8) {
        let mut speed = self.config.leg_move_speed;

        for _ in 0..times {
            if self.current_pos[Leg::FrontRight][1] == Y_START {
                self.set_site(Leg::FrontRight, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.send_cmd().await;
                self.set_site(
                    Leg::FrontRight,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_UP,
                    speed,
                );
                self.send_cmd().await;
                self.set_site(
                    Leg::FrontRight,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_DEFAULT,
                    speed,
                );
                self.send_cmd().await;

                speed = self.config.body_move_speed;
                self.set_site(
                    Leg::FrontLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::FrontRight,
                    X_DEFAULT - X_OFFSET,
                    Y_START + Y_STEP,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomRight,
                    X_DEFAULT - X_OFFSET,
                    Y_START + Y_STEP,
                    Z_DEFAULT,
                    speed,
                );
                self.send_cmd().await;

                speed = self.config.leg_move_speed;
                self.set_site(
                    Leg::BottomLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_UP,
                    speed,
                );
                self.send_cmd().await;
                self.set_site(Leg::BottomLeft, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.send_cmd().await;
                self.set_site(
                    Leg::BottomLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START,
                    Z_DEFAULT,
                    speed,
                );
                self.send_cmd().await;
            } else {
                self.set_site(Leg::FrontLeft, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.send_cmd().await;
                self.set_site(
                    Leg::FrontLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_UP,
                    speed,
                );
                self.send_cmd().await;
                self.set_site(
                    Leg::FrontLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_DEFAULT,
                    speed,
                );
                self.send_cmd().await;

                speed = self.config.body_move_speed;
                self.set_site(
                    Leg::FrontLeft,
                    X_DEFAULT - X_OFFSET,
                    Y_START + Y_STEP,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomLeft,
                    X_DEFAULT - X_OFFSET,
                    Y_START + Y_STEP,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::FrontRight,
                    X_DEFAULT + X_OFFSET,
                    Y_START,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomRight,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_DEFAULT,
                    speed,
                );
                self.send_cmd().await;

                speed = self.config.leg_move_speed;
                self.set_site(
                    Leg::BottomRight,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_UP,
                    speed,
                );
                self.send_cmd().await;
                self.set_site(Leg::BottomRight, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.send_cmd().await;
                self.set_site(
                    Leg::BottomRight,
                    X_DEFAULT + X_OFFSET,
                    Y_START,
                    Z_DEFAULT,
                    speed,
                );
                self.send_cmd().await;
            }
        }
    }

    /// Move front right leg or front left depending on?
    pub async fn wave(&mut self, times: u8) {
        let (x_tmp, y_tmp, z_tmp);
        let leg;
        let speed = self.config.body_move_speed;

        if self.current_pos[3][1] == Y_START {
            leg = Leg::FrontRight;
            self.body_right(15).await;
            x_tmp = self.current_pos[leg][0];
            y_tmp = self.current_pos[leg][1];
            z_tmp = self.current_pos[leg][2];

            for _ in 0..times {
                self.set_site(leg, self.config.turn_x1, self.config.turn_y1, 50.0, speed);
                self.send_cmd().await;
                self.set_site(leg, self.config.turn_x0, self.config.turn_y0, 50.0, speed);
                self.send_cmd().await;
            }
            self.set_site(leg, x_tmp, y_tmp, z_tmp, speed);
            self.send_cmd().await;
            self.body_left(15).await;
        } else {
            leg = Leg::FrontLeft;
            self.body_left(15).await;
            x_tmp = self.current_pos[leg][0];
            y_tmp = self.current_pos[leg][1];
            z_tmp = self.current_pos[leg][2];
            self.config.move_speed = self.config.body_move_speed; //update the move speed before
                                                                  //setting sites

            for _ in 0..times {
                self.set_site(leg, self.config.turn_x1, self.config.turn_y1, 50.0, speed);
                self.send_cmd().await;
                self.set_site(leg, self.config.turn_x0, self.config.turn_y0, 50.0, speed);
                self.send_cmd().await;
            }
            self.set_site(leg, x_tmp, y_tmp, z_tmp, speed);
            self.send_cmd().await;
            self.config.move_speed = 1.0;
            self.body_right(15).await;
        }
        debug!("[MOTION TASK] wave completed!")
    }

    /// Update expected site and temp_speed
    fn set_site(&mut self, leg: Leg, x: f32, y: f32, z: f32, move_speed: f32) {
        let (mut length_x, mut length_y, mut length_z) = (0.0, 0.0, 0.0);

        if x != KEEP {
            length_x = x - self.current_pos[leg][0];
        }
        if y != KEEP {
            length_y = y - self.current_pos[leg][1];
        }
        if z != KEEP {
            length_z = z - self.current_pos[leg][2];
        }

        let length = (length_x.powi(2) + length_y.powi(2) + length_z.powi(2)).sqrt();

        self.temp_speed[leg][0] = length_x / length * move_speed * self.config.speed_multiple;
        self.temp_speed[leg][1] = length_y / length * move_speed * self.config.speed_multiple;
        self.temp_speed[leg][2] = length_z / length * move_speed * self.config.speed_multiple;

        if x != KEEP {
            self.expected_pos[leg][0] = x;
        }
        if y != KEEP {
            self.expected_pos[leg][1] = y;
        }
        if z != KEEP {
            self.expected_pos[leg][2] = z;
        }
    }

    async fn body_left(&mut self, i: i32) {
        let speed = self.config.move_speed;
        self.set_site(
            Leg::FrontLeft,
            self.current_pos[0][0] + i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.set_site(
            Leg::BottomLeft,
            self.current_pos[1][0] + i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.set_site(
            Leg::FrontRight,
            self.current_pos[2][0] - i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.set_site(
            Leg::BottomRight,
            self.current_pos[3][0] - i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.send_cmd().await;
    }

    async fn body_right(&mut self, i: i32) {
        let speed = self.config.move_speed;
        self.set_site(
            Leg::FrontLeft,
            self.current_pos[0][0] - i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.set_site(
            Leg::BottomLeft,
            self.current_pos[1][0] - i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.set_site(
            Leg::FrontRight,
            self.current_pos[2][0] + i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.set_site(
            Leg::BottomRight,
            self.current_pos[3][0] + i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.send_cmd().await;
    }
}

impl core::fmt::Debug for GaitEngine {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("GaitEngine")
            .field("current_pos", &self.current_pos)
            .field("expected_pos", &self.expected_pos)
            .field("temp_speed", &self.temp_speed)
            .finish()
    }
}
