use crate::robot::state::ROBOT_STATE;
use crate::robot::{config::*, leg::Leg};
use core::f32;
use embassy_time::Timer;
use log::{debug, info};
use micromath::F32Ext;

/// State machine that calculate movements and update its posisions and speed accordingly
pub struct GaitEngine {
    config: RobotConfig,
}

impl GaitEngine {
    pub fn new() -> Self {
        let config = RobotConfig::new();

        Self { config }
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

        ROBOT_STATE.lock(|state| {
            let mut state = state.borrow_mut();
            for leg in 0..4 {
                for joint in 0..3 {
                    state.current_pos[leg][joint] = state.expected_pos[leg][joint];
                }
            }
        });
        self.wait_all_reach().await;

        info!("Spider robot initialized!");
    }

    pub async fn calibrate(&mut self) {
        let speed = self.config.move_speed;
        self.set_site(Leg::FrontLeft, 0.0, 0.0, 0.0, speed);
        self.set_site(Leg::BottomLeft, 0.0, 0.0, 0.0, speed);
        self.set_site(Leg::FrontRight, 0.0, 0.0, 0.0, speed);
        self.set_site(Leg::BottomRight, 0.0, 0.0, 0.0, speed);
        self.wait_all_reach().await;
    }

    /// Update expected site and temp_speed
    fn set_site(&mut self, leg: Leg, x: f32, y: f32, z: f32, move_speed: f32) {
        ROBOT_STATE.lock(|state| {
            let mut state = state.borrow_mut();
            let (mut length_x, mut length_y, mut length_z) = (0.0, 0.0, 0.0);

            if x != KEEP {
                length_x = x - state.current_pos[leg][0];
            }
            if y != KEEP {
                length_y = y - state.current_pos[leg][1];
            }
            if z != KEEP {
                length_z = z - state.current_pos[leg][2];
            }

            let length = (length_x.powi(2) + length_y.powi(2) + length_z.powi(2)).sqrt();
            if length == 0.0 {
                state.temp_speed[leg] = [0.0; 3];
                return;
            }

            state.temp_speed[leg][0] = length_x / length * move_speed * self.config.speed_multiple;
            state.temp_speed[leg][1] = length_y / length * move_speed * self.config.speed_multiple;
            state.temp_speed[leg][2] = length_z / length * move_speed * self.config.speed_multiple;

            if x != KEEP {
                state.expected_pos[leg][0] = x;
            }
            if y != KEEP {
                state.expected_pos[leg][1] = y;
            }
            if z != KEEP {
                state.expected_pos[leg][2] = z;
            }
        });
    }

    async fn wait_all_reach(&self) {
        loop {
            if self.is_all_reached() {
                break;
            }
            Timer::after_millis(2).await;
        }
    }

    fn is_all_reached(&self) -> bool {
        let mut all_reached = true;

        ROBOT_STATE.lock(|state| {
            let state = state.borrow();
            for leg in 0..4 {
                for joint in 0..3 {
                    let diff =
                        (state.current_pos[leg][joint] - state.expected_pos[leg][joint]).abs();
                    if diff > 0.001 {
                        all_reached = false;
                        return;
                    }
                }
            }
        });

        all_reached
    }

    fn debug_positions(&self) {
        ROBOT_STATE.lock(|state| {
            let state_ref = state.borrow();
            info!("{:?}", &*state_ref);
        });
    }

    pub async fn do_test(&mut self) {
        info!("Stand");
        self.stand().await;
        Timer::after_secs(2).await;
        info!("Step forward");
        self.step_forward(5).await;
        Timer::after_secs(2).await;
        info!("Turn left");
        self.turn_left(5).await;
        Timer::after_secs(2).await;
        info!("Turn right");
        self.turn_right(5).await;
        Timer::after_secs(2).await;
        info!("Wave");
        self.wave(5).await;
        Timer::after_secs(2).await;
        info!("Sit");
        self.sit().await;
        Timer::after_secs(5).await;
    }

    pub async fn sit(&mut self) {
        for leg in 0..4 {
            self.set_site(leg.into(), KEEP, KEEP, Z_BOOT, self.config.stand_seat_speed);
        }
        self.wait_all_reach().await;
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
        self.wait_all_reach().await;
    }

    pub async fn step_forward(&mut self, times: u8) {
        let mut speed = self.config.leg_move_speed;

        for _ in 0..times {
            let front_right_pos =
                ROBOT_STATE.lock(|state| state.borrow().current_pos[Leg::FrontRight][1]);
            if front_right_pos == Y_START {
                self.set_site(Leg::FrontRight, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.debug_positions();
                self.wait_all_reach().await;
                self.set_site(
                    Leg::FrontRight,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_UP,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;
                self.set_site(
                    Leg::FrontRight,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_DEFAULT,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;

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
                self.debug_positions();
                self.wait_all_reach().await;

                speed = self.config.leg_move_speed;
                self.set_site(
                    Leg::BottomLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_UP,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;
                self.set_site(Leg::BottomLeft, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.debug_positions();
                self.wait_all_reach().await;
                self.set_site(
                    Leg::BottomLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START,
                    Z_DEFAULT,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;
            } else {
                self.set_site(Leg::FrontLeft, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.debug_positions();
                self.wait_all_reach().await;
                self.set_site(
                    Leg::FrontLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_UP,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;
                self.set_site(
                    Leg::FrontLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_DEFAULT,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;

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
                self.debug_positions();
                self.wait_all_reach().await;

                speed = self.config.leg_move_speed;
                self.set_site(
                    Leg::BottomRight,
                    X_DEFAULT + X_OFFSET,
                    Y_START + 2.0 * Y_STEP,
                    Z_UP,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;
                self.set_site(Leg::BottomRight, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.debug_positions();
                self.wait_all_reach().await;
                self.set_site(
                    Leg::BottomRight,
                    X_DEFAULT + X_OFFSET,
                    Y_START,
                    Z_DEFAULT,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;
            }
        }
    }

    pub async fn step_backward(&mut self, times: u8) {
        for _ in 0..times {
            todo!()
        }
    }

    pub async fn turn_left(&mut self, times: i32) {
        let speed = self.config.spot_turn_speed;

        for _ in 0..times {
            let bottom_right_pos =
                ROBOT_STATE.lock(|state| state.borrow().current_pos[Leg::BottomRight][1]);
            if bottom_right_pos == Y_START {
                // Leg 3 & 1 move
                self.set_site(Leg::BottomRight, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.debug_positions();
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontLeft,
                    self.config.turn_x1 - X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomLeft,
                    self.config.turn_x0 - X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::FrontRight,
                    self.config.turn_x1 + X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomRight,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_UP,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;

                self.set_site(
                    Leg::BottomRight,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontLeft,
                    self.config.turn_x1 + X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomLeft,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::FrontRight,
                    self.config.turn_x1 - X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomRight,
                    self.config.turn_x0 - X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;

                self.set_site(
                    Leg::BottomLeft,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_UP,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(Leg::BottomLeft, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
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
                self.debug_positions();
                self.wait_all_reach().await;

                self.set_site(
                    Leg::BottomLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START,
                    Z_DEFAULT,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;
            } else {
                // Leg 0 & 2 move
                self.set_site(Leg::FrontLeft, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.debug_positions();
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontLeft,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_UP,
                    speed,
                );
                self.set_site(
                    Leg::BottomLeft,
                    self.config.turn_x1 + X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::FrontRight,
                    self.config.turn_x0 - X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomRight,
                    self.config.turn_x1 - X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontLeft,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontLeft,
                    self.config.turn_x0 - X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomLeft,
                    self.config.turn_x1 - X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::FrontRight,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomRight,
                    self.config.turn_x1 + X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontRight,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_UP,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;

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
                self.set_site(Leg::FrontRight, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.set_site(
                    Leg::BottomRight,
                    X_DEFAULT + X_OFFSET,
                    Y_START,
                    Z_DEFAULT,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontRight,
                    X_DEFAULT + X_OFFSET,
                    Y_START,
                    Z_DEFAULT,
                    speed,
                );
                self.debug_positions();
                self.wait_all_reach().await;
            }
        }
    }

    pub async fn turn_right(&mut self, times: i32) {
        let speed = self.config.spot_turn_speed;

        for _ in 0..times {
            let front_right_leg =
                ROBOT_STATE.lock(|state| state.borrow().current_pos[Leg::FrontRight][1]);

            if front_right_leg == Y_START {
                // Leg 2 (FrontRight) & 0 (FrontLeft) move
                self.set_site(Leg::FrontRight, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontLeft,
                    self.config.turn_x0 - X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomLeft,
                    self.config.turn_x1 - X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::FrontRight,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_UP,
                    speed,
                );
                self.set_site(
                    Leg::BottomRight,
                    self.config.turn_x1 + X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontRight,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontLeft,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomLeft,
                    self.config.turn_x1 + X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::FrontRight,
                    self.config.turn_x0 - X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomRight,
                    self.config.turn_x1 - X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontLeft,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_UP,
                    speed,
                );
                self.wait_all_reach().await;

                self.set_site(Leg::FrontLeft, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.set_site(
                    Leg::BottomLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START,
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
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontLeft,
                    X_DEFAULT + X_OFFSET,
                    Y_START,
                    Z_DEFAULT,
                    speed,
                );
                self.wait_all_reach().await;
            } else {
                // Leg 1 (BottomLeft) & 3 (BottomRight) move
                self.set_site(Leg::BottomLeft, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontLeft,
                    self.config.turn_x1 + X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomLeft,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_UP,
                    speed,
                );
                self.set_site(
                    Leg::FrontRight,
                    self.config.turn_x1 - X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomRight,
                    self.config.turn_x0 - X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.wait_all_reach().await;

                self.set_site(
                    Leg::BottomLeft,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.wait_all_reach().await;

                self.set_site(
                    Leg::FrontLeft,
                    self.config.turn_x1 - X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomLeft,
                    self.config.turn_x0 - X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::FrontRight,
                    self.config.turn_x1 + X_OFFSET,
                    self.config.turn_y1,
                    Z_DEFAULT,
                    speed,
                );
                self.set_site(
                    Leg::BottomRight,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_DEFAULT,
                    speed,
                );
                self.wait_all_reach().await;

                self.set_site(
                    Leg::BottomRight,
                    self.config.turn_x0 + X_OFFSET,
                    self.config.turn_y0,
                    Z_UP,
                    speed,
                );
                self.wait_all_reach().await;

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
                self.set_site(Leg::BottomRight, X_DEFAULT + X_OFFSET, Y_START, Z_UP, speed);
                self.wait_all_reach().await;

                self.set_site(
                    Leg::BottomRight,
                    X_DEFAULT + X_OFFSET,
                    Y_START,
                    Z_DEFAULT,
                    speed,
                );
                self.wait_all_reach().await;
            }
        }
    }

    /// Move front right leg or front left depending on?
    pub async fn wave(&mut self, times: u8) {
        let (x_tmp, y_tmp, z_tmp);
        let leg;
        let speed = self.config.body_move_speed;
        let current_pos = ROBOT_STATE.lock(|state| state.borrow().current_pos);

        if current_pos[Leg::BottomRight][1] == Y_START {
            leg = Leg::FrontRight;
            self.body_right(15).await;
            x_tmp = current_pos[leg][0];
            y_tmp = current_pos[leg][1];
            z_tmp = current_pos[leg][2];

            for _ in 0..times {
                self.set_site(leg, self.config.turn_x1, self.config.turn_y1, 50.0, speed);
                self.wait_all_reach().await;
                self.set_site(leg, self.config.turn_x0, self.config.turn_y0, 50.0, speed);
                self.wait_all_reach().await;
            }
            self.set_site(leg, x_tmp, y_tmp, z_tmp, speed);
            self.wait_all_reach().await;
            self.body_left(15).await;
        } else {
            leg = Leg::FrontLeft;
            self.body_left(15).await;
            x_tmp = current_pos[leg][0];
            y_tmp = current_pos[leg][1];
            z_tmp = current_pos[leg][2];
            self.config.move_speed = self.config.body_move_speed; //update the move speed before
                                                                  //setting sites

            for _ in 0..times {
                self.set_site(leg, self.config.turn_x1, self.config.turn_y1, 50.0, speed);
                self.wait_all_reach().await;
                self.set_site(leg, self.config.turn_x0, self.config.turn_y0, 50.0, speed);
                self.wait_all_reach().await;
            }
            self.set_site(leg, x_tmp, y_tmp, z_tmp, speed);
            self.wait_all_reach().await;
            self.config.move_speed = 1.0;
            self.body_right(15).await;
        }
        debug!("[MOTION TASK] wave completed!")
    }

    async fn body_left(&mut self, i: i32) {
        let speed = self.config.move_speed;
        let current_pos = ROBOT_STATE.lock(|state| state.borrow().current_pos);

        self.set_site(
            Leg::FrontLeft,
            current_pos[0][0] + i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.set_site(
            Leg::BottomLeft,
            current_pos[1][0] + i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.set_site(
            Leg::FrontRight,
            current_pos[2][0] - i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.set_site(
            Leg::BottomRight,
            current_pos[3][0] - i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.wait_all_reach().await;
    }

    async fn body_right(&mut self, i: i32) {
        let speed = self.config.move_speed;
        let current_pos = ROBOT_STATE.lock(|state| state.borrow().current_pos);

        self.set_site(
            Leg::FrontLeft,
            current_pos[0][0] - i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.set_site(
            Leg::BottomLeft,
            current_pos[1][0] - i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.set_site(
            Leg::FrontRight,
            current_pos[2][0] + i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.set_site(
            Leg::BottomRight,
            current_pos[3][0] + i as f32,
            KEEP,
            KEEP,
            speed,
        );
        self.wait_all_reach().await;
    }
}
