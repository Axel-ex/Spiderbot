//! Asynchronous tasks for Spiderbot operation.
//!
//! This module contains Embassy async tasks for the robot's runtime, including:
//! - [`motion_task`]: Handles high-level motion commands and gait execution.
//! - [`servo_task`]: Drives the servo controller to move legs as commanded.
//! - [`net_task`]: Manages WiFi, TCP server, and command reception.
//!
//! Tasks are spawned from `main.rs` and communicate via Embassy channels.
pub mod motion_task;
pub mod net_task;
pub mod servo_task;
