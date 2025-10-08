//! Kinematics algorithms and servo conversion for Spiderbot.
//!
//! This module provides the mathematical routines for converting between Cartesian
//! coordinates and joint angles, as well as routines for generating and sequencing
//! leg movements (gaits).
//!
//! - [`conversion`] handles forward/inverse kinematics and servo pulse mapping.
//! - [`gait_engine`] implements the state machine for coordinated leg movement.
//!
//! Used by the motion task to plan and execute robot movement.
pub mod conversion;
pub mod gait_engine;
