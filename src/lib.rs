//! Library root for Spiderbot firmware.
//!
//! Re-exports all main modules: [`kinematics`], [`robot`], and [`tasks`].
//! Used by the main binary and for integration in tests or other binaries.
#![no_std]

pub mod kinematics;
pub mod robot;
pub mod tasks;

pub const SERVOCMD_CHANNEL_SIZE: usize = 4;
pub const TCPCMD_CHANNEL_SIZE: usize = 4;
