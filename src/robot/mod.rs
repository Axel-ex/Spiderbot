//! Core robot types and configuration.
//!
//! This module defines the main types and constants for Spiderbot, including:
//! - [`commands`]: Command types for inter-task communication (TCP and servo).
//! - [`config`]: Physical and movement constants for the robot.
//! - [`leg`]: Leg enumeration and indexing helpers.
//! - [`joint`]: Joint enumeration and display helpers.
//!
//! These types are used throughout the firmware for movement, configuration, and control.
pub mod commands;
pub mod config;
pub mod joint;
pub mod leg;
