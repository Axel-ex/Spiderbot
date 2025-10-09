//! Joint enumeration and display helpers.
//!
//! Defines the [`Joint`] enum for identifying each joint (coxa, femur, tibia),
//! and provides display formatting for debugging and logging.
use core::fmt::Display;

#[derive(Debug, Clone, Copy)]
pub enum Joint {
    Femur = 0,
    Tibia = 1,
    Coxa = 2,
}

impl Display for Joint {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Joint::Femur => f.write_str("femur"),
            Joint::Tibia => f.write_str("tibia"),
            Joint::Coxa => f.write_str("coxa"),
        }
    }
}

impl From<usize> for Joint {
    fn from(value: usize) -> Self {
        match value {
            0 => Joint::Femur,
            1 => Joint::Tibia,
            2 => Joint::Coxa,
            _ => unreachable!(),
        }
    }
}
