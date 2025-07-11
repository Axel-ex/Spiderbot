use core::fmt::Display;
use core::ops::{Index, IndexMut};

use super::servo::AnyServo;

#[derive(Debug, Clone, Copy)]
pub enum Joint {
    Coxa = 0,
    Femur = 1,
    Tibia = 2,
}

impl Display for Joint {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Joint::Coxa => f.write_str("coxa"),
            Joint::Femur => f.write_str("femur"),
            Joint::Tibia => f.write_str("tibia"),
        }
    }
}

impl From<usize> for Joint {
    fn from(value: usize) -> Self {
        match value {
            0 => Joint::Femur,
            1 => Joint::Tibia,
            2 => Joint::Coxa,
            _ => Joint::Coxa, // default that should not occur
        }
    }
}

impl Index<Joint> for [AnyServo; 3] {
    type Output = AnyServo;

    fn index(&self, joint: Joint) -> &Self::Output {
        &self[joint as usize]
    }
}

impl IndexMut<Joint> for [AnyServo; 3] {
    fn index_mut(&mut self, joint: Joint) -> &mut Self::Output {
        &mut self[joint as usize]
    }
}
