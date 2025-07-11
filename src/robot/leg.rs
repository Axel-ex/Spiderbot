use core::fmt::Display;
use core::ops::{Index, IndexMut};

use super::servo::AnyServo;

#[derive(Debug, Clone, Copy)]
pub enum Leg {
    FrontLeft = 0,
    BottomLeft = 1,
    FrontRight = 2,
    BottomRight = 3,
}

impl Display for Leg {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Leg::FrontLeft => f.write_str("Front left"),
            Leg::FrontRight => f.write_str("Front right"),
            Leg::BottomLeft => f.write_str("Bottom left"),
            Leg::BottomRight => f.write_str("Bottom right"),
        }
    }
}

impl From<usize> for Leg {
    fn from(value: usize) -> Self {
        match value {
            0 => Leg::FrontLeft,
            1 => Leg::BottomLeft,
            2 => Leg::FrontRight,
            3 => Leg::BottomRight,
            _ => Leg::BottomRight,
        }
    }
}

//TODO: implement Index<Leg> for [[f32; 3]; 4]
impl Index<Leg> for [[f32; 3]; 4] {
    type Output = [f32; 3];

    fn index(&self, leg: Leg) -> &Self::Output {
        &self[leg as usize]
    }
}

impl IndexMut<Leg> for [[f32; 3]; 4] {
    fn index_mut(&mut self, leg: Leg) -> &mut Self::Output {
        &mut self[leg as usize]
    }
}

impl Index<Leg> for [[AnyServo; 3]; 4] {
    type Output = [AnyServo; 3];

    fn index(&self, leg: Leg) -> &Self::Output {
        &self[leg as usize]
    }
}

impl IndexMut<Leg> for [[AnyServo; 3]; 4] {
    fn index_mut(&mut self, leg: Leg) -> &mut Self::Output {
        &mut self[leg as usize]
    }
}
