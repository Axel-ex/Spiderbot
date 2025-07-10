use core::fmt::Display;

#[derive(Debug)]
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
