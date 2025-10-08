//! Command types for robot control and inter-task communication.
//!
//! Defines enums and structs for high-level robot commands (e.g., walking, turning)
//! and low-level servo commands, as well as TCP command parsing.
//!
//! Used by the network, motion, and servo tasks.
pub enum TcpCommand {
    Calibrate,
    Test,
    Sit,
    Stand,
    Wave(u8),
    StepForward(u8),
    TurnLeft(u8),
    TurnRight(u8),
    SetAngles([u8; 12]),
}

#[derive(Debug, PartialEq, Eq)]
pub struct ParseCommandError;

impl TryFrom<&str> for TcpCommand {
    type Error = ParseCommandError;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        let mut tokens = value.trim().split_whitespace();

        let cmd = tokens.next().ok_or(ParseCommandError)?;
        let steps = tokens
            .next()
            .map(|s| s.parse::<u8>().unwrap_or(1))
            .unwrap_or(1);

        match cmd {
            "t" => Ok(TcpCommand::Test),
            "w" => Ok(TcpCommand::Wave(steps)),
            "d" => Ok(TcpCommand::StepForward(steps)),
            "r" => Ok(TcpCommand::Sit),
            "s" => Ok(TcpCommand::Stand),
            "c" => Ok(TcpCommand::Calibrate),
            "tl" => Ok(TcpCommand::TurnLeft(steps)),
            "tr" => Ok(TcpCommand::TurnRight(steps)),
            _ => Err(ParseCommandError),
        }
    }
}

pub struct ServoCommand {
    pub current_pos: [[f32; 3]; 4],
    pub expected_pos: [[f32; 3]; 4],
    pub temp_speed: [[f32; 3]; 4],
}

impl ServoCommand {
    pub fn new(
        current_pos: [[f32; 3]; 4],
        expected_pos: [[f32; 3]; 4],
        temp_speed: [[f32; 3]; 4],
    ) -> Self {
        Self {
            current_pos,
            expected_pos,
            temp_speed,
        }
    }
}
