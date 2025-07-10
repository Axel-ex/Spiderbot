pub enum TcpCommand {
    Sit,
    Stand,
    Wave(u8),
    StepForward(u8),
    TurnLeft(i32),
    TurnRight(i32),
    SetAngles([u8; 12]),
}

#[derive(Debug, PartialEq, Eq)]
pub struct ParseCommandError;

// TODO: Implement the parsing logic
impl TryFrom<&str> for TcpCommand {
    type Error = ParseCommandError;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        let mut tokens = value.trim().split_whitespace();

        match tokens.next() {
            Some("w") => Ok(TcpCommand::Wave(1)),
            Some("d") => Ok(TcpCommand::StepForward(1)),
            Some("r") => Ok(TcpCommand::Sit),
            Some("s") => Ok(TcpCommand::Stand),
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
