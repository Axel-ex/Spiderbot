pub enum Command {
    Sit,
    Wave,
    StepForward(i32),
    TurnLeft(i32),
    TurnRight(i32),
    SetAngles([u8; 12]),
}

#[derive(Debug, PartialEq, Eq)]
pub struct ParseCommandError;

impl TryFrom<&str> for Command {
    type Error = ParseCommandError;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        let mut tokens = value.trim().split_whitespace();

        match tokens.next() {
            Some("") => Ok(Command::Sit),
            _ => Err(ParseCommandError),
        }
    }
}
