use micromath::F32Ext;

// ROBOT SIZE
pub const LENGTH_A: f32 = 55.0;
pub const LENGTH_B: f32 = 77.5;
pub const LENGTH_C: f32 = 27.5;
pub const LENGTH_SIDE: f32 = 71.0;
pub const Z_ABSOLUTE: f32 = -28.0;

///CONST FOR MOVEMENT
pub const Z_DEFAULT: f32 = -50.0;
pub const Z_UP: f32 = -30.0;
pub const Z_BOOT: f32 = Z_ABSOLUTE;
pub const X_DEFAULT: f32 = 62.0;
pub const X_OFFSET: f32 = 0.0;
pub const Y_START: f32 = 0.0;
pub const Y_STEP: f32 = 40.0;

/// functions parameter
const KEEP: f32 = 255.0;

/// Stores the constant that need runtime op like sqrt or cos
#[derive(Debug)]
pub struct RobotConfig {
    pub temp_a: f32,
    pub temp_b: f32,
    pub temp_c: f32,
    pub temp_alpha: f32,
    pub turn_x1: f32,
    pub turn_y1: f32,
    pub turn_x0: f32,
    pub turn_y0: f32,
}

impl RobotConfig {
    pub fn new() -> Self {
        let temp_a = ((2.0 * X_DEFAULT + LENGTH_SIDE).powi(2) + Y_STEP.powi(2)).sqrt();
        let temp_b = 2.0 * (Y_START + Y_STEP) + LENGTH_SIDE;
        let temp_c = ((2.0 * X_DEFAULT + LENGTH_SIDE).powi(2)
            + (2.0 * Y_START + Y_STEP + LENGTH_SIDE).powi(2))
        .sqrt();
        let temp_alpha =
            ((temp_a.powi(2) + temp_b.powi(2) - temp_c.powi(2)) / 2.0 / temp_a / temp_b).acos();

        let turn_x1 = (temp_a - LENGTH_SIDE) / 2.0;
        let turn_y1 = (Y_START + Y_STEP) / 2.0;
        let turn_x0 = turn_x1 - temp_b * temp_a.cos();
        let turn_y0 = temp_b * temp_alpha.sin() - turn_y1 - LENGTH_SIDE;

        Self {
            temp_a,
            temp_b,
            temp_c,
            temp_alpha,
            turn_x1,
            turn_y1,
            turn_x0,
            turn_y0,
        }
    }
}
