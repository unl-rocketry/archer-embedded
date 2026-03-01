#![no_std]

use pololu_tic::variables::StepMode;

pub const STEPS_PER_DEGREE_VERTICAL: u32 =
    (23.3 * tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as f32) as u32;
pub const STEPS_PER_DEGREE_HORIZONTAL: u32 =
    (126.0 * tic_step_mult(DEFAULT_STEP_MODE_HORIZONTAL) as f32) as u32;
pub const SPEED_VERYSLOW: i32 = 20000 * tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as i32; //only used on CALV so no need to add a second one
pub const SPEED_DEFAULT_VERTICAL: i32 = 15000000 * tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as i32;
pub const SPEED_DEFAULT_HORIZONTAL: i32 =
    10000000 * tic_step_mult(DEFAULT_STEP_MODE_HORIZONTAL) as i32;
pub const SPEED_MAX_VERTICAL: u32 = 15000000 * tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as u32;
pub const SPEED_MAX_HORIZONTAL: u32 = 10000000 * tic_step_mult(DEFAULT_STEP_MODE_HORIZONTAL) as u32;

pub const fn tic_step_mult(step_mode: StepMode) -> u16 {
    match step_mode {
        StepMode::Full => 1,
        StepMode::Half => 2,
        StepMode::Microstep2_100p => 2,
        StepMode::Microstep4 => 4,
        StepMode::Microstep8 => 8,
        StepMode::Microstep16 => 16,
        StepMode::Microstep32 => 32,
        StepMode::Microstep64 => 64,
        StepMode::Microstep128 => 128,
        StepMode::Microstep256 => 256,
    }
}

pub const TIC_DECEL_DEFAULT_VERTICAL: u32 =
    400000 * (tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as u32 / 2);
pub const TIC_DECEL_DEFAULT_HORIZONTAL: u32 =
    300000 * tic_step_mult(DEFAULT_STEP_MODE_HORIZONTAL) as u32;

pub const DEFAULT_CURRENT: u16 = 1024;

pub const DEFAULT_STEP_MODE_VERTICAL: StepMode = StepMode::Microstep16;
pub const DEFAULT_STEP_MODE_HORIZONTAL: StepMode = StepMode::Microstep8;

// Offsets calculated manually from accelerometer data
pub const ACC_OFFSET_X: i16 = 53 / 8;
pub const ACC_OFFSET_Y: i16 = 83 / 8;
pub const ACC_OFFSET_Z: i16 = -154 / 8;

/// Calculate most optimal difference in current and destination angle
pub fn get_delta_angle(curr_angle: f32, new_angle: f32) -> f32 {
    let diff: f32 = ((new_angle - curr_angle + 180.0) % 360.0) - 180.0;
    if diff < -180.0 {
        diff + 360.0 //if angle less than -180 switch directions
    } else {
        diff
    }
}

pub fn get_relative_angle(curr_position: f32) -> f32 {
    let mut curr_angle: f32 = curr_position / STEPS_PER_DEGREE_HORIZONTAL as f32;

    while curr_angle > 180.0 {
        curr_angle -= 360.0;
    }
    while curr_angle < 180.0 {
        curr_angle += 360.0;
    }
    curr_angle
}
