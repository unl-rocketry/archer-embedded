use crate::{
    calibrate_vertical, get_delta_angle, get_relative_angle, SPEED_DEFAULT_HORIZONTAL,
    SPEED_DEFAULT_VERTICAL, SPEED_MAX_HORIZONTAL, SPEED_MAX_VERTICAL, STEPS_PER_DEGREE_HORIZONTAL,
    STEPS_PER_DEGREE_VERTICAL,
};
use alloc::format;
use alloc::string::{String, ToString};
use esp_println::println;
use mma8x5x::ic::Mma8451;
use mma8x5x::{mode, Mma8x5x};
use pololu_tic::{TicBase, TicI2C};

#[derive(Debug, thiserror::Error)]
pub enum ParseErr {
    #[error("the command was empty")]
    Empty,
    #[error("the command provided was invalid")]
    InvalidCommand,
    #[error("the argument provided was invalid")]
    InvalidArgument,
    #[error("the number could not be parsed")]
    InvalidNumber,
    #[error("not enough arguments were provided")]
    TooFewArguments,
    #[error("the motor controllers experienced an error")]
    InternalError(#[from] pololu_tic::TicHandlerError),
    #[error("the vertical position has not been calibrated.")]
    Uncalibrated,
    #[error("Command Locked")]
    CommandLockoutError
}

const BLACKLIST: &[&str] = &["DVER", "DHOR"];

pub async fn parse_command<I: embedded_hal::i2c::I2c>(
    motor_vertical: &mut TicI2C<I>,
    motor_horizontal: &mut TicI2C<I>,
    accel: &mut Option<Mma8x5x<I, Mma8451, mode::Active>>,
    input: &str,
    is_calibrated: &mut bool,
) -> Result<String, ParseErr> {
    let input = input.to_ascii_uppercase();
    let mut arguments = input.split_whitespace().peekable();

    if !*is_calibrated && arguments.peek().is_some_and(|a| BLACKLIST.contains(a)) {
        return Err(ParseErr::Uncalibrated);
    }

    match arguments.next().ok_or(ParseErr::Empty)? {
        "DVER" => {
            let target_pos = match arguments
                .next()
                .ok_or(ParseErr::TooFewArguments)?
                .parse::<f32>()
            {
                Ok(n) => n.clamp(0.0, 90.0),
                _ => return Err(ParseErr::InvalidNumber),
            };

            motor_vertical
                .set_target_position((target_pos * STEPS_PER_DEGREE_VERTICAL as f32) as i32)?;
        }
        "DHOR" => {
            let target_pos = match arguments
                .next()
                .ok_or(ParseErr::TooFewArguments)?
                .parse::<f32>()
            {
                Ok(n) => n.clamp(-180.0, 180.0),
                _ => return Err(ParseErr::InvalidNumber),
            };
            let angle_steps = get_delta_angle(get_relative_angle(motor_horizontal), target_pos)
                * STEPS_PER_DEGREE_HORIZONTAL as f32;
            let move_to = motor_horizontal.current_position()? as f32 + angle_steps;
            motor_horizontal.set_target_position(move_to as i32)?;
        }
        "CALV" => match arguments.next() {
            Some("SET") => {
                motor_vertical.halt_and_set_position(0)?;
                *is_calibrated = true;
            }
            _ => {
                match accel {
                    Some(accel) => {
                        calibrate_vertical(motor_vertical, accel).await;
                        *is_calibrated = true
                    }
                    None => {
                        return Err(ParseErr::InvalidCommand)
                    }
                }
            }
        },
        "CALH" => {
            motor_horizontal.halt_and_set_position(0)?;
        }
        "MOVV" => {
            let steps_to_move = match arguments
                .next()
                .ok_or(ParseErr::TooFewArguments)?
                .parse::<f32>()
            {
                Ok(n) => n,
                Err(_) => Err(ParseErr::TooFewArguments)?,
            };
            let current_position = motor_vertical.current_position()? as f32;
            let move_to = current_position + steps_to_move;
            motor_vertical.set_target_position(move_to as i32)?;
        }
        "MOVH" => {
            let steps_to_move = match arguments
                .next()
                .ok_or(ParseErr::TooFewArguments)?
                .parse::<f32>()
            {
                Ok(n) => n,
                Err(_) => Err(ParseErr::TooFewArguments)?,
            };
            let current_position = motor_horizontal.current_position()? as f32;
            let move_to = current_position - steps_to_move;
            motor_horizontal.set_target_position(move_to as i32)?;
        }
        "GETP" => {
            let vertical_position: f32 =
                motor_vertical.current_position()? as f32 / STEPS_PER_DEGREE_VERTICAL as f32;
            let mut horizontal_position: f32 =
                motor_horizontal.current_position()? as f32 / STEPS_PER_DEGREE_HORIZONTAL as f32;
            while horizontal_position > 180.0 {
                horizontal_position -= 360.0;
            }
            while horizontal_position < -180.0 {
                horizontal_position += 360.0;
            }
            return Ok(format!("{} {}", vertical_position, horizontal_position));
        }
        "INFO" => {
            let command_list = [
                "DVER INT",
                "DHOR INT",
                "CALV {SET}",
                "CALH",
                "MOVV INT",
                "MOVH INT",
                "GETP",
                "SSPD INT {VER INT} {HOR INT} {RST}",
                "GSPD",
                "HALT",
                "VERS",
                "GETC",
                "INFO",
            ];

            for command in command_list {
                println!("  {}", command);
            }
        }
        "GETC" => {
            println!("{}", is_calibrated);
        }
        "VERS" => {
            println!("{}", env!("CARGO_PKG_VERSION"));
        }
        "SSPD" => match arguments.next() {
            Some("VER") => {
                let new_speed = match arguments
                    .next()
                    .ok_or(ParseErr::TooFewArguments)?
                    .parse::<f32>()
                {
                    Ok(n) => n,
                    Err(_) => Err(ParseErr::TooFewArguments)?,
                };
                motor_vertical
                    .set_max_speed(new_speed.clamp(0.0, SPEED_MAX_VERTICAL as f32) as u32)?;
            }
            Some("HOR") => {
                let new_speed = match arguments
                    .next()
                    .ok_or(ParseErr::TooFewArguments)?
                    .parse::<f32>()
                {
                    Ok(n) => n,
                    Err(_) => Err(ParseErr::TooFewArguments)?,
                };
                motor_horizontal
                    .set_max_speed(new_speed.clamp(0.0, SPEED_MAX_HORIZONTAL as f32) as u32)?;
            }
            Some("RST") => {
                motor_vertical.set_max_speed(SPEED_DEFAULT_VERTICAL as u32)?;
                motor_horizontal.set_max_speed(SPEED_DEFAULT_HORIZONTAL as u32)?;
            }
            Some(int) => {
                let new_speed = match int.parse::<f32>() {
                    Ok(n) => n,
                    Err(_) => Err(ParseErr::TooFewArguments)?,
                };
                motor_vertical
                    .set_max_speed(new_speed.clamp(0.0, SPEED_MAX_VERTICAL as f32) as u32)?;
                motor_horizontal
                    .set_max_speed(new_speed.clamp(0.0, SPEED_MAX_HORIZONTAL as f32) as u32)?;
            }
            _ => return Err(ParseErr::InvalidArgument),
        },
        "GSPD" => {
            return Ok(format!(
                "{} {}",
                motor_vertical.max_speed()?,
                motor_horizontal.max_speed()?
            ));
        }
        "HALT" => {
            motor_vertical.halt_and_hold()?;
            motor_horizontal.halt_and_hold()?
        }
        _ => Err(ParseErr::InvalidCommand)?,
    }
    Ok("".to_string())
}
