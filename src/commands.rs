use crate::{setup_motor, TicProduct, ACC_OFFSET_X, ACC_OFFSET_Y, ACC_OFFSET_Z};
use crate::{calibrate_vertical, get_delta_angle, get_relative_angle, CalibrationStatus, Control, MotorAxis, Position, Status, SPEED_DEFAULT_HORIZONTAL, SPEED_DEFAULT_VERTICAL, SPEED_MAX_HORIZONTAL, SPEED_MAX_VERTICAL, STEPS_PER_DEGREE_HORIZONTAL, STEPS_PER_DEGREE_VERTICAL};
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::rwlock::RwLock;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_bus::i2c::RefCellDevice;
use embedded_hal_bus::spi::NoDelay;
use esp_hal::uart::Uart;
use esp_hal::{i2c};
use esp_hal::i2c::master::I2c;
use esp_println::{print, println};
use log::{error, info};
use mma8x5x::{GScale, Mma8x5x, OutputDataRate, PowerMode};
use pololu_tic::{I2c as TicI2C, TicBase};
use core::cell::RefCell;
use esp_hal::peripherals::{GPIO1, GPIO18, GPIO19, GPIO3, I2C0, UART0};

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
    InternalError(#[from] pololu_tic::HandlerError),
    #[error("the vertical position has not been calibrated.")]
    Uncalibrated,
    #[error("The acceleromter was not found. Use CALV SET instead")]
    NoAccel,
}

const BLACKLIST: &[&str] = &["DVER", "DHOR"];

#[embassy_executor::task]
pub async fn control_loop(
    sda: GPIO18<'static>,
    scl: GPIO19<'static>,
    i2c0: I2C0<'static>,
    command_channel: Receiver<'static, CriticalSectionRawMutex, Control, 50>,
    e_stop_channel: Receiver<'static, CriticalSectionRawMutex, Control, 1>,
    position: Arc<RwLock<CriticalSectionRawMutex, Position>>,
    status: Arc<RwLock<CriticalSectionRawMutex, Status>>,
) {
    let i2c_bus = I2c::new(
        i2c0,
        i2c::master::Config::default().with_timeout(i2c::master::BusTimeout::Maximum),
    )
        .unwrap()
        .with_sda(sda)
        .with_scl(scl)
        .into_async();
    let i2c_bus = RefCell::new(i2c_bus);

    let mut motor_horizontal = TicI2C::new_with_address(
        RefCellDevice::new(&i2c_bus),
        TicProduct::Tic36v4,
        NoDelay,
        14,
    );
    let mut motor_vertical = TicI2C::new_with_address(
        RefCellDevice::new(&i2c_bus),
        TicProduct::Tic36v4,
        NoDelay,
        15,
    );

    let mut accelerometer = Mma8x5x::new_mma8451(
        RefCellDevice::new(&i2c_bus),
        mma8x5x::SlaveAddr::Alternative(true),
    );
    let _ = accelerometer.disable_auto_sleep();
    let _ = accelerometer.set_scale(GScale::G2);
    let _ = accelerometer.set_data_rate(OutputDataRate::Hz50);
    let _ = accelerometer.set_wake_power_mode(PowerMode::HighResolution);
    let _ = accelerometer.set_read_mode(mma8x5x::ReadMode::Normal);
    let _ = accelerometer.set_offset_correction(
        ACC_OFFSET_X as i8,
        ACC_OFFSET_Y as i8,
        ACC_OFFSET_Z as i8,
    );

    let mut accel = if let Ok(a) = accelerometer.into_active() {
        info!("MMA8451 set up!!");
        Some(a)
    } else {
        None
    };

    setup_motor(&mut motor_horizontal, MotorAxis::Horizontal)
        .expect("Horizontal motor setup error");
    setup_motor(&mut motor_vertical, MotorAxis::Vertical).expect("Vertical motor setup error");
    info!("Motors set up!!");

    let mut stat = status.write().await;
    stat.vertical_speed = motor_vertical.max_speed().unwrap();
    stat.horizontal_speed = motor_horizontal.max_speed().unwrap();
    drop(stat);

    let mut pos = position.write().await;
    pos.vertical_position = motor_vertical.current_position().unwrap() as f32 / STEPS_PER_DEGREE_VERTICAL as f32;
    pos.horizontal_position = motor_horizontal.current_position().unwrap() as f32 / STEPS_PER_DEGREE_HORIZONTAL as f32;
    drop(pos);


    let mut position_clock = Timer::after(Duration::from_millis(200));
    let mut timer = Instant::now();

    loop {
        if timer.elapsed() > Duration::from_millis(100) {
            while motor_horizontal.reset_command_timeout().is_err() {
                error!("Horizontal motor communication failure, attempting reconnection");
                motor_horizontal = TicI2C::new_with_address(
                    RefCellDevice::new(&i2c_bus),
                    TicProduct::Tic36v4,
                    NoDelay,
                    14,
                );

                let _ = setup_motor(&mut motor_horizontal, MotorAxis::Horizontal);
                Timer::after(Duration::from_secs(1)).await;
            }

            while motor_vertical.reset_command_timeout().is_err() {
                error!("Vertical motor communication failure, attempting reconnection");
                motor_vertical = TicI2C::new_with_address(
                    RefCellDevice::new(&i2c_bus),
                    TicProduct::Tic36v4,
                    NoDelay,
                    15,
                );

                let _ = setup_motor(&mut motor_vertical, MotorAxis::Vertical);
                Timer::after(Duration::from_secs(1)).await;
            }

            timer = Instant::now();
        }
        match select(command_channel.receive(), &mut position_clock).await {
            Either::First(cmd) => {
                if !e_stop_channel.is_empty() {
                    motor_vertical.halt_and_hold().unwrap();
                    motor_horizontal.halt_and_hold().unwrap();
                    e_stop_channel.clear();
                }
                match cmd {
                    Control::DVER(x) => {
                        let target_pos = x.clamp(0.0, 90.0);

                        motor_vertical
                            .set_target_position((target_pos * STEPS_PER_DEGREE_VERTICAL as f32) as i32).unwrap();
                    }
                    Control::DHOR(x) => {
                        let target_pos = x.clamp(-180.0, 180.0);
                        let angle_steps = get_delta_angle(get_relative_angle(&mut motor_horizontal), target_pos)
                            * STEPS_PER_DEGREE_HORIZONTAL as f32;
                        let move_to = motor_horizontal.current_position().unwrap() as f32 + angle_steps;
                        motor_horizontal.set_target_position(move_to as i32).unwrap();
                    }
                    Control::CALV => {
                        if let Some(accel) = &mut accel {
                            status.write().await.calibration_status(CalibrationStatus::Calibrating);
                            calibrate_vertical(&mut motor_vertical, accel).await;
                            status.write().await.calibration_status(CalibrationStatus::Calibrated);
                        } else {
                            // return Err(ParseErr::NoAccel)
                        }
                    },
                    Control::CALV_SET => {
                        motor_vertical.halt_and_set_position(0).unwrap();
                        status.write().await.calibration_status(CalibrationStatus::Calibrated);
                    },
                    Control::CALH => {
                        motor_horizontal.halt_and_set_position(0).unwrap();
                    },
                    Control::MOVC_UP => {
                        motor_vertical.set_target_velocity(SPEED_DEFAULT_HORIZONTAL / 2).unwrap();
                    },
                    Control::MOVC_DN => {
                        motor_vertical.set_target_velocity(-SPEED_DEFAULT_HORIZONTAL / 2).unwrap();
                    },
                    Control::MOVC_LT => {
                        motor_horizontal.set_target_velocity(SPEED_DEFAULT_HORIZONTAL / 2).unwrap();
                    },
                    Control::MOVC_RT => {
                        motor_horizontal.set_target_velocity(-SPEED_DEFAULT_HORIZONTAL / 2).unwrap();
                    },
                    Control::MOVC_SV => {
                        motor_vertical.set_target_velocity(0).unwrap();
                    },
                    Control::MOVC_SH => {
                        motor_horizontal.set_target_velocity(0).unwrap();
                    },
                    Control::MOVV(x) => {
                        let steps_to_move = x;
                        let current_position = motor_vertical.current_position().unwrap() as f32;
                        let move_to = current_position + steps_to_move;
                        motor_vertical.set_target_position(move_to as i32).unwrap();
                    },
                    Control::MOVH(x) => {
                        let steps_to_move = x;
                        let current_position = motor_horizontal.current_position().unwrap() as f32;
                        let move_to = current_position - steps_to_move;
                        motor_horizontal.set_target_position(move_to as i32).unwrap();
                    },
                    Control::SSPD => {
                        //Handles SSPD and SSPD_RST because the command parsing updates status with the correct values for both and the code to apply it is the same.
                        let status = *status.read().await;
                        motor_vertical.set_max_speed(status.vertical_speed).unwrap();
                        motor_horizontal.set_max_speed(status.horizontal_speed).unwrap();
                    },
                    Control::SSPD_VER => {
                        let x = status.read().await.vertical_speed;
                        motor_vertical.set_max_speed(x).unwrap();
                    },
                    Control::SSPD_HOR => {
                        let x = status.read().await.horizontal_speed;
                        motor_horizontal.set_max_speed(x).unwrap();
                    },
                    Control::HALT => {
                        motor_vertical.halt_and_hold().unwrap();
                        motor_horizontal.halt_and_hold().unwrap();
                    }
                }
            }
            Either::Second(()) => {
                let pos_v: f32 = motor_vertical.current_position().unwrap() as f32 / STEPS_PER_DEGREE_VERTICAL as f32;
                let pos_h: f32 = motor_horizontal.current_position().unwrap() as f32 / STEPS_PER_DEGREE_HORIZONTAL as f32;

                let position = position.write().await;
                position.set_position(MotorAxis::Vertical, pos_v);
                position.set_position(MotorAxis::Horizontal, pos_h);
                drop(position);

                position_clock = Timer::after(Duration::from_millis(200));
            }
        }
    }
}

pub async fn command_loop (
    tx_pin: GPIO1<'_>,
    rx_pin: GPIO3<'_>,
    uart0: UART0<'_>,
    position: Arc<RwLock<CriticalSectionRawMutex, Position>>,
    status: Arc<RwLock<CriticalSectionRawMutex, Status>>,
    command_send: Sender<'_, CriticalSectionRawMutex, Control, 50>,
    e_stop_send: Sender<'_, CriticalSectionRawMutex, Control, 1>
) {
    let config = esp_hal::uart::Config::default().with_rx(
        esp_hal::uart::RxConfig::with_fifo_full_threshold(Default::default(), 64),
    );

    let mut uart0 = Uart::new(uart0, config)
        .unwrap()
        .with_tx(tx_pin)
        .with_rx(rx_pin)
        .into_async();

    uart0.set_at_cmd(esp_hal::uart::AtCmdConfig::default().with_cmd_char(0x04));

    let mut buffer = [0; 1];
    let mut command_string = String::new();

    loop {
        let Ok(count) = uart0.read_buffered(&mut buffer) else {
            continue;
        };

        // If there were no bytes read, don't try to use them
        if count == 0 {
            continue;
        }

        if buffer[0] == b'\x1B' {
            command_string.clear();
            e_stop_send.send(Control::HALT).await;
            // match parse_command(
            //     command_send,
            //     "HALT ",
            // )
            //     .await
            // {
            //     Ok(_) => print!("OK SOFTWARE E-STOP (ESC RECIEVED)\n"),
            //     Err(e) => print!("ERR: {:?}, {}\n", e, e),
            // }
            continue;
        }

        if buffer[0] == b'\r' || buffer[0] == b'\n' {
            println!();
            command_string += " ";

            match parse_command(
                &position,
                &status,
                command_send,
                e_stop_send,
                &command_string,
            ).await
            {
                Ok(s) => print!("OK {}\n", s),
                Err(e) => print!("ERR: {:?}, {}\n", e, e),
            }

            command_string.clear();

        } else if buffer[0] == b'\x08' {
            if !command_string.is_empty() {
                command_string.remove(command_string.len() - 1);
                print!("\x08 \x08");
            }

        } else if buffer[0] != 0xFF {
            print!("{}", buffer[0] as char);
            command_string.push(buffer[0] as char);
        }
    }



}

pub async fn parse_command(
    position: &Arc<RwLock<CriticalSectionRawMutex, Position>>,
    status: &Arc<RwLock<CriticalSectionRawMutex, Status>>,
    command_channel: Sender<'_, CriticalSectionRawMutex, Control, 50>,
    e_stop_send: Sender<'_, CriticalSectionRawMutex, Control, 1>,
    input: &str
) -> Result<String, ParseErr> {
    let input = input.to_ascii_uppercase();
    let mut arguments = input.split_whitespace().peekable();

    if (status.read().await.calibration_status == CalibrationStatus::Uncalibrated)
        && arguments.peek().is_some_and(|a| BLACKLIST.contains(a))
    {
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
            command_channel.send(Control::DVER(target_pos)).await;
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
            command_channel.send(Control::DHOR(target_pos)).await;
        }
        "CALV" => match arguments.next() {
            Some("SET") => {
                command_channel.send(Control::CALV_SET).await;
            }
            _ => {
                command_channel.send(Control::CALV).await;
            },
        },
        "CALH" => {
            command_channel.send(Control::CALH).await;
        }
        "MOVC" => match arguments.next() {
            Some("UP") => {
                command_channel.send(Control::MOVC_UP).await;
            }
            Some("DN") => {
                command_channel.send(Control::MOVC_DN).await;
            }
            Some("LT") => {
                command_channel.send(Control::MOVC_LT).await;
            }
            Some("RT") => {
                command_channel.send(Control::MOVC_RT).await;
            }
            Some("SV") => {
                command_channel.send(Control::MOVC_SV).await;
            }
            Some("SH") => {
                command_channel.send(Control::MOVC_SH).await;
            }
            _ => return Err(ParseErr::InvalidCommand),
        },
        "MOVV" => {
            let steps_to_move = match arguments
                .next()
                .ok_or(ParseErr::TooFewArguments)?
                .parse::<f32>()
            {
                Ok(n) => n,
                Err(_) => Err(ParseErr::TooFewArguments)?,
            };
            command_channel.send(Control::MOVV(steps_to_move)).await;
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
            command_channel.send(Control::MOVH(steps_to_move)).await;
        }



        "GETP" => {
            let position: Position = *position.read().await;
            let vertical_position = position.vertical_position;
            let mut horizontal_position = position.horizontal_position;
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
                "MOVC [UP DN LT RT SV SH]",
                "MOVV INT",
                "MOVH INT",
                "GETP",
                "GETC",
                "VERS",
                "SSPD INT {VER INT} {HOR INT} {RST}",
                "GSPD",
                "HALT",
                "INFO",
            ];

            for command in command_list {
                println!("  {}", command);
            }
        }
        "GETC" => {
            return Ok(status.read().await.calibration_status.to_string());
        }
        "VERS" => {
            return Ok(env!("PROTOCOL_VERSION").to_string());
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
                status.write().await.set_speed(MotorAxis::Vertical, new_speed.clamp(0.0, SPEED_MAX_VERTICAL as f32) as u32);
                command_channel.send(Control::SSPD_VER).await;
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
                status.write().await.set_speed(MotorAxis::Horizontal, new_speed.clamp(0.0, SPEED_MAX_HORIZONTAL as f32) as u32);
                command_channel.send(Control::SSPD_HOR).await;
            }
            Some("RST") => {
                status.write().await.set_speed(MotorAxis::Vertical, SPEED_DEFAULT_VERTICAL as u32);
                status.write().await.set_speed(MotorAxis::Horizontal, SPEED_DEFAULT_HORIZONTAL as u32);
                command_channel.send(Control::SSPD).await;
            }
            Some(int) => {
                let new_speed = match int.parse::<f32>() {
                    Ok(n) => n,
                    Err(_) => Err(ParseErr::TooFewArguments)?,
                };

                status.write().await.set_speed(MotorAxis::Vertical, new_speed.clamp(0.0, SPEED_MAX_VERTICAL as f32) as u32);
                status.write().await.set_speed(MotorAxis::Horizontal, new_speed.clamp(0.0, SPEED_MAX_HORIZONTAL as f32) as u32);
                command_channel.send(Control::SSPD).await;
            }
            _ => return Err(ParseErr::InvalidArgument),
        },
        "GSPD" => {
            return Ok(format!(
                "{} {}",
                status.read().await.vertical_speed,
                status.read().await.horizontal_speed
            ));
        }
        "HALT" => {
            e_stop_send.send(Control::HALT).await;
        }
        _ => Err(ParseErr::InvalidCommand)?,
    }

    Ok("".to_string())
}
