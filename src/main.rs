#![no_main]
#![no_std]

mod commands;

use alloc::sync::Arc;
use core::cell::RefCell;
use derive_more::Display;
use embedded_hal_bus::i2c::RefCellDevice;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    i2c::{self, master::I2c},
};

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::rwlock::RwLock;
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::NoDelay;
use log::{info, warn};
use mma8x5x::{GScale, Mma8x5x, OutputDataRate, PowerMode, ic::Mma8451, mode};
use pololu_tic::variables::StepMode as TicStepMode;
use pololu_tic::{
    HandlerError as TicHandlerError, I2c as TicI2C, Product as TicProduct, TicBase as _,
};
use crate::commands::{command_loop, control_loop};

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

const STEPS_PER_DEGREE_VERTICAL: u32 =
    (23.3 * tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as f32) as u32;
const STEPS_PER_DEGREE_HORIZONTAL: u32 =
    (126.0 * tic_step_mult(DEFAULT_STEP_MODE_HORIZONTAL) as f32) as u32;
const SPEED_VERYSLOW: i32 = 20000 * tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as i32; //only used on CALV so no need to add a second one
const SPEED_DEFAULT_VERTICAL: i32 = 15000000 * tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as i32;
const SPEED_DEFAULT_HORIZONTAL: i32 = 10000000 * tic_step_mult(DEFAULT_STEP_MODE_HORIZONTAL) as i32;
const SPEED_MAX_VERTICAL: u32 = 15000000 * tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as u32;
const SPEED_MAX_HORIZONTAL: u32 = 10000000 * tic_step_mult(DEFAULT_STEP_MODE_HORIZONTAL) as u32;

const TIC_DECEL_DEFAULT_VERTICAL: u32 =
    400000 * (tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as u32 / 2);
const TIC_DECEL_DEFAULT_HORIZONTAL: u32 =
    300000 * tic_step_mult(DEFAULT_STEP_MODE_HORIZONTAL) as u32;

const DEFAULT_CURRENT: u16 = 1024;

const DEFAULT_STEP_MODE_VERTICAL: TicStepMode = TicStepMode::Microstep16;
const DEFAULT_STEP_MODE_HORIZONTAL: TicStepMode = TicStepMode::Microstep8;

pub const fn tic_step_mult(step_mode: TicStepMode) -> u16 {
    match step_mode {
        TicStepMode::Full => 1,
        TicStepMode::Half => 2,
        TicStepMode::Microstep2_100p => 2,
        TicStepMode::Microstep4 => 4,
        TicStepMode::Microstep8 => 8,
        TicStepMode::Microstep16 => 16,
        TicStepMode::Microstep32 => 32,
        TicStepMode::Microstep64 => 64,
        TicStepMode::Microstep128 => 128,
        TicStepMode::Microstep256 => 256,
    }
}

// Offsets calculated manually from accelerometer data
const ACC_OFFSET_X: i16 = 53 / 8;
const ACC_OFFSET_Y: i16 = 83 / 8;
const ACC_OFFSET_Z: i16 = -154 / 8;

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(size: 72 * 1024);
    esp_println::logger::init_logger_from_env();
    let timer0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
    esp_rtos::start(timer0.timer0);
    info!("Embassy initialized!");

    let (sda, scl) = (peripherals.GPIO18, peripherals.GPIO19);
    let i2c0 = peripherals.I2C0;
    let (tx_pin, rx_pin) = (peripherals.GPIO1, peripherals.GPIO3);
    let uart0 = peripherals.UART0;

    let _ = spawner;

    // todo!("Spawn threads for the command loop and control loop");
    let status = Status {
        calibration_status: CalibrationStatus::Uncalibrated,
        vertical_speed: 0,
        horizontal_speed: 0,
        // vertical_speed: motor_vertical.max_speed().unwrap(),
        // horizontal_speed: motor_horizontal.max_speed().unwrap()
    };

    let position = Position {
        vertical_position: 0.0,
        horizontal_position: 0.0,
        // vertical_position: motor_vertical.current_position().unwrap() as f32 / STEPS_PER_DEGREE_VERTICAL as f32,
        // horizontal_position: motor_horizontal.current_position().unwrap() as f32 / STEPS_PER_DEGREE_HORIZONTAL as f32,
    };

    let status = Arc::new(RwLock::new(status));
    let position = Arc::new(RwLock::new(position));
    let command_channel = Channel::<CriticalSectionRawMutex, Control, 50>::new();
    let command_send = command_channel.sender();
    let command_recv = command_channel.receiver();
    let e_stop_channel = Channel::<CriticalSectionRawMutex, Control, 1>::new();
    let e_stop_send = e_stop_channel.sender();
    let e_stop_recv = e_stop_channel.receiver();
    
    control_loop(sda, scl, i2c0, command_recv, e_stop_recv, position.clone(), status.clone()).await;
    command_loop(tx_pin, rx_pin, uart0, position.clone(), status.clone(), command_send, e_stop_send).await;



//     let mut timer = Instant::now();
//
//     loop {
//         if timer.elapsed() > Duration::from_millis(100) {
//             while motor_horizontal.reset_command_timeout().is_err() {
//                 error!("Horizontal motor communication failure, attempting reconnection");
//                 motor_horizontal = TicI2C::new_with_address(
//                     RefCellDevice::new(&i2c_bus),
//                     TicProduct::Tic36v4,
//                     NoDelay,
//                     14,
//                 );
//
//                 let _ = setup_motor(&mut motor_horizontal, MotorAxis::Horizontal);
//                 Timer::after(Duration::from_secs(1)).await;
//             }
//
//             while motor_vertical.reset_command_timeout().is_err() {
//                 error!("Vertical motor communication failure, attempting reconnection");
//                 motor_vertical = TicI2C::new_with_address(
//                     RefCellDevice::new(&i2c_bus),
//                     TicProduct::Tic36v4,
//                     NoDelay,
//                     15,
//                 );
//
//                 let _ = setup_motor(&mut motor_vertical, MotorAxis::Vertical);
//                 Timer::after(Duration::from_secs(1)).await;
//             }
//
//             timer = Instant::now();
//         }
//     }
}

#[derive(Clone, Copy, PartialEq, Eq, Display)]
pub enum CalibrationStatus {
    Calibrated,
    Calibrating,
    Uncalibrated,
}

#[derive(Clone, Copy)]
pub struct Status {
    calibration_status: CalibrationStatus,
    vertical_speed: u32,
    horizontal_speed: u32,
}

impl Status {
    fn calibration_status(mut self, calibration_status: CalibrationStatus) {
        self.calibration_status = calibration_status;
    }

    fn set_speed(mut self, motor_axis: MotorAxis, speed: u32) {
        if motor_axis == MotorAxis::Vertical {
            self.vertical_speed = speed;
        } else if motor_axis == MotorAxis::Horizontal {
            self.horizontal_speed = speed;
        } else {
            warn!("Invalid motor axis provided to set_speed: {:?}", motor_axis);
        }
    }
}

#[derive(Clone, Copy)]
pub struct Position {
    vertical_position: f32,
    horizontal_position: f32,
}

impl Position {
    fn set_position(mut self, motor_axis: MotorAxis, position: f32) {
        if motor_axis == MotorAxis::Vertical {
            self.vertical_position = position;
        } else if motor_axis == MotorAxis::Horizontal {
            self.horizontal_position = position;
        } else {
            warn!(
                "Invalid motor axis provided to set_position: {:?}",
                motor_axis
            );
        }
    }
}

#[allow(non_camel_case_types)]
pub enum Control {
    DVER(f32),
    DHOR(f32),
    CALV,
    CALV_SET,
    CALH,
    MOVC_UP,
    MOVC_DN,
    MOVC_LT,
    MOVC_RT,
    MOVC_SV,
    MOVC_SH,
    MOVV(f32),
    MOVH(f32),
    SSPD,
    SSPD_VER,
    SSPD_HOR,
    HALT,
}

#[derive(PartialEq, Eq)]
#[derive(Debug)]
enum MotorAxis {
    Horizontal,
    Vertical,
}

/// Calculates pitch from MMA8451 data
fn calculate_pitch<I: embedded_hal::i2c::I2c>(
    accel: &mut Mma8x5x<I, Mma8451, mode::Active>,
) -> f32 {
    let data = accel.read().unwrap();
    let x = data.y;
    let y = data.x;
    let z = data.z;

    libm::atan2f(-x, libm::powf(y, 2.0) + libm::powf(z, 2.0))
        * (180.0 / core::f64::consts::PI as f32)
}

/// Function to set up motors
fn setup_motor<I: embedded_hal::i2c::I2c>(
    motor: &mut TicI2C<I, NoDelay>,
    motor_axis: MotorAxis,
) -> Result<(), TicHandlerError> {
    motor.set_current_limit(DEFAULT_CURRENT)?;
    motor.halt_and_set_position(0)?;

    match motor_axis {
        MotorAxis::Vertical => {
            motor.set_max_decel(TIC_DECEL_DEFAULT_VERTICAL)?;
            motor.set_max_accel(TIC_DECEL_DEFAULT_VERTICAL)?;
            motor.set_max_speed(SPEED_MAX_VERTICAL)?;
            motor.set_step_mode(DEFAULT_STEP_MODE_VERTICAL)?;
        }
        MotorAxis::Horizontal => {
            motor.set_max_decel(TIC_DECEL_DEFAULT_HORIZONTAL)?;
            motor.set_max_accel(TIC_DECEL_DEFAULT_HORIZONTAL)?;
            motor.set_max_speed(SPEED_MAX_HORIZONTAL)?;
            motor.set_step_mode(DEFAULT_STEP_MODE_HORIZONTAL)?;
        }
    }

    motor.exit_safe_start()?;

    Ok(())
}

async fn calibrate_vertical<I: embedded_hal::i2c::I2c>(
    motor: &mut TicI2C<I, NoDelay>,
    accel: &mut Mma8x5x<I, Mma8451, mode::Active>,
) {
    const ZERO_CAL: f64 = 0.2;
    let mut target_velocity: i32;
    motor.set_max_decel(5000000).unwrap();
    motor.set_max_accel(5000000).unwrap();
    motor.set_step_mode(TicStepMode::Microstep8).unwrap();

    //find zero
    loop {
        Timer::after(Duration::from_millis(100)).await;
        let mut pitch_sum: f64 = 0.0;
        for _i in 0..20 {
            let pitch: f64 = calculate_pitch(accel) as f64;
            pitch_sum += pitch;
            Timer::after(Duration::from_millis(20)).await;
        }
        pitch_sum /= 20.0;

        // Prevents movement from erroring out
        motor
            .reset_command_timeout()
            .expect("Motor horizontal communication failure");

        // Slow down after reaching within 0.5 degrees
        if f64::abs(pitch_sum - ZERO_CAL) < 0.5 {
            target_velocity = -SPEED_VERYSLOW;
            Timer::after(Duration::from_millis(50)).await;
        } else {
            target_velocity = -7000000;
        }

        if pitch_sum <= -0.02 {
            motor.set_target_velocity(target_velocity).unwrap();
        } else if pitch_sum >= 0.02 {
            motor.set_target_velocity(-target_velocity).unwrap();
        } else {
            motor.halt_and_set_position(0).unwrap();
            break;
        }
        Timer::after(Duration::from_millis(200)).await;
        motor.set_target_velocity(0).unwrap();
    }
    motor.set_max_decel(TIC_DECEL_DEFAULT_VERTICAL).unwrap();
    motor.set_max_accel(TIC_DECEL_DEFAULT_VERTICAL).unwrap();
    motor.set_step_mode(DEFAULT_STEP_MODE_VERTICAL).unwrap();
}

/// Calculate most optimal difference in current and destination angle
fn get_delta_angle(curr_angle: f32, new_angle: f32) -> f32 {
    let diff: f32 = ((new_angle - curr_angle + 180.0) % 360.0) - 180.0;
    if diff < -180.0 {
        diff + 360.0 //if angle less than -180 switch directions
    } else {
        diff
    }
}

fn get_relative_angle<I: embedded_hal::i2c::I2c>(motor: &mut TicI2C<I, NoDelay>) -> f32 {
    let mut curr_angle: f32 =
        motor.current_position().unwrap() as f32 / STEPS_PER_DEGREE_HORIZONTAL as f32;

    while curr_angle > 180.0 {
        curr_angle -= 360.0;
    }
    while curr_angle < 180.0 {
        curr_angle += 360.0;
    }
    curr_angle
}
