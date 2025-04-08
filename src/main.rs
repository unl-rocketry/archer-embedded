#![no_main]
#![no_std]

mod commands;
use commands::parse_command;

use alloc::string::String;
use core::cell::RefCell;
use embedded_hal_bus::i2c::RefCellDevice;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    i2c::{self, master::I2c},
};
use esp_println::{print, println};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use log::{error, info};
use mma8x5x::{GScale, Mma8x5x, OutputDataRate, PowerMode, ic::Mma8451, mode};
use pololu_tic::{TicHandlerError, TicI2C, TicProduct, TicStepMode, base::TicBase};

extern crate alloc;

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

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(72 * 1024);
    esp_println::logger::init_logger_from_env();
    let timer0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);
    info!("Embassy initialized!");

    let sda = peripherals.GPIO18;
    let scl = peripherals.GPIO19;

    let _ = spawner;

    let i2c_bus = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_timeout(i2c::master::BusTimeout::Maximum),
    )
    .unwrap()
    .with_sda(sda)
    .with_scl(scl)
    .into_async();
    let i2c_bus = RefCell::new(i2c_bus);

    let mut motor_horizontal =
        pololu_tic::TicI2C::new_with_address(RefCellDevice::new(&i2c_bus), TicProduct::Tic36v4, 14);
    let mut motor_vertical =
        pololu_tic::TicI2C::new_with_address(RefCellDevice::new(&i2c_bus), TicProduct::Tic36v4, 15);

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

    let (tx_pin, rx_pin) = (peripherals.GPIO1, peripherals.GPIO3);
    let config = esp_hal::uart::Config::default().with_rx_fifo_full_threshold(64);

    let mut uart0 = esp_hal::uart::Uart::new(peripherals.UART0, config)
        .unwrap()
        .with_tx(tx_pin)
        .with_rx(rx_pin)
        .into_async();

    uart0.set_at_cmd(esp_hal::uart::AtCmdConfig::default().with_cmd_char(0x04));

    let mut accelerometer = if let Ok(a) = accelerometer.into_active() {
        info!("MMA8451 set up!!");
        Some(a)
    } else {
        None
    };

    setup_motor(&mut motor_horizontal, MotorAxis::Horizontal)
        .expect("Horizontal motor setup error");
    setup_motor(&mut motor_vertical, MotorAxis::Vertical).expect("Vertical motor setup error");
    info!("Motors set up!!");

    let mut is_calibrated = false;

    let mut buffer = [0; 1];
    let mut command_string = String::new();

    loop {
        Timer::after(Duration::from_millis(1)).await;

        if motor_horizontal.reset_command_timeout().is_err() {
            loop {
                error!("Horizontal motor communication failure");
                Timer::after(Duration::from_secs(1)).await;
            }
        }
        if motor_vertical.reset_command_timeout().is_err() {
            loop {
                error!("Vertical motor communication failure");
                Timer::after(Duration::from_secs(1)).await;
            }
        }

        let count = uart0.read_buffered_bytes(&mut buffer).unwrap();

        // If there were no bytes read, don't try to use them
        if count == 0 {
            continue;
        }

        if buffer[0] == b'\x1B' {
            command_string.clear();
            match parse_command(
                &mut motor_vertical,
                &mut motor_horizontal,
                &mut accelerometer,
                "HALT ",
                &mut is_calibrated,
            )
            .await
            {
                Ok(_) => print!("OK SOFTWARE E-STOP (ESC RECIEVED)\n"),
                Err(e) => print!("ERR: {:?}, {}\n", e, e),
            }
            continue;
        }

        if buffer[0] == b'\r' || buffer[0] == b'\n' {
            println!();
            command_string += " ";

            match parse_command(
                &mut motor_vertical,
                &mut motor_horizontal,
                &mut accelerometer,
                &command_string,
                &mut is_calibrated,
            )
            .await
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

#[derive(PartialEq, Eq)]
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
    motor: &mut TicI2C<I>,
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
    motor: &mut TicI2C<I>,
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

fn get_relative_angle<I: embedded_hal::i2c::I2c>(motor: &mut TicI2C<I>) -> f32 {
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
