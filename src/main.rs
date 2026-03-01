#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

mod commands;

use core::cell::RefCell;

use alloc::string::String;
use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::i2c::RefCellDevice;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_println::{println, print};
use esp_radio::ble::controller::BleConnector;
use log::info;
use mma8x5x::{GScale, Mma8x5x, OutputDataRate, PowerMode, ic::Mma8451, mode};
use trouble_host::prelude::*;

use pololu_tic::{HandlerError, I2c as TicI2c, Product, TicBase as _, variables::StepMode};

use esp_hal::i2c::{self, master::I2c};

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

const STEPS_PER_DEGREE_VERTICAL: u32 = (23.3 * tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as f32) as u32;
const STEPS_PER_DEGREE_HORIZONTAL: u32 = (126.0 * tic_step_mult(DEFAULT_STEP_MODE_HORIZONTAL) as f32) as u32;
const SPEED_VERYSLOW: i32 = 20000 * tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as i32; //only used on CALV so no need to add a second one
const SPEED_DEFAULT_VERTICAL: i32 = 15000000 * tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as i32;
const SPEED_DEFAULT_HORIZONTAL: i32 = 10000000 * tic_step_mult(DEFAULT_STEP_MODE_HORIZONTAL) as i32;
const SPEED_MAX_VERTICAL: u32 = 15000000 * tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as u32;
const SPEED_MAX_HORIZONTAL: u32 = 10000000 * tic_step_mult(DEFAULT_STEP_MODE_HORIZONTAL) as u32;

const TIC_DECEL_DEFAULT_VERTICAL: u32 =
    400000 * (tic_step_mult(DEFAULT_STEP_MODE_VERTICAL) as u32 / 2);
const TIC_DECEL_DEFAULT_HORIZONTAL: u32 =
    300000 * tic_step_mult(DEFAULT_STEP_MODE_HORIZONTAL) as u32;

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

const DEFAULT_CURRENT: u16 = 1024;

const DEFAULT_STEP_MODE_VERTICAL: StepMode = StepMode::Microstep16;
const DEFAULT_STEP_MODE_HORIZONTAL: StepMode = StepMode::Microstep8;

// Offsets calculated manually from accelerometer data
const ACC_OFFSET_X: i16 = 53 / 8;
const ACC_OFFSET_Y: i16 = 83 / 8;
const ACC_OFFSET_Z: i16 = -154 / 8;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 1>::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let _stack = trouble_host::new(ble_controller, &mut resources);

    let sda = peripherals.GPIO18;
    let scl = peripherals.GPIO19;

    // TODO: Spawn some tasks
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
        pololu_tic::I2c::new_with_address(RefCellDevice::new(&i2c_bus), pololu_tic::Product::Tic36v4, Delay, 14);
    let mut motor_vertical =
        pololu_tic::I2c::new_with_address(RefCellDevice::new(&i2c_bus), pololu_tic::Product::Tic36v4, Delay, 15);

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

    setup_motor(&mut motor_horizontal, MotorAxis::Horizontal).expect("Horizontal motor setup error");
    setup_motor(&mut motor_vertical, MotorAxis::Vertical).expect("Vertical motor setup error");
    info!("Motors set up!!");

    let (tx_pin, rx_pin) = (peripherals.GPIO1, peripherals.GPIO3);
    let config = esp_hal::uart::Config::default();

    let mut uart0 = esp_hal::uart::Uart::new(peripherals.UART0, config)
        .unwrap()
        .with_tx(tx_pin)
        .with_rx(rx_pin)
        .into_async();

    let mut accelerometer = if let Ok(a) = accelerometer.into_active() {
        info!("MMA8451 set up!!");
        Some(a)
    } else {
        None
    };

    let mut is_calibrated = false;

    let mut buffer = [0; 1];
    let mut command_string = String::new();

    let mut timer = Instant::now();

    loop {
        if timer.elapsed() > Duration::from_millis(100) {
            while motor_horizontal.reset_command_timeout().is_err() {
                log::error!("Horizontal motor communication failure, attempting reconnection");
                motor_horizontal =
                TicI2c::new_with_address(RefCellDevice::new(&i2c_bus), Product::Tic36v4, Delay, 14);

                if setup_motor(&mut motor_horizontal, MotorAxis::Horizontal).is_ok() {
                    info!("Horizontal motor communication restored!");
                }
                Timer::after(Duration::from_secs(1)).await;
            }

            while motor_vertical.reset_command_timeout().is_err() {
                log::error!("Vertical motor communication failure, attempting reconnection");
                motor_vertical =
                TicI2c::new_with_address(RefCellDevice::new(&i2c_bus), Product::Tic36v4, Delay, 15);

                if setup_motor(&mut motor_vertical, MotorAxis::Vertical).is_ok() {
                    info!("Vertical motor communication restored!");
                }
                Timer::after(Duration::from_secs(1)).await;
            }

            timer = Instant::now();
        }

        let count = uart0.read_buffered(&mut buffer).unwrap();

        // If there were no bytes read, don't try to use them
        if count == 0 {
            continue;
        }

        if buffer[0] == b'\x1B' {
            command_string.clear();
            match commands::parse_command(
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

            match commands::parse_command(
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
fn setup_motor<I: embedded_hal::i2c::I2c, D: DelayNs>(
    motor: &mut TicI2c<I, D>,
    motor_axis: MotorAxis,
) -> Result<(), HandlerError> {
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


async fn calibrate_vertical<I: embedded_hal::i2c::I2c, D: DelayNs>(
    motor: &mut TicI2c<I, D>,
    accel: &mut Mma8x5x<I, Mma8451, mode::Active>,
) {
    const ZERO_CAL: f64 = 0.2;
    let mut target_velocity: i32;
    motor.set_max_decel(5000000).unwrap();
    motor.set_max_accel(5000000).unwrap();
    motor.set_step_mode(StepMode::Microstep8).unwrap();

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

fn get_relative_angle<I: embedded_hal::i2c::I2c, D: DelayNs>(motor: &mut TicI2c<I, D>) -> f32 {
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
