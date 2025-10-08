use crate::robot::{
    joint::Joint,
    leg::*,
    servo::{AnyServo, Servo},
};
extern crate alloc;

use crate::kinematics::{
    conversion::{cartesian_to_polar, polar_to_servo},
    gait_engine::MOVEMENT_COMPLETED,
};
use crate::robot::commands::ServoCommand;
use alloc::boxed::Box;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Receiver};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::gpio::AnyPin;
use esp_hal::ledc::channel::{self, ChannelIFace, Number};
use esp_hal::ledc::timer::{HSClockSource, LSClockSource, TimerIFace};
use esp_hal::ledc::{timer, HighSpeed, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::peripherals::LEDC;
use esp_hal::time::Rate;
use fugit::HertzU32;
use log::{debug, info};

#[embassy_executor::task]
pub async fn servo_task(
    servo_pins: [AnyPin<'static>; 12],
    ledc: LEDC<'static>,
    receiver: Receiver<'static, CriticalSectionRawMutex, ServoCommand, 3>,
) {
    info!("Starting servo task");
    let mut ledc = Ledc::new(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    //Configure timers: Leak them to get static lifetime.
    let (timer_low, timer_high) = create_configure_timers(&mut ledc).await;
    let timer_low = Box::leak(Box::new(timer_low));
    let timer_high = Box::leak(Box::new(timer_high));

    let mut servos = creates_servos(servo_pins, &mut ledc, timer_low, timer_high)
        .await
        .expect("Fail creating the servos");

    loop {
        let cmd = receiver.receive().await;
        debug!("[SERVO_TASK] Received a command!");
        update_position(cmd, &mut servos).await;
    }
}

pub async fn update_position(mut cmd: ServoCommand, servos: &mut [[AnyServo; 3]; 4]) {
    let mut ticker = Ticker::every(Duration::from_millis(20));

    loop {
        for leg in 0..4 {
            for joint in 0..3 {
                let diff = (cmd.current_pos[leg][joint] - cmd.expected_pos[leg][joint]).abs();
                let speed = cmd.temp_speed[leg][joint].abs();

                if diff >= speed {
                    cmd.current_pos[leg][joint] += cmd.temp_speed[leg][joint];
                } else {
                    cmd.current_pos[leg][joint] = cmd.expected_pos[leg][joint];
                }
            }
            let (alpha, beta, gamma) = cartesian_to_polar(
                cmd.current_pos[leg][0],
                cmd.current_pos[leg][1],
                cmd.current_pos[leg][2],
            );
            polar_to_servo(servos, leg.into(), alpha, beta, gamma);
        }
        if movement_is_done(&cmd) {
            break;
        }
        ticker.next().await;
    }
    MOVEMENT_COMPLETED.signal(());
}

pub fn movement_is_done(cmd: &ServoCommand) -> bool {
    for i in 0..4 {
        if !leg_movement_is_done(cmd, i) {
            return false;
        }
    }
    true
}

/// Check if the servos of a leg have reach their position
fn leg_movement_is_done(cmd: &ServoCommand, leg_id: usize) -> bool {
    cmd.current_pos[leg_id][0] == cmd.expected_pos[leg_id][0]
        && cmd.current_pos[leg_id][1] == cmd.expected_pos[leg_id][1]
        && cmd.current_pos[leg_id][2] == cmd.expected_pos[leg_id][2]
}

// 10bit register (max duty = 1024)
pub async fn create_configure_timers(
    ledc: &mut Ledc<'static>,
) -> (
    timer::Timer<'static, LowSpeed>,
    timer::Timer<'static, HighSpeed>,
) {
    let mut timer_low = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    let mut timer_high = ledc.timer::<HighSpeed>(timer::Number::Timer1);
    Timer::after_millis(500).await;

    //Configure timers
    timer_low
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty11Bit,
            clock_source: LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .expect("Fail creating ledc timer");
    Timer::after_millis(500).await;

    timer_high
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty11Bit,
            clock_source: HSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .expect("Fail creating high speed timer");

    (timer_low, timer_high)
}

pub async fn creates_servos(
    pins: [AnyPin<'static>; 12],
    ledc: &mut Ledc<'static>,
    timer_low: &'static timer::Timer<'static, LowSpeed>,
    timer_high: &'static timer::Timer<'static, HighSpeed>,
) -> Result<[[AnyServo; 3]; 4], anyhow::Error> {
    // Create array of options first
    let mut servo_array: [[Option<AnyServo>; 3]; 4] = [
        [None, None, None],
        [None, None, None],
        [None, None, None],
        [None, None, None],
    ];

    // Define the channel numbers for each pin
    let channel_numbers = [
        // Low speed channels (0-7)
        Number::Channel0, // 0
        Number::Channel1, // 1
        Number::Channel2, // 2
        Number::Channel3, // 3
        Number::Channel4, // 4
        Number::Channel5, // 5
        Number::Channel6, // 6
        Number::Channel7, // 7
        // High speed channels (0-3)
        Number::Channel0, // 8
        Number::Channel1, // 9
        Number::Channel2, // 10
        Number::Channel3, // 11
    ];

    for (i, pin) in pins.into_iter().enumerate() {
        let (leg, joint) = match i {
            0 => (0, 0),
            1 => (0, 1),
            2 => (0, 2), // Leg 0 (Front Left)
            3 => (1, 0),
            4 => (1, 1),
            5 => (1, 2), // Leg 1 (bottom Left)
            6 => (2, 0),
            7 => (2, 1),
            8 => (2, 2), // Leg 2 (Front Right)
            9 => (3, 0),
            10 => (3, 1),
            11 => (3, 2), // Leg 3 (Bottom right)
            _ => unreachable!(),
        };

        let channel_num = channel_numbers[i];
        let is_high_speed = i >= 8;

        let servo = if is_high_speed {
            // Configure high speed channel
            let mut channel = ledc.channel(channel_num, pin);
            channel
                .configure(channel::config::Config {
                    timer: timer_high,
                    duty_pct: 0,
                    pin_config: channel::config::PinConfig::PushPull,
                })
                .map_err(|_| anyhow::anyhow!("Failed to configure high speed channel {}", i))?;

            let max_duty = channel.max_duty_cycle() as u32;
            AnyServo::High(Servo::new(
                channel,
                max_duty,
                HertzU32::from_raw(50),
                Leg::from(leg),
                Joint::from(joint),
            ))
        } else {
            // Configure low speed channel
            let mut channel = ledc.channel(channel_num, pin);
            channel
                .configure(channel::config::Config {
                    timer: timer_low,
                    duty_pct: 0,
                    pin_config: channel::config::PinConfig::PushPull,
                })
                .map_err(|_| anyhow::anyhow!("Failed to configure low speed channel {}", i))?;

            let max_duty = channel.max_duty_cycle() as u32;
            AnyServo::Low(Servo::new(
                channel,
                max_duty,
                HertzU32::from_raw(50),
                Leg::from(leg),
                Joint::from(joint),
            ))
        };

        servo_array[leg][joint] = Some(servo);
    }

    // Convert Option<AnyServo> to AnyServo
    Ok(servo_array.map(|leg| leg.map(|s| s.expect("All servo positions should be filled"))))
}

pub async fn calibrate(servos: &mut [[AnyServo; 3]; 4]) {
    for leg in 0..4 {
        for joint in 0..3 {
            servos[leg][joint].set_angle(90);
        }
    }
}
