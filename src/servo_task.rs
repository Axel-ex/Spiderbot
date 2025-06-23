use crate::servo::{AnyServo, Servo, ServoTrait};

use embassy_time::Timer;
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::gpio::AnyPin;
use esp_hal::ledc::channel::{self, Channel, ChannelIFace, Number};
use esp_hal::ledc::timer::{LSClockSource, TimerIFace};
use esp_hal::ledc::{timer, HighSpeed, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::peripherals::LEDC;
use esp_hal::time::Rate;
use fugit::HertzU32;
use heapless::Vec;
use log::{error, info};
extern crate alloc;
use alloc::boxed::Box;

pub async fn create_configure_timers(
    ledc: &mut Ledc<'static>,
) -> (
    timer::Timer<'static, LowSpeed>,
    timer::Timer<'static, HighSpeed>,
) {
    let mut timer_low = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    let mut timer_high = ledc.timer::<HighSpeed>(timer::Number::Timer0);
    Timer::after_millis(500).await;

    //Configure timers
    timer_low
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .expect("Fail creating ledc timer");
    Timer::after_millis(500).await;

    timer_high
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::HSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .expect("Fail creating high speed timer");

    (timer_low, timer_high)
}

#[embassy_executor::task]
pub async fn servo_task(servo_pins: [AnyPin<'static>; 12], ledc: LEDC<'static>) {
    info!("Starting servo task");
    let mut ledc = Ledc::new(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    //Configure timers: Leak them to get static lifetime.
    let (timer_low, timer_high) = create_configure_timers(&mut ledc).await;
    let timer_low = Box::leak(Box::new(timer_low));
    let timer_high = Box::leak(Box::new(timer_high));
    let [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11] = servo_pins;

    //Create the vec of servos
    let mut servos: Vec<AnyServo, 12> = Vec::new();

    let low_speed_channels: [Channel<'_, LowSpeed>; 8] = [
        ledc.channel(Number::Channel0, p0),
        ledc.channel(Number::Channel1, p1),
        ledc.channel(Number::Channel2, p2),
        ledc.channel(Number::Channel3, p3),
        ledc.channel(Number::Channel4, p4),
        ledc.channel(Number::Channel5, p5),
        ledc.channel(Number::Channel6, p6),
        ledc.channel(Number::Channel7, p7),
    ];
    for mut channel in low_speed_channels {
        channel
            .configure(channel::config::Config {
                timer: timer_low,
                duty_pct: 7,
                pin_config: channel::config::PinConfig::PushPull,
            })
            .expect("Fail configurating low speed channels");
        let max_duty = channel.max_duty_cycle() as u32;
        let servo = Servo::new(channel, max_duty, HertzU32::from_raw(50));
        let _ = servos.push(AnyServo::Low(servo));
    }

    let high_speed_channels: [Channel<'_, HighSpeed>; 4] = [
        ledc.channel(Number::Channel0, p8),
        ledc.channel(Number::Channel1, p9),
        ledc.channel(Number::Channel2, p10),
        ledc.channel(Number::Channel3, p11),
    ];
    for mut channel in high_speed_channels {
        channel
            .configure(channel::config::Config {
                timer: timer_high,
                duty_pct: 7,
                pin_config: channel::config::PinConfig::PushPull,
            })
            .expect("Fail configurating high speed channels");
        let max_duty = channel.max_duty_cycle() as u32;
        let servo = Servo::new(channel, max_duty, HertzU32::from_raw(50));
        let _ = servos.push(AnyServo::High(servo));
    }

    loop {
        for mut servo in servos {
            servo
                .set_angle(0)
                .unwrap_or_else(|_e| error!("Angle set failed"));

            Timer::after_millis(500).await;
        }
        todo!();
    }
}
