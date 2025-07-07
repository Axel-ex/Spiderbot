use crate::servo::{AnyServo, Servo};
extern crate alloc;

use alloc::boxed::Box;
use embassy_time::Timer;
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::gpio::AnyPin;
use esp_hal::ledc::channel::{self, Channel, ChannelIFace, Number};
use esp_hal::ledc::timer::{HSClockSource, LSClockSource, TimerIFace};
use esp_hal::ledc::{timer, HighSpeed, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::peripherals::LEDC;
use esp_hal::time::Rate;
use fugit::HertzU32;
use heapless::Vec;
use log::{error, info};

#[embassy_executor::task]
pub async fn servo_task(servo_pins: [AnyPin<'static>; 12], ledc: LEDC<'static>) {
    info!("Starting servo task");
    let mut ledc = Ledc::new(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    //Configure timers: Leak them to get static lifetime.
    let (timer_low, timer_high) = create_configure_timers(&mut ledc).await;
    let timer_low = Box::leak(Box::new(timer_low));
    let timer_high = Box::leak(Box::new(timer_high));

    let mut servos = creates_servos(servo_pins, &mut ledc, timer_low, timer_high)
        .await
        .unwrap();

    loop {
        for servo in servos.iter_mut() {
            servo
                .set_angle(0)
                .unwrap_or_else(|_e| error!("Angle set failed"));

            Timer::after_millis(500).await;
        }
    }
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

/// Create the channel and configure them in loop
/// create the servos by wrapping tem into ANyServo to get the same behaviour no matter the
/// underlying channel speed
pub async fn creates_servos(
    pins: [AnyPin<'static>; 12],
    ledc: &mut Ledc<'static>,
    timer_low: &'static timer::Timer<'static, LowSpeed>,
    timer_high: &'static timer::Timer<'static, HighSpeed>,
) -> Result<Vec<AnyServo, 12>, anyhow::Error> {
    let mut servos: Vec<AnyServo, 12> = Vec::new();

    let [p32, p33, p25, p26, p27, p14, p12, p13, p19, p18, p5, p17] = pins;
    let low_speed_channels: [Channel<'_, LowSpeed>; 8] = [
        ledc.channel(Number::Channel0, p32),
        ledc.channel(Number::Channel1, p33),
        ledc.channel(Number::Channel2, p25),
        ledc.channel(Number::Channel3, p26),
        ledc.channel(Number::Channel4, p27),
        ledc.channel(Number::Channel5, p14),
        ledc.channel(Number::Channel6, p12),
        ledc.channel(Number::Channel7, p13),
    ];
    for mut channel in low_speed_channels {
        channel
            .configure(channel::config::Config {
                timer: timer_low,
                duty_pct: 7,
                pin_config: channel::config::PinConfig::PushPull,
            })
            .map_err(|_| anyhow::anyhow!("Configurating low"))?;
        let max_duty = channel.max_duty_cycle() as u32;
        let servo = Servo::new(channel, max_duty, HertzU32::from_raw(50));
        let _ = servos.push(AnyServo::Low(servo));
    }

    let high_speed_channels: [Channel<'_, HighSpeed>; 4] = [
        ledc.channel(Number::Channel0, p19),
        ledc.channel(Number::Channel1, p18),
        ledc.channel(Number::Channel2, p5),
        ledc.channel(Number::Channel3, p17),
    ];
    for mut channel in high_speed_channels {
        channel
            .configure(channel::config::Config {
                timer: timer_high,
                duty_pct: 7,
                pin_config: channel::config::PinConfig::PushPull,
            })
            .map_err(|_| anyhow::anyhow!("Configurating high"))?;

        let max_duty = channel.max_duty_cycle() as u32;
        let servo = Servo::new(channel, max_duty, HertzU32::from_raw(50));
        let _ = servos.push(AnyServo::High(servo));
    }

    Ok(servos)
}
