//! Main entry point for Spiderbot firmware.
//!
//! Initializes hardware, networking, and spawns async tasks for motion, networking, and servo control.
//! Uses Embassy for async execution and ESP HAL for hardware access.
//!
//! The main loop keeps the firmware alive after spawning all tasks.
#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;

use core::future::pending;
use embassy_executor::Spawner;
use embassy_net::{Config as NetConfig, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::timer::timg::TimerGroup;
use pwm_pca9685::Pca9685;
use spider_robot::robot::commands::{ServoCommand, TcpCommand};
use spider_robot::tasks::motion_task::motion_task;
use spider_robot::tasks::net_task::{configurate_and_start_wifi, net_task, runner_task};
use spider_robot::tasks::servo_task::servo_task;
use spider_robot::{SERVOCMD_CHANNEL_SIZE, TCPCMD_CHANNEL_SIZE};

esp_bootloader_esp_idf::esp_app_desc!();

static TCP_CMD_CHANNEL: Channel<CriticalSectionRawMutex, TcpCommand, TCPCMD_CHANNEL_SIZE> =
    Channel::new();
static SERVO_CMD_CHANNEL: Channel<CriticalSectionRawMutex, ServoCommand, SERVOCMD_CHANNEL_SIZE> =
    Channel::new();

macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.init_with(|| $val)
    }};
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    //Boilerplate to init clocks, setup the heap and take important peripherals
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 32 * 1024);
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 96 * 1024);

    // Start the embassy runtime
    let timer0 = TimerGroup::new(p.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    // Init wifi
    let mut rng = esp_hal::rng::Rng::new(p.RNG);
    let timer1 = TimerGroup::new(p.TIMG0);
    let wifi_init = esp_wifi::init(timer1.timer0, rng, p.RADIO_CLK)
        .expect("Failed to initialize WIFI/BLE controller");
    let wifi_init = mk_static!(esp_wifi::EspWifiController, wifi_init);

    let (mut wifi_controller, interfaces) = esp_wifi::wifi::new(wifi_init, p.WIFI).unwrap();
    configurate_and_start_wifi(&mut wifi_controller).await;

    //Get the embassy net stack up and working.
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let config = NetConfig::dhcpv4(Default::default());
    let device = interfaces.sta;
    let (stack, runner) = embassy_net::new(
        device,
        config,
        mk_static!(StackResources<3>, StackResources::new()),
        seed,
    );

    // I2c
    let i2c_dev = I2c::new(p.I2C0, Config::default())
        .unwrap()
        .with_sda(p.GPIO21)
        .with_scl(p.GPIO22)
        .into_async();

    //Pca9685
    let pwm = Pca9685::new(i2c_dev, pwm_pca9685::Address::from(0x7f)).unwrap();

    spawner
        .spawn(runner_task(runner))
        .expect("Fail spawning runner task");
    spawner
        .spawn(net_task(stack, TCP_CMD_CHANNEL.sender()))
        .expect("Fail spawning net task");
    spawner
        .spawn(motion_task(
            TCP_CMD_CHANNEL.receiver(),
            SERVO_CMD_CHANNEL.sender(),
        ))
        .expect("Fail spawning the motion task"); // listen for commands forwarded from the tcp
                                                  // server
    spawner
        .spawn(servo_task(pwm, SERVO_CMD_CHANNEL.receiver()))
        .expect("Fail spawning servo task");

    loop {
        pending::<()>().await; //the main loop doesnt perform any job
    }
}
