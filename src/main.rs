#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;

use alloc::boxed::Box;
use core::future::pending;
use embassy_executor::Spawner;
use embassy_net::{Config as NetConfig, StackResources};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{AnyPin, Pin};
use esp_hal::timer::timg::TimerGroup;
use log::info;
use spider_robot::tasks::net_task::{configurate_and_start_wifi, runner_task, tcp_server};
use spider_robot::tasks::servo_task::servo_task;

esp_bootloader_esp_idf::esp_app_desc!();

//LEGS: [mid, tip, tail]
//FRONT_L: [32, 33, 25]
//BOTTOM_L: [26, 27, 14]
//FRONT_R: [12, 13, 19]
//BOTTOM_R: [18, 5, 4]

// #[panic_handler]
// fn panic(info: &core::panic::PanicInfo) -> ! {
//     println!("Panic: {}", info);
//     loop {}
// }
//
macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.init_with(|| $val)
    }};
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 32 * 1024);
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 96 * 1024);

    let timer0 = TimerGroup::new(p.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    // take important peripherals
    let mut rng = esp_hal::rng::Rng::new(p.RNG);
    let timer1 = TimerGroup::new(p.TIMG0);
    let wifi_init = esp_wifi::init(timer1.timer0, rng, p.RADIO_CLK)
        .expect("Failed to initialize WIFI/BLE controller");
    let wifi_init = Box::leak(Box::new(wifi_init));
    let (mut wifi_controller, interfaces) =
        esp_wifi::wifi::new(wifi_init, p.WIFI).expect("Failed to initialize WIFI controller");

    configurate_and_start_wifi(&mut wifi_controller).await;

    let servo_pins: [AnyPin<'static>; 12] = [
        p.GPIO32.degrade(),
        p.GPIO33.degrade(),
        p.GPIO25.degrade(),
        p.GPIO26.degrade(),
        p.GPIO27.degrade(),
        p.GPIO14.degrade(),
        p.GPIO12.degrade(),
        p.GPIO13.degrade(),
        p.GPIO19.degrade(),
        p.GPIO18.degrade(),
        p.GPIO5.degrade(),
        p.GPIO17.degrade(),
    ];

    //Get trhe embassy net stack up and working.
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let config = NetConfig::dhcpv4(Default::default());
    let device = interfaces.sta;
    let (stack, runner) = embassy_net::new(
        device,
        config,
        mk_static!(StackResources<3>, StackResources::new()),
        seed,
    );

    info!("Starting spider robot...");
    spawner
        .spawn(servo_task(servo_pins, p.LEDC))
        .expect("Fail spawning servo task");
    // spawner
    //     .spawn(runner_task(runner))
    //     .expect("Fail spawning runner task");
    // spawner
    //     .spawn(tcp_server(stack))
    //     .expect("Fail spawning net task");

    loop {
        pending::<()>().await;
    }
}
