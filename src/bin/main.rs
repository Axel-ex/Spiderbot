#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::future::pending;

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{AnyPin, Pin};
use esp_hal::timer::timg::TimerGroup;
use esp_wifi::ble::controller::BleConnector;
use log::info;
use spider_robot::servo_task::servo_task;

extern crate alloc;
esp_bootloader_esp_idf::esp_app_desc!();

//LEGS: [mid, tip, tail]
//FRONT_L: [32, 33, 25]
//BOTTOM_L: [26, 27, 14]
//FRONT_R: [12, 13, 23]
//BOTTOM_R: [22, 1, 3]

// #[panic_handler]
// fn panic(info: &core::panic::PanicInfo) -> ! {
//     println!("Panic: {}", info);
//     loop {}
// }

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 32 * 1024);
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 96 * 1024);

    let timer0 = TimerGroup::new(p.TIMG1);
    esp_hal_embassy::init(timer0.timer0);
    info!("Embassy initialized");

    let rng = esp_hal::rng::Rng::new(p.RNG);
    let timer1 = TimerGroup::new(p.TIMG0);
    let wifi_init = esp_wifi::init(timer1.timer0, rng, p.RADIO_CLK)
        .expect("Failed to initialize WIFI/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_wifi::wifi::new(&wifi_init, p.WIFI).expect("Failed to initialize WIFI controller");
    let transport = BleConnector::new(&wifi_init, p.BT);
    let _ble_controller = ExternalController::<_, 20>::new(transport);

    let servo_pins: [AnyPin<'static>; 12] = [
        p.GPIO32.degrade(),
        p.GPIO33.degrade(),
        p.GPIO25.degrade(),
        p.GPIO26.degrade(),
        p.GPIO27.degrade(),
        p.GPIO14.degrade(),
        p.GPIO12.degrade(),
        p.GPIO13.degrade(),
        p.GPIO23.degrade(),
        p.GPIO22.degrade(),
        p.GPIO1.degrade(),
        p.GPIO3.degrade(),
    ];

    spawner.spawn(servo_task(servo_pins, p.LEDC)).unwrap();

    loop {
        pending::<()>().await;
    }
}
