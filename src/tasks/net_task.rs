extern crate alloc;

use crate::robot::commands::TcpCommand;
use alloc::string::String;
use core::str::FromStr;
use embassy_net::{tcp::TcpSocket, IpListenEndpoint, Stack};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Sender};
use embassy_time::{Duration, Timer};
use esp_wifi::wifi::{ClientConfiguration, WifiController, WifiDevice};
use log::{error, info};

/// Keeps the net stack up. (processes network events)
#[embassy_executor::task]
pub async fn runner_task(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}

#[embassy_executor::task]
pub async fn tcp_server(
    stack: Stack<'static>,
    cmd_sender: Sender<'static, CriticalSectionRawMutex, TcpCommand, 3>,
) {
    let mut rx_buf = [0u8; 512];
    let mut tx_buf = [0u8; 512];

    loop {
        // Create NEW socket for each connection
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        socket.set_timeout(Some(Duration::from_secs(2)));

        info!("Waiting for connection...");
        match socket
            .accept(IpListenEndpoint {
                port: 1234,
                addr: None,
            })
            .await
        {
            Ok(_) => {
                info!("Client connected!");
                handle_connection(&mut socket, &cmd_sender).await;
            }
            Err(e) => {
                error!("Accept failed: {:?}", e);
                Timer::after_millis(500).await; // Backoff delay
                continue;
            }
        }
    }
}

pub async fn handle_connection(
    socket: &mut TcpSocket<'_>,
    cmd_sender: &Sender<'static, CriticalSectionRawMutex, TcpCommand, 3>,
) {
    let mut buf = [0u8; 256];
    loop {
        match socket.read(&mut buf).await {
            Ok(0) => break,
            Ok(n) => {
                if let Ok(cmd) = TcpCommand::try_from(core::str::from_utf8(&buf[..n]).unwrap()) {
                    cmd_sender.send(cmd).await;
                };
                break; // Close after first command
            }
            Err(e) => {
                error!("Read error: {:?}", e);
                break;
            }
        }
    }
}

pub async fn configurate_and_start_wifi(wifi_controller: &mut WifiController<'_>) {
    let ssid = env!("WIFI_SSID");
    let password = env!("WIFI_PASS");
    let config = esp_wifi::wifi::Configuration::Client(ClientConfiguration {
        ssid: String::from_str(ssid).unwrap(),
        password: String::from_str(password).unwrap(),
        ..Default::default()
    });

    info!("Connecting to wifi: {ssid}");
    wifi_controller
        .set_configuration(&config)
        .expect("fail setting configuration of wifi controller");

    wifi_controller
        .set_power_saving(esp_wifi::config::PowerSaveMode::None)
        .expect("Fail setting wifi power mode");

    wifi_controller.start().unwrap();
    wifi_controller
        .connect_async()
        .await
        .inspect_err(|e| error!("An error occured trying to connect to wifi: {e:?}"))
        .unwrap();

    match wifi_controller.rssi().ok() {
        Some(rssi) => info!("Wifi connected! signal: {}", rssi),
        None => info!("Wifi connected!"),
    }
}
