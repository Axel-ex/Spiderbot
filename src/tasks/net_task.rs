//! Networking and TCP command server task.
//!
//! Manages WiFi connection, listens for TCP commands, parses them, and forwards
//! them to the motion task for execution.
//!
//! Handles network errors and reconnection logic.
extern crate alloc;

use crate::config::TCPCMD_CHANNEL_SIZE;
use crate::config::{PORT, RX_BUF_SIZE, TX_BUF_SIZE};
use crate::robot::commands::TcpCommand;
use alloc::string::String;
use core::str::FromStr;
use embassy_net::{tcp::TcpSocket, IpListenEndpoint, Stack};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Sender};
use embassy_time::Timer;
use esp_wifi::wifi::{ClientConfiguration, WifiController, WifiDevice};
use log::{error, info, warn};

#[embassy_executor::task]
pub async fn runner_task(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}

#[embassy_executor::task]
pub async fn net_task(
    stack: Stack<'static>,
    cmd_sender: Sender<'static, CriticalSectionRawMutex, TcpCommand, TCPCMD_CHANNEL_SIZE>,
) {
    let mut rx_buf = [0u8; RX_BUF_SIZE];
    let mut tx_buf = [0u8; TX_BUF_SIZE];

    while !stack.is_link_up() {
        Timer::after_millis(500).await;
    }

    if let Some(config) = stack.config_v4() {
        info!(
            "TCP server listening at address {}:{}",
            config.address, PORT
        );
    }

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);

        match socket
            .accept(IpListenEndpoint {
                port: PORT,
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
    cmd_sender: &Sender<'static, CriticalSectionRawMutex, TcpCommand, TCPCMD_CHANNEL_SIZE>,
) {
    let mut rx_buf = [0u8; RX_BUF_SIZE];
    loop {
        match socket.read(&mut rx_buf).await {
            Ok(0) => break,
            Ok(n) => {
                let received_str = core::str::from_utf8(&rx_buf[..n])
                    .unwrap_or_default()
                    .trim();
                if let Ok(cmd) = TcpCommand::try_from(received_str) {
                    match cmd {
                        TcpCommand::CloseConnection => break, // special case
                        _ => cmd_sender.send(cmd).await,
                    }
                } else {
                    warn!("Unrecognised command: {}", received_str);
                }
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

    if let Ok(rssi) = wifi_controller.rssi() {
        info!("Wifi connected! signal: {}", rssi)
    }
}
