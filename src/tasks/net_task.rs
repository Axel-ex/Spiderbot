extern crate alloc;

use alloc::string::String;
use core::str::FromStr;
use embassy_net::{tcp::TcpSocket, IpListenEndpoint, Stack};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Sender};
use embassy_time::{Duration, Timer};
use esp_wifi::wifi::{ClientConfiguration, WifiController, WifiDevice};
use log::{debug, error, info};

use crate::commands::TcpCommand;

/// Keeps the net stack up. (processes network events)
#[embassy_executor::task]
pub async fn runner_task(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}

//TODO: This task will be constructed with a channel, that communicates the right enum variant upon
//receiving the corresponding tcp packet
#[embassy_executor::task]
pub async fn tcp_server(
    stack: Stack<'static>,
    cmd_sender: Sender<'static, CriticalSectionRawMutex, TcpCommand, 3>,
) {
    info!("Waiting for network link..");
    while !stack.is_link_up() {
        Timer::after_millis(200).await;
    }
    if let Some(config) = stack.config_v4() {
        info!("Net interface up: IP {:?}", config.address);
    } else {
        info!("Net interface up!");
    }

    let mut rx_buf = [0u8; 512];
    let mut tx_buf = [0u8; 512];
    let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
    socket.set_timeout(Some(Duration::from_secs(2)));

    loop {
        info!("Listening for incoming TCP connections..");
        //WARN: This can lead to infinite loop
        if socket
            .accept(IpListenEndpoint {
                addr: None,
                port: 1234,
            })
            .await
            .is_err()
        {
            error!("Socket accept failed");
            continue;
        }

        info!("Client connected!");

        let mut buf = [0u8; 256];
        loop {
            match socket.read(&mut buf).await {
                Ok(n) => {
                    debug!("received: {n} bytes");
                    let data = &buf[..n];
                    if let Ok(cmd) = TcpCommand::try_from(core::str::from_utf8(data).unwrap()) {
                        cmd_sender.send(cmd).await;
                    } else {
                        info!("Couldnt parse the cmd");
                    }
                    break;
                }
                Err(e) => {
                    error!("{:?}", e);
                    break;
                }
            }
        }
        socket.close();
        Timer::after_millis(20).await;
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
