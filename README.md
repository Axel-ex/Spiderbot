# **ESP32-Rust Quadruped Spider Robot**

<div align="center">
  <img src=".assets/demo.jpg" width="60%" alt="Weather Station Demo" />
</div>


This repository contains the firmware for a 3D-printable, Wi-Fi controlled quadruped (spider) robot. The brain of the robot is an ESP32 microcontroller, and the firmware is written entirely in Rust using the embassy asynchronous framework for real-time performance.

This project is heavily inspired by the [DIY Spider Robot](https://www.instructables.com/DIY-Spider-RobotQuad-robot-Quadruped/) on Instructables.

## **Introduction**

This project aims to create an accessible yet advanced robotic platform. By leveraging the power of the ESP32 and the safety and concurrency of Rust, we can achieve complex movements and remote control capabilities. The robot has 12 degrees of freedom (3 per leg) and uses inverse kinematics to calculate the joint angles required to position its feet in 3D space.

Control is handled remotely over Wi-Fi via a simple TCP server running on the ESP32, allowing you to send commands from any computer on the same network.

## **Hardware**

### **Bill of Materials**

| Component | Quantity | Notes |
| :---- | :---- | :---- |
| ESP32 DevKitC (or similar) | 1 | The main microcontroller. |
| PCA9685 16-Channel Servo Driver | 1 | Controls all servos over I2C. Essential for synchronous movement. |
| SG90 Micro Servos | 12 | One for each joint of the robot's four legs. |
| 5V Power Supply | 1 | A stable power source capable of at least 3A (e.g., LiPo \+ Buck Converter or wall adapter). |
| 3D Printed Robot Chassis | 1 | Files can be found in the original [Instructables guide](https://www.instructables.com/DIY-Spider-RobotQuad-robot-Quadruped/). |
| Jumper Wires | Various | For connecting the components. |

### **Wiring Diagram**

**ESP32 connections**

You must power the servos separately from the ESP32. The ESP32's onboard regulator cannot supply the high current the 12 servos require.

| ESP32 Pin | Connects to |
| :---- | :---- |
| GND | PCA9685 GND |
| 3v3 | PCA9685 VCC |
| GPIO22 | PCA9685 SCL |
| GPIO21 | PCA9685 SDA |

**Power Connections:**

* Connect your **5V 3A+ Power Supply** \+ terminal to the V+ terminal on the PCA9685. This powers the servos.  
* Connect the Power Supply \- terminal to the GND terminal on the PCA9685.  
* **Important:** Ensure the ESP32's GND is also connected to the Power Supply's GND to create a common ground reference.

**Servo Connections:**

* Plug the 12 servos into the PCA9685 channels as defined below. Each servo has three wires: GND (Brown), V+ (Red), and Signal (Orange).  
* The servo channels are assigned in arrays, with the required joint order being \[Femur, Tibia, Coxa\].

| Leg | PCA9685 Channels |
| :---- | :---- |
| Front Left | \[C15, C14, C13\] |
| Bottom Left | \[C12, C11, C10\] |
| Front Right | \[C0, C1, C2\] |
| Bottom Right | \[C3, C4, C5\] |

## **Software Design**

The firmware is written in Rust and built upon the embassy asynchronous framework. This allows for clean, non-blocking management of networking, motion planning, and low-level motor control simultaneously.

### **Architecture Overview**

The software is divided into three primary asynchronous tasks:

1. **net\_task:**  
   * Connects the ESP32 to your local Wi-Fi network.  
   * Starts a TCP server on port 1234\.  
   * Listens for incoming string-based commands (e.g., sf 4).  
   * Parses these commands and sends them to the motion\_task for execution.  
2. **motion\_task:**  
   * The "brain" of the robot. It receives high-level commands from the net\_task.  
   * It uses a GaitEngine state machine to translate simple commands into a sequence of precise leg movements.  
   * It calculates the target Cartesian coordinates (X, Y, Z) for each leg's endpoint.  
   * This position data is then sent to the servo\_task.  
3. **servo\_task:**  
   * The low-level controller that directly interfaces with the hardware.  
   * It receives target coordinates and movement speeds from the motion\_task.  
   * It performs **Inverse Kinematics** calculations (conversion.rs) to convert the (X, Y, Z) coordinates into the three required servo angles (alpha, beta, gamma) for that leg.  
   * It smoothly interpolates the servo positions from their current state to the target state, preventing jerky movements.  
   * Finally, it communicates with the PCA9685 driver over I2C to set the final PWM signals for each servo.
                                                                                                                                                                                                                                                                 e
The original code uses a global shared state and many variables in the global scope. As a result, it is hard for the reader to understand which part of the code is mutating those variables and overall to follow the flow of data. This design solves the issue by maintaining clear owner of resources: The motion_task is the task that own the robot positions and mutate these values, the servo task is "unaware" of the positions and simply update the pulses it writes to the servo.

### **Why a PCA9685 Servo Driver?**

While the ESP32 has a built-in PWM generator (ledc), it's not ideal for robotics applications requiring synchronized multi-joint movement (figured that out the hard way…). Each ledc channel runs on its own timer. When you command a leg to move, its three servos need to start and stop turning at *precisely* the same time for a smooth, coordinated motion.

The **PCA9685** solves this problem. It's a dedicated I2C PWM driver that allows you to update the pulse width for all 16 channels in a single, atomic-like operation. The firmware can calculate the positions for all 12 servos and send the update command, ensuring all servos refresh in unison. This results in the fluid, life-like motion essential for a legged robot.

## **Build It Yourself**

### **Step 1: Assembly**

First, 3D print and assemble the robot's chassis and legs. Follow the excellent guide provided in the original [Instructables project](https://www.instructables.com/DIY-Spider-RobotQuad-robot-Quadruped/).

### **Step 2: Wiring**

Wire the components together according to the [Wiring Diagram](https://www.google.com/search?q=%23wiring-diagram) section above. Be very careful with the power supply connections to avoid damaging your components.

### **Step 3: Flashing the Firmware**

**Prerequisites:**

* A working Rust environment (see [rustup.rs](https://rustup.rs/)).  
* The xtensa-esp32-elf target installed: rustup target add xtensa-esp32-elf

**Configuration:**

1. Clone this repository.  
2. Rename the file .cargo/config.toml.example to .cargo/config.toml.  
3. Open the new .cargo/config.toml file and add your Wi-Fi credentials under the \[env\] section:  
   \[env\]  
   WIFI\_SSID="YourNetworkName"  
   WIFI\_PASS="YourNetworkPassword"

4. Connect the ESP32 to your computer via USB.

Build and Flash:  
The project is configured to build and flash automatically. Run the following command in your terminal:  
cargo run

### **Step 4: Controlling the Robot**

1. Once flashed, the robot will connect to your Wi-Fi. The cargo run command will open a serial monitor where you can find the IP address it was assigned (e.g., 19\_2.168.1.123).  
2. You can control the robot by sending TCP commands to it on port 1234\. A simple way to do this is with netcat (nc) or telnet.

\# Example: Tell the robot to take 2 steps forward  
echo "sf 2" | nc 192.168.1.123 1234

\# Example: Tell the robot to turn left 4 times using telnet  
telnet 192.168.1.123 1234  
\> stand  
\> tl 4

Available Commands:  
| Command | Description | Example |  
| :------ | :---------------------------------------- | :-------- |  
| test | Runs a pre-programmed demo sequence. | test |  
| stand | Puts the robot in a standing position. | stand |  
| sit | Puts the robot in a sitting/resting position. | sit |  
| sf | Walks forward N steps. | sf 2 |  
| sb | Walks backward N steps. | sb 2 |  
| tl | Turns left on the spot for N steps. | tl 4 |  
| tr | Turns right on the spot for N steps. | tr 4 |  
| w | Waves one of its front legs N times. | w 3 |  
| close | Closes the TCP connection. | close |

## **Troubleshooting**

* **Robot doesn't connect to Wi-Fi:** Double-check your SSID and password in the .cargo/config.toml file. Check the serial monitor for any error messages from the ESP32.  
* **Servos are jittery or don't move:** This is almost always a power issue. Your 5V power supply cannot provide enough current. Ensure it is rated for at least 3A and that the wires are thick enough. Adding a large capacitor (e.g., 1000uF) across the V+ and GND rails of the PCA9685 can help smooth out power delivery.  
* **Robot doesn't respond to commands:**  
  * Confirm you have the correct IP address from the serial monitor.  
  * Check your I2C wiring between the ESP32 and PCA9685.  
  * **Note:** This firmware uses a non-standard I2C address of 0x7f for the PCA9685. Most modules default to 0x40. Check if your module has solder pads to change the address, or you may need to update the address in main.rs.  
* **Legs move in the wrong direction:** This likely means a servo was mounted facing the wrong way during assembly. You may need to either physically remount the servo.
