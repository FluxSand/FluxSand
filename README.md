# **FluxSand - Interactive Digital Hourglass**

<div align=center>
<img src="./imgs/FluxSand.jpg" height="200">
<p>

[![pages-build-deployment](https://github.com/FluxSand/FluxSand.github.io/actions/workflows/pages/pages-build-deployment/badge.svg)](https://github.com/FluxSand/FluxSand.github.io/actions/workflows/pages/pages-build-deployment)

[![Train and Release Model](https://github.com/FluxSand/ModelTrainer/actions/workflows/build.yml/badge.svg)](https://github.com/FluxSand/ModelTrainer/actions/workflows/build.yml)

</div>

🚀 **FluxSand** is an **interactive digital hourglass** designed using **real-time embedded systems**. It combines **gyroscope sensing, LED matrix display, and touch interaction** to create a dynamic and visually engaging timekeeping experience. By leveraging advanced sensor fusion and real-time processing, this system responds seamlessly to user actions, providing an immersive interaction.

---

## **📌 Project Overview**

**FluxSand** utilizes **sensor fusion technology** and **real-time processing** to simulate a physical hourglass in a unique way:

- **Smart Flow Effect**: The LED matrix dynamically displays the movement of sand grains, adapting to device orientation.
- **Multiple Modes**:
  - **Pomodoro Timer Mode**
  - **Weather Clock Mode**
  - **Timing/Countdown Mode**
- **Enhanced Physical Interaction**: Users can flip the device or use touch buttons to control functions like pause, speed adjustment, or reset.
- **Adaptive Brightness**: A light sensor detects ambient light levels and automatically adjusts LED brightness for optimal visibility.
- **Audio Feedback**: The buzzer provides audio cues when the countdown finishes or when switching modes, enhancing user experience.

<div align=center>
<img src="./imgs/preview.png">
<p>
</div>

---


**📌 Key Technologies**  

- **Event-driven programming**: Uses **callbacks** to process sensor inputs & LED refresh, avoiding blocking operations and ensuring responsiveness.
- **Multithreading control**: Separates data acquisition & display updates for real-time performance.
- **GitHub version control**: Implements **Git for version tracking**, including commit history, issue tracking, and pull requests for structured development.

---

## **🚀 Development Progress**

🔄 **Hardware selection & procurement** ························································································✅[Completed]  
🔄 **Initial code framework setup (C++ & sensor drivers)** ···················································✅[Completed]    
🔄 **Optimization of real-time data processing** ·········································································✅[Completed]    
🔄 **Enhancing user interaction (touch buttons & LED animation)** ·································✅[Completed]    
🔄 **Software testing & debugging** ··································································································✅[Completed]    
📢 **Project promotion (social media & Hackaday)** ·································································✅[Completed]  

---

## **🎯 Key Features**

✅ **Real-time sensor data acquisition**: The MPU-9250 gyroscope & accelerometer detect device orientation to control sand flow direction and speed.  
✅ **High-efficiency LED visual display**: MAX7219 LED matrix presents fluid hourglass effects and weather information.  
✅ **Touch button interaction**: Users can switch between different modes, such as a countdown timer or a weather clock.  
✅ **Smart brightness adjustment**: The light sensor automatically modifies LED brightness based on ambient conditions.  
✅ **Audio feedback**: The buzzer signals key events, such as countdown completion or mode changes.  
✅ **Temperature & air pressure detection**: The AHT20 + BMP280 module measures ambient temperature and air pressure readings for the weather clock mode.  

---

## **🔧 Hardware Components**

| Component                 | Specification                              | Quantity | Purpose                                             |
| ------------------------- | ------------------------------------------ | -------- | --------------------------------------------------- |
| **MAX7219 LED Matrix**    | 8x8 units                                  | 8        | Hourglass visualization & weather clock display     |
| **AHT20 + BMP280 Module** | Temperature, humidity, and pressure sensor | 1        | Weather clock mode measurements                     |
| **GY-9250 Module**        | 9-axis gyroscope & accelerometer           | 1        | Orientation detection & sand flow control           |
| **TTP223 Touch Button**   | Capacitive                                 | 2        | Interaction control (mode switching)                |
| **Buzzer**                | 3V active                                  | 1        | Audio alerts                                        |
| **Light Sensor**          | 5506                                       | 1        | Light detection for automatic brightness adjustment |
| **ADS1115**               | 16-bit ADC module                          | 1        | Analog signal conversion                            |
| **Thermistor (MF11)**     | 10kΩ NTC                                   | 1        | Temperature compensation                            |
| **Type-C Connector**      | DIY solderable                             | 1        | Power supply connection                             |
| **Type-C Cable**          | 24P 3A data cable                          | 1        | Data & power transfer                               |
| **Wires/Dupont Lines**    | Various                                    | -        | Circuit connections                                 |
| **Prototyping Board**     | Breadboard                                 | -        | Circuit assembly                                    |

---


## **💻 Software Architecture**

This project is developed primarily in **C++**, running on a **Linux + Raspberry Pi** platform, utilizing an **event-driven real-time architecture** to ensure seamless interactions.

**📌 Code Structure**

The source code is organized under the `src/` directory as follows:

```
src/
├── main.cpp               # Main entry point of the program
├── ahrs/                  # AHRS (Attitude and Heading Reference System) algorithm module
│   ├── ahrs.cpp
│   └── ahrs.h
├── drivers/               # Hardware drivers
│   ├── mpu9250.cpp        # Driver for MPU9250 gyroscope and accelerometer
│   ├── mpu9250.h
│   ├── led_matrix.cpp     # Control logic for MAX7219 LED matrix
│   └── led_matrix.h
├── gui/                   # GUI and display logic
│   ├── display.cpp
│   └── display.h
└── utils/                 # Utility functions and common helpers
    ├── timer.cpp
    └── timer.h
```

Additional directories and files:

- `third_party/libxr/`: Integrated third-party library for extended functionality.
- `imgs/`: Visual assets and diagrams used in the README.
- `.vscode/`: Editor configuration files for VSCode development environment.
- `.clang-format`, `.clangd`: Code formatting and language server configuration.
- `CMakeLists.txt`: CMake configuration file for building the project.

---


## 👉 Division of responsibilities among team members

- **Cong Liu (3055752L)**: Designed and implemented AHRS, systemd service integration, LED smoothing, and overall architecture control.
- **Xinkai Li (3030890L)**: Integrated ADS1115 module; refactored and cleaned up third-party dependencies.
- **Jiahe Chen (3049643C)**: Refined sensor fusion algorithms, improved inference runtime behavior, and optimized visual output handling.
- **Haoming Wang (2987352W)**: Developed drivers for I2C peripherals including AHT20, BMP280, MAX7219; contributed to device interfacing.
- **Lianxiao Yao (3048246Y)**: Developed GUI logic including countdown display, implemented motion-based flow behavior and fixed direction control.
- **Yinjie Fan (3062833F)**: Implemented GUI components, humidity/temperature rendering, and contributed to LED matrix port control.


---

## **📢 Future Improvements**

🔹 **Additional visual display modes**, such as different sand animations or symbol-based representations.  
🔹 **Wireless remote control**, allowing users to configure settings via WiFi/Bluetooth.  
🔹 **Data storage & visualization**, enabling users to track historical temperature & air pressure readings via a web interface.  

## **📝References & Acknowledgments**

🔹 [**Madgwick's filter**](https://github.com/xioTechnologies/Open-Source-AHRS-With-x-IMU): Madgwick algorithm for orientation estimation.

🔹 [**XRobot**](https://github.com/xrobot-org/XRobot): An embedded software framework for MCU, Arm/x86 Linux and simulator.


## **🛠️ Compile & Installation**

To compile FluxSand from source and register it as a systemd service, follow the steps below:

### 🧪 Step 1: Build the Project

```bash
mkdir build
cd build
export CC=clang
export CXX=clang++
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

You can also use multi-core compilation for better performance:

```bash
make -j$(nproc)
```

### 🛰 Step 2: Register as a systemd Service

Create a new systemd unit file `/etc/systemd/system/fluxsand.service` and paste the following content:

```ini
[Unit]
Description=FluxSand Digital Hourglass Service
After=network.target

[Service]
ExecStart=/home/YOUR_USERNAME/repository/FluxSand/build/FluxSand
WorkingDirectory=/home/YOUR_USERNAME/repository/FluxSand/build/
Restart=always
RestartSec=5
User=YOUR_USERNAME
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

Replace `YOUR_USERNAME` with your actual Linux username.

Then reload systemd and enable the service:

```bash
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable fluxsand.service
sudo systemctl start fluxsand.service
```

You can check the service status or view logs using:

```bash
systemctl status fluxsand.service
journalctl -u fluxsand.service -f
```


## **🔗 Relevant Links**

[**Documentation 📝**](https://fluxsand.github.io/)  
[**GitHub Repository 🔗**](https://github.com/FluxSand/FluxSand)  
[**Demo Video 🎥**](https://www.instagram.com/reel/DItV1Tgt_SF/?igsh=OG52cnpjYjh2Z2E0)  
[**Social Media Promotion 📢**](https://www.instagram.com/fluxsand?igsh=Z2p2bWhleHZlZGo=)