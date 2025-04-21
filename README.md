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
TODO: Add detailed code organization once development is complete.

---

## 👉 Division of responsibilities among team members

- **Cong Liu (3055752L)**: [e.g.A]
- **Xinkai Li (3030890L)**: [e.g. B]
- **Jiahe Chen (3049643C)**: [e.g. C]
- **Haoming Wang (2987352W)**: [e.g. D]
- **Lianxiao Yao (3048246Y)**: [e.g.E]
- **Yinjie Fan (3062833F)**: [e.g. F]

---

## **📢 Future Improvements**

🔹 **Additional visual display modes**, such as different sand animations or symbol-based representations.  
🔹 **Wireless remote control**, allowing users to configure settings via WiFi/Bluetooth.  
🔹 **Data storage & visualization**, enabling users to track historical temperature & air pressure readings via a web interface.  

## **📝References & Acknowledgments**

🔹 [**Madgwick's filter**](https://github.com/xioTechnologies/Open-Source-AHRS-With-x-IMU): Madgwick algorithm for orientation estimation.

🔹 [**XRobot**](https://github.com/xrobot-org/XRobot): An embedded software framework for MCU, Arm/x86 Linux and simulator.

## **🛠️ Compile & Installation**

See [**Usage Guide**](https://fluxsand.github.io/3.run/README.html) to compile and install the program.

Full program need to be compiled and installed on the Raspberry Pi 5B. But the unit test can be run on any Linux platform with onnxruntime.

## 📸 Social Media

Please see our [Instagram](https://www.instagram.com/fluxsand/reels/)
