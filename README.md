# Drone Auto Pilot  

🚧 **Work in Progress** 🚧  
A custom drone flight controller and autopilot built with a **Raspberry Pi Pico W**, IMU sensors, and external controllers.  

[![Build Status](https://img.shields.io/badge/status-in%20progress-yellow)]()  
[![License](https://img.shields.io/badge/license-MIT-blue)]()  

---

## 🛠️ Hardware  

- **Flight Controller**  
  - Raspberry Pi Pico W  
  - MPU9250 (9-axis gyroscope & accelerometer)  
  - BMM150 (magnetometer)  
  - 3D-printed chassis: [Thingiverse link](https://www.thingiverse.com/thing:264955/comments)  

- **Controller Options**  
  - Laptop program via Pico W WiFi (⚠️ works but too slow for real flight)  
  - **Recommended:** FlySky handheld transmitter + receiver  
    [🔗 Buy here](https://www.aliexpress.com/item/1005006102515851.html?src=google&pdp_npi=4%40dis!SEK!1544.88!478.92!!!!!%40!12000035754793147!ppc!!!)  

- **Motors & ESCs**  
  - ESC: [40A Brushless ESC](https://rcdrone.top/sv/products/40a-brushless-esc-speed-controller)  
  - Motors: [1000KV Brushless Motors (D2826)](https://fyndiq.se/produkt/borstlos-motor-d2826-med-adapter-ersattningsmotor-for-rc-flygplan-1000kv-96d02990da3c41fd/)  
  - (To be added: propeller size, battery specs)  

---

## 🧑‍💻 Software  

- **Sensor Fusion:** Mahony filter  
- **Signal Processing:** Low-pass filter (with more advanced filters in progress)  
- **Planned Features:**  
  - Stable autopilot loop  
  - Real-time control via handheld transmitter  
  - WiFi-based telemetry & flight data logging  

> Control flow:  
> **IMU → Filters → Control Loop (PID planned) → ESC/Motor output**  

---

## 📌 Project Status  

- ✅ Basic hardware setup complete  
- ⚠️ WiFi-based control is functional but too slow for real flight  
- 🔄 FlySky controller integration for low-latency input  
- 🚀 Next step: refining filters & implementing stable flight control loop  

---

## 📖 Notes  

This is an **experimental project** under active development.  
⚠️ Use at your own risk — drones can be dangerous if not handled properly!  

---

## 📜 License  

This project is licensed under the [MIT License](LICENSE).  
