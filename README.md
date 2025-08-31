# Drone Auto Pilot

🚧 **Work in Progress** 🚧  
A custom drone flight controller and autopilot built with a Raspberry Pi Pico W, IMU sensors, and external controllers.  

---

## 🛠️ Hardware

- **Flight Controller:**  
  - Raspberry Pi Pico W  
  - MPU9250 (9-axis gyroscope & accelerometer)  
  - BMM150 (magnetometer)  
  - Chassit is 3D-printed

- **Controller Options:**  
  - Laptop program connected via Pico W WiFi (⚠️ currently slow)  
  - **Faster option:** FlySky handheld transmitter + receiver  
    [🔗 Buy here](https://www.aliexpress.com/item/1005006102515851.html?src=google&pdp_npi=4%40dis!SEK!1544.88!478.92!!!!!%40!12000035754793147!ppc!!!)  

- **Motors & ESCs:**  
  - ESC: [40A Brushless ESC](https://rcdrone.top/sv/products/40a-brushless-esc-speed-controller)  
  - Motors: [1000KV Brushless Motors (D2826)](https://fyndiq.se/produkt/borstlos-motor-d2826-med-adapter-ersattningsmotor-for-rc-flygplan-1000kv-96d02990da3c41fd/)  

---

## 🧑‍💻 Software

- Sensor fusion: **Mahony filter**  
- Signal processing: **Low-pass filter** (plus additional filters in progress)  
- Planned features:  
  - Stable autopilot loop  
  - Real-time control via handheld transmitter  
  - WiFi-based telemetry  

---

## 📌 Status

- ✅ Basic hardware setup complete  
- ⚠️ WiFi-based control is functional but too slow for real flight  
- 🔄 Integrating FlySky controller for low-latency control  
- 🚀 Next step: refining filters and implementing stable flight control loop  

---

## 📖 Notes

This is an **experimental project** under active development.  
Use at your own risk — drones can be dangerous if not handled properly!  
