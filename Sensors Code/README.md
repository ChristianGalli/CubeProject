# 📡 IMU and Motor Sensor Reading Repository

This repository contains the **Python scripts** developed to receive, interpret, and process real-time sensor data from the system.

---

## 🧭 IMU (Inertial Measurement Unit) Data Acquisition
- A Python script that interfaces directly with the **IMU hardware** (via I2C) to acquire raw readings from the **accelerometer** and **gyroscope**.  
- This data is crucial for monitoring the system’s **orientation** and **kinematic state**.
- [IMU Code](imu_publisher.py)

---

## ⚡ Maxon Motor Sensor Feedback 
- A Python script that listens to the **serial port** to acquire motor feedback data, such as speed in **RPM**.  
- The data is transmitted by an **Arduino sketch** acting as an intermediary between the motor drivers (ESCON50/5) and the main computer.
- [Arduino Code](../Motors%20Code/arduino%20controller.ino)
- [Sensors Code](flywheel_sensor.py)

---




