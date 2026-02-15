# 📡 IMU and Motor Sensor Reading Repository

This repository contains the **Python scripts** developed to receive, interpret, and process real-time sensor data from the system.

---

## 🧭 IMU (Inertial Measurement Unit) Data Acquisition
- A Python script that interfaces directly with the **IMU hardware** (via I2C) to acquire raw readings from the **accelerometer** and **gyroscope**.  
- This data is crucial for monitoring the system’s **orientation** and **kinematic state**.
- [IMU Code](imu_publisher.py)

---

## ⚡ Maxon Motor Sensor Feedback 
- A Python script that interfaces with the **A/D converters** (via software I2C) to acquire raw readings from ESCON50/5 motor drivers.
- **Functionality**: Real-time acquisition of RPM and current data for precise torque control monitoring.
- [Sensors Code](flywheel_sensor.py)

---




