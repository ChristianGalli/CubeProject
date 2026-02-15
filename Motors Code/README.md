# 🤖 Flywheel and Brake Control Repository  

This repository contains the **low-level control software** for the flywheel actuation and braking system.  
The complete control architecture runs on a **Raspberry Pi 5**, with all drivers and actuators directly connected to the GPIO interface.  

The detailed system design and control strategy are described in the associated paper. This repository focuses only on the implementation.

---

## 🧭 System Overview  

The control framework is based on **ROS 2**, enabling modular, real-time communication between subsystems.  
Two main nodes are implemented:

1. Flywheel motor control  
2. Solenoid brake control  

Both are written in Python and designed for embedded real-time execution.

---

## ⚙️ Flywheel Motor Control  

High-frequency controller for the flywheel motors.

### Features
- Direct GPIO and PWM control  
- Configurable current limits and PWM frequency  
- High-frequency loop (up to 1 kHz)  
- Watchdog for communication loss  
- Thread-safe architecture  
- Automatic safe shutdown  

### ROS Interface
**Topic:** `/flywheels`  
**Message:** `Float64MultiArray`  

Each motor receives:
- Direction  
- Current reference  

📄 Code:  
- [motor_controller.py](motor_controller.py)

---

## 🛑 Brake Control  

The braking system uses **electromagnetic solenoids** driven through MOSFET stages.

### Features
- Parallel actuator control  
- Runtime configurable parameters  
- Thread pool for concurrent actuation  
- State monitoring and diagnostics  
- Lock and safety mechanisms  

### ROS Interface
**Subscription topic:** `/brakes`  
**Message:** `Float64MultiArray`  

Each actuator receives:
- Activation state  
- Movement time  

**Status topic:** `/brakes/status`  

📄 Code:  
- [brakes_control.py](brakes_control.py)



---
