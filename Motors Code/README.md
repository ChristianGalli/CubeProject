# 🏎️ Motor and Brake Control Repository

This repository contains the **low-level control software** for the main Maxon motors and the associated braking system.  
The control architecture is divided into two main components:

---

## ⚙️ Main Motor Control (Python & Arduino)

### 🔌 Arduino Sketch
- Interfaces directly with the **motor drivers** (Maxon ESCON50/5).  
- Receives commands via **serial port** specifying:
  - Direction
  - Speed (RPM)  
- Translates commands into the required **PWM** and **digital signals** to drive the motors.
- [Arduino code](arduino controller.ino)

### 🐍 Python Script
- Works as a **high-level controller**.  
- Sends **serial commands** to the Arduino.  
- Orchestrates **coordinated movement** of the main motors.
- [Controller code](motor_controller.py)

---

## 🛑 Brake Control (Python / ROS 2)

- Implemented as a **ROS 2 node in Python**, designed to run on a **Raspberry Pi**.  
- Directly manages the braking system’s motors via **GPIO pins**.  
- Executes specific cycles:
  1. Engagement  
  2. Holding  
  3. Release  
- Receives commands via a **ROS topic** for synchronized operation.
- [Brakes code](brakes_control.py)



