# Motor and Brake Control Repository

This folder contains the source code for the low-level control of the main Maxon motors and an associated braking system. The control architecture is divided into two distinct components:

## Main Motor Control (Python & Arduino)

- **Arduino Sketch**: Acts as a direct interface with the motor drivers (Maxon ESCON50/5). It receives commands via the serial port, specifying direction and speed (RPM), and translates them into the necessary PWM and digital signals to drive the motors.

- **Python Script**: Acts as a high-level controller. It is responsible for sending serial commands to the Arduino to orchestrate the coordinated movement of the main motors.

## Brake Control (Python/ROS 2)

- A ROS 2 node in Python, designed to run on a Raspberry Pi. This script manages the actuation of the braking system's motors by directly driving the GPIO pins. It executes specific cycles of engagement, holding, and release based on commands received on a ROS topic.

