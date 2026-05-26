# Autonomous Line-Following Robot Firmware

A hardware-software implementation of an autonomous line-following robot utilizing infrared sensor arrays and closed-loop control logic.

## Tech Stack & Hardware
- **Microcontroller:** ESP32
- **Development Environment:** Arduino IDE 
- **Language:** C/C++
- **Actuators:** Dual DC Motor Setup with L298N H-Bridge Driver
- **Sensors:** 5x IR transiever modules

## Core Architecture & Logic
- **Sensor Calibration:** Implements a baseline thresholding algorithm  to account for varying ambient light conditions and track reflectivity.
- **Control Strategy:** Dual-differential motor driving logic. Parses digital/analog sensor inputs to execute instant trajectory corrections.
- **Non-Blocking Execution:** Structured code to minimize processing delays between sensor polling cycles and hardware motor response.

## 📂 Repository Structure
- `/src`: Contains the primary `.ino` source code files.
