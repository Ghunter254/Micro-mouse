

# Micromouse: STM32 Autonomous Navigator

An autonomous, bare-metal maze-solving robot built on the STM32F411 (Black Pill) microcontroller. 

This codebase implements a custom Sensor Fusion strategy utilizing an IMU, Ultrasonic sensors, and a highly optimized memory map to achieve robust maze navigation and SLAM (Simultaneous Localization and Mapping).

## The Engineering Architecture
Standard Micromouse navigation requires precise linear and angular displacement data. To achieve reliable autonomous navigation and minimize cumulative drift, this system implements a multi-layered localization strategy:

1. **Gyroscope PID Control:** Maintains absolute straight-line vectors and eliminates angular drift during high-speed runs.
2. **Ultrasonic Distance Anchoring:** Utilizes the physical maze walls as absolute positioning milestones to continuously calibrate and correct longitudinal tracking.
3. **Memory-Mapped Landmarks:** Deploys a highly compressed 8-bit array to store spatial metadata, allowing the microcontroller to predict upcoming physical constraints and verify its position within the grid.

---

## Hardware Stack
* **MCU:** STM32F411CEU6 "Black Pill" (ARM Cortex-M4 @ 100MHz)
* **Motor Driver:** L298N / L293D Dual H-Bridge
* **Actuators:** 2x Micro Metal Gearmotors
* **Sensors:** * 4x HC-SR04 / Ultrasonic Sensors (Front, Back, Left, Right)
    * 1x MPU6050 6-DOF IMU (I2C)
* **Power:** 2S Li-Po (7.4V) stepped down for logic.

---

## Software Architecture
Written in bare-metal C++ utilizing the STM32duino core. The architecture avoids blocking delays, utilizing hardware timers and state machines to ensure real-time responsiveness.

### Core Modules
* **Config.h:** Global pin definitions, hardware constants, and control loop tuning parameters.
* **Sensor.cpp / .h:** Non-blocking state machine for ultrasonic polling (30ms intervals) and fast, fixed-point IMU integration.
* **Motors.cpp / .h:** Hardware PWM generation (analogWriteFrequency at 20000Hz) and motor state control.
* **Navigation.cpp / .h:** The core logic engine. Handles the operational state machine (IDLE, EXPLORE, SPRINT), pathfinding, and movement calibration.

### The 3-Bit Map Localization System
To navigate the 16x16 maze (180mm cells) efficiently given embedded memory constraints, the `_maze[16][16]` array is heavily optimized. A single 8-bit integer stores wall layouts, exploration status, and crucial localization metadata:

| Bits | Function | Description |
| :--- | :--- | :--- |
| **0-3** | **Walls** | North, East, South, West (1 = Wall present). |
| **4** | **Visited** | Exploration flag (1 = Visited, 0 = Unknown). |
| **5-6** | **Front-Anchor**| Distance (in cells) to the next dead-ahead wall. Used for predictive deceleration and position verification. |
| **7** | **Alignment** | Corridor flag. 1 = Safe to use lateral sensors for centering algorithms. |

---

## Flashing & Setup

### Toolchain Requirements
* Arduino IDE 2.x
* Manual Core Install: `Arduino_Core_STM32-2.12.0`
* Compiler: `xpack-arm-none-eabi-gcc`

### Uploading to the Black Pill (DFU Mode)
Because this project utilizes the native USB hardware on the STM32, flashing requires DFU mode:
1. Connect the Black Pill via USB-C.
2. Press and hold the **BOOT0** button.
3. Tap the **NRST** (Reset) button.
4. Release the **BOOT0** button.
5. In your IDE, select **STM32CubeProgrammer (DFU)** as the upload method.
6. Compile and Upload.

---

## Future Improvements
* **Enhanced Odometry:** Integration of physical wheel encoders or PMW-series Optical Flow sensors for higher-resolution dead reckoning.
* **Sensor Upgrades:** Swap lateral ultrasonic sensors for VL53L0X Time-of-Flight (ToF) IR sensors for millimeter-accurate centering at higher velocities.
* **Hardware Interface:** Implement a multi-pin rotary encoder to act as a physical menu selector for pre-run configuration and mode selection.

---

**Author:** Paul Otsyula Yona  
**Institution:** Strathmore University, School of Computing and Engineering Sciences  
**Date:** April 2026
