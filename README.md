# 🤖 Gyro - Two-Wheeled Self-Balancing Robot

![C++](https://img.shields.io/badge/Language-C++-blue)
![Platform](https://img.shields.io/badge/Platform-Arduino-00979D)
![Status](https://img.shields.io/badge/Status-Completed-success)

## 📌 Abstract
**Gyro** is a custom-built, two-wheeled self-balancing robot based on the classic inverted pendulum control problem. Developed as a team engineering project at the **Bialystok University of Technology**, this robot autonomously maintains an upright position using real-time sensor fusion and a custom Proportional-Derivative (PD) control loop.

During physical testing, the final prototype successfully maintained stable dynamic equilibrium for **over 60 seconds** continuously.

(📸 **https://github.com/user-attachments/assets/14bfd2c6-7836-42ce-a22d-3989b2a5717c**)

---

## 🧑‍💻 My Role in the Project
While this was a collaborative team effort, my primary responsibilities focused entirely on the **Software and Electronics** architecture:
* **Firmware Development (C++):** Wrote the entire control loop from scratch, implemented the Complementary Filter for IMU data fusion, and tuned the PD controller.
* **Hardware Integration:** Configured the Pololu Motoron M2T550 I2C driver and programmed the safety fail-safe state machine.
* **Electronics Assembly:** Soldered the custom PCB components and successfully built/integrated the 4S Battery Management System (BMS) for safe power delivery.

---

## 🛠️ Hardware Architecture & BOM

The robot's structure was custom-designed in SolidWorks and 3D printed using PLA to keep the center of gravity low and the frame rigid. 

**Key Electronic Components:**
* **Microcontroller:** Arduino Nano (ATmega328P)
* **IMU Sensor:** MPU-6050 (6-DOF Accelerometer & Gyroscope)
* **Motor Driver:** Pololu Motoron M2T550 Dual I2C Motor Controller
* **Actuators:** 2x DC Gearmotors 25GA-370
* **Power Supply:** Custom 4S Li-Ion 18650 Battery Pack with a 40A BMS

(🖼️ <img width="469" height="580" alt="image" src="https://github.com/user-attachments/assets/4872b4b1-f801-4427-8ed1-a223dc8cfde5" />)


---

## 💻 Software & Control Theory

The firmware is written in **C++** and focuses heavily on minimizing loop delays and accurately filtering physical data. 

### 1. Sensor Fusion (Complementary Filter)
Raw data from the MPU-6050 is read via a fast **400 kHz I2C bus**. To calculate the accurate tilt angle, the code implements a **Complementary Filter** (alpha = 0.98). This approach perfectly merges the high-frequency responsiveness of the gyroscope with the stable, long-term accuracy of the accelerometer.

### 2. PD Controller & Dead Zone Compensation
Balancing is achieved using a closed-loop **PD controller** (Proportional-Derivative). 
* The Integral (I) term is disabled ($K_i = 0$) to prevent wind-up in this highly dynamic system.
* **Dead Zone Compensation (DEAD_ZONE = 80):** A critical software feature that forces a minimum PWM signal to overcome the static friction (stiction) of the DC gearmotors.

### 3. State Machine & Fail-Safe Mechanism
* **Fail-Safe (Fall Detection):** If the pitch angle exceeds **35°**, the system instantly cuts power to the motors.
* **Recovery:** The robot automatically resets and re-engages the motors only when it is manually brought back to a vertical position (error < 5°).

(🖼️ <img width="887" height="1226" alt="image" src="https://github.com/user-attachments/assets/fa35368b-cf9d-4ca0-8361-3ff5e32cdc7c" />)

---

## 🚀 Getting Started

1. Assemble the hardware and ensure the battery is fully charged (up to 16.8V).
2. Upload `gyro_robot.ino` to the Arduino Nano.
3. Place the robot on a flat surface and hold it **perfectly still and vertical**.
4. Turn on the power. The robot will run a 3-second auto-calibration routine.
5. Once the console outputs `"Gotowy! Postaw robota pionowo i pusc."`, let go of the robot!

---

## 👥 Team & Credits
This project was successfully built by a team of engineering students at Bialystok University of Technology:
* **Gabriel Kozłowski (Me)** - Firmware (C++), Electronics & BMS
* **Maksymilian Jańczak** - 3D CAD Design, Schematics 
* **Kacper Kołodko** - Technical Documentation & Assembly
* **Cyprian Jarmołowicz** - Technical Documentation & Assembly
