# STM32 Gimbal Controller (MPU6050 + Mahony Filter)

Embedded gimbal stabilization firmware using STM32F4 and MPU6050.

## 📌 Overview

This project implements a basic 2-axis gimbal stabilization system using:

- STM32F4
- MPU6050 IMU
- Mahony AHRS filter
- Angle estimation (Roll, Pitch, Yaw)
- PID control (in progress)

The system reads IMU data via I2C, estimates orientation using Mahony filter, and outputs control signals for motor stabilization.

---

## 🧠 System Architecture

IMU (MPU6050)  
→ Sensor Calibration  
→ Mahony AHRS  
→ Angle Estimation  
→ PID Controller  
→ PWM Output (Motor Control)

---

## ⚙️ Hardware

- STM32F4 (tested on F407)
- MPU6050 (I2C)
- USB Debug (ITM printf)
- Motor driver (planned)

---

## 📂 Project Structure

