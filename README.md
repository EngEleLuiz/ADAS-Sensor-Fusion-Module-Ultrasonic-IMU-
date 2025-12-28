# ADAS Sensor Fusion Module (Ultrasonic + IMU)

![Status](https://img.shields.io/badge/Status-Active-success)
![Language](https://img.shields.io/badge/Language-Python%20%7C%20C%2B%2B-blue)
![Topic](https://img.shields.io/badge/Topic-Kalman%20Filter%20%7C%20Embedded-orange)

## 📌 Project Overview
This repository contains a **real-time Collision Warning System** designed for embedded environments. It utilizes **Sensor Fusion** to combine noisy Ultrasonic range data with high-speed IMU acceleration data.

The core algorithm is a **Kalman Filter** with a Control Input model, allowing the system to detect braking events and update Time-To-Collision (TTC) estimates with minimal latency, overcoming the physical lag of standard position-only trackers.

## 🏗 System Architecture

The system operates in a recursive Predict-Update loop:

1.  **Prediction Step (IMU Driven):**
    The system propagates the state forward using the physical equations of motion, driven by the accelerometer ($u_k$). This provides an immediate response to speed changes.
    $$\hat{x}_{k|k-1} = F \hat{x}_{k-1} + B u_k$$

2.  **Update Step (Ultrasonic Driven):**
    The system corrects the estimated position using the ultrasonic range finder ($z_k$).
    $$\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H \hat{x}_{k|k-1})$$

## 📂 Repository Structure

```text
.
├── notebooks/
│   ├── 01_kalman_cv_basic.ipynb    # Baseline Constant Velocity Model
│   └── 02_sensor_fusion_imu.ipynb  # Fusion Model (Python Prototype)
├── src/
│   └── 03_kalman_fusion_raw.cpp    # Production C++ Code (No Dependencies)
├── logs/                           # Simulation Data Logs
├── README.md                       # Documentation
└── requirements.txt                # Python Dependencies
```
## 🚀 Key Features
Zero-Dependency C++: The src/ implementation uses raw optimized math, removing the need for heavy libraries like Eigen. Ready for bare-metal deployment.

Latency Reduction: By fusing acceleration data, the velocity estimate converges 3x faster during emergency braking than standard differentiation methods.

Dynamic TTC Alerts: * TTC > 4.0s: Safe

1.5s < TTC < 4.0s: Warning

TTC < 1.5s: Emergency Braking

## 🛠 Getting Started
1. Python Prototype (Visual Analysis)
Recommended for viewing the algorithm performance graphs.

```code
pip install -r requirements.txt
jupyter notebook notebooks/02_sensor_fusion_imu.ipynb
```

2. C++ Deployment (Real-time Simulation)
Compiles with standard G++ (no external libraries required).

```code
cd src
g++ 03_kalman_fusion_raw.cpp -o adas_system
./adas_system
```

## 📊 Performance Analysis
Complexity: O(1) constant time per step (Matrix dimensions are fixed at 2x2).

Memory Footprint: < 1KB stack usage (Ideal for Arduino/STM32).

Sensor Noise Rejection: Configurable R matrix allows tuning for cheap vs. expensive ultrasonic sensors.
