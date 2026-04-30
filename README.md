# Autonomous-Self-Balancing-Robot
An inverted pendulum robotic system utilizing PID control and MPU6050 DMP sensor fusion for real-time vertical stability.

**Link Video**: https://youtu.be/tN_EWRsiDz8

## Project Overview
This project implements a self-balancing robot based on the inverted pendulum principle. The system maintains equilibrium on two wheels by actively adjusting motor movements in response to tilt changes, utilizing a high-frequency closed-loop control system.

## Technical Features
*   **PID Control Algorithm**: Fine-tuned constants ($K_p=30, K_i=100, K_d=1.5$) to achieve stability and reduce oscillations.
*   **Sensor Fusion (MPU6050 DMP)**: Utilizes the Digital Motion Processor (DMP) to compute yaw-pitch-roll data internally, reducing CPU load while providing stable angle readings.
*   **Real-time Actuation**: High-speed communication with the L298N driver to control DC motors with precision feedback.

## Performance Metrics
*   **Rising Time**: 0.5 - 0.7 seconds.
*   **Settling Time**: ~5-6 seconds.
*   **Steady-State Error**: 0.44°.

## Hardware Stack
*   Arduino Uno (Microcontroller).
*   MPU6050 (IMU Sensor).
*   L298N (Motor Driver).
*   DC Motors with Gearboxes.
