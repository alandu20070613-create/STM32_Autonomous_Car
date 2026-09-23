# STM32 FreeRTOS Smart Car Project

## Overview
This project is a multi-functional intelligent robotic car based on STM32F103 MCU, developed with STM32CubeIDE and HAL library. The system adopts **FreeRTOS real-time operating system** to manage multiple sensor tasks and control logic, realizing stable and intelligent vehicle motion control.

The car integrates speed closed-loop PID, attitude PID, infrared line tracking, ultrasonic obstacle avoidance, object following and serial port parameter tuning, forming a complete embedded real-time control system.

## Hardware System
- **Main Controller**: STM32F103RCT6
- **Power & Drive**: Dual DC motors with quadrature encoder
- **IMU Module**: MPU6050 6-axis attitude sensor (DMP attitude solution)
- **Tracking Sensor**: 4-channel infrared tracking sensor array
- **Distance Sensor**: HC-SR04 ultrasonic ranging module
- **Display Module**: 0.96-inch OLED screen
- **Communication**: USART1 / USART3 serial communication

## Core Functions & Control Algorithm
### 1. Motor Speed Closed-loop PID Control
Obtain real-time motor speed through encoder timer interruption, calculate PWM output by PID algorithm, realize stable speed regulation of dual motors, and improve vehicle motion stability.

### 2. Infrared Line Tracking (PID Correction)
Collect road track information through infrared sensor array, calculate offset error, and use independent PID algorithm to differentially adjust the speed of left and right motors to achieve smooth and high-precision line tracking.

### 3. Ultrasonic Obstacle Avoidance
Real-time distance detection via HC-SR04. When encountering obstacles, the car will automatically judge left and right space, perform steering avoidance and route reset, realizing autonomous obstacle avoidance logic.

### 4. Intelligent Object Following
Take the detected distance as the error input, calculate the follow speed through distance PID, automatically adjust forward and backward speed according to the distance of the target object, and realize stable automatic following function.

### 5. MPU6050 Attitude Stabilization Control
Use MPU6050 built-in DMP engine to obtain accurate yaw angle. Through attitude PID algorithm, correct car deflection and realize heading angle self-stabilization.

### 6. Serial Port JSON PID Tuning
Support online modification of PID parameters (P/I/D) and target speed through serial port JSON data parsing, which is convenient for algorithm debugging and parameter optimization.

## Working Modes (Switchable by Key_Mode)
- **Mode 0**: Standby mode, real-time display of sensor data and motor speed
- **Mode 1**: PID infrared line tracking
- **Mode 2**: Computer serial remote control
- **Mode 3**: Ultrasonic intelligent obstacle avoidance
- **Mode 4**: Automatic object following
- **Mode 5**: MPU6050 yaw attitude stabilization control

## Software Architecture
- **Development Platform**: STM32CubeIDE
- **Library**: STM32 HAL Library
- **RTOS**: CMSIS-FreeRTOS (multi-task scheduling)
- **Algorithm**: Multi-group independent PID closed-loop control
- **Third-party Library**: cJSON, MPU6050 DMP driver

## Project Structure


## Build & Flash Guide
1. Open the `.ioc` file via STM32CubeMX
2. Regenerate peripheral initialization code
3. Import the project into STM32CubeIDE
4. Compile, build and download firmware to MCU

## Project Features & Personal Gains
This project integrates **sensor acquisition, timer interrupt, motor PID control, real-time operating system scheduling, attitude solving and serial communication data parsing**.

It fully demonstrates the ability of embedded hardware debugging, algorithm transplantation and real-time program architecture design.

