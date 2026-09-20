# System Architecture

## Overview

The vehicle uses a distributed architecture:

- Raspberry Pi 4: High-level autonomy and perception.
- ESP32: Low-level control and sensor acquisition.

## Raspberry Pi Responsibilities

- ROS 2 Jazzy
- LiDAR driver
- Camera driver
- Lane detection
- Stop sign detection
- GPS waypoint navigation
- Path planning
- Obstacle avoidance
- High-level vehicle commands

## ESP32 Responsibilities

- Motor control
- Wheel encoder acquisition
- BNO055 IMU acquisition
- Steering servo control
- Low-level safety and control loops

## Communication

The ESP32 communicates with the Raspberry Pi using a ROS 2 compatible interface.

The communication method will be selected during implementation:
- Serial
- Wi-Fi
- micro-ROS

## Data Flow

### ESP32 to Raspberry Pi

- Wheel encoder measurements
- Wheel odometry
- IMU data
- Motor feedback
- Steering feedback (if available)

### Raspberry Pi to ESP32

- Target motor speed
- Steering angle
- Emergency stop
- Vehicle control commands

## Autonomy Pipeline

1. Acquire sensor data.
2. Estimate vehicle position.
3. Detect lanes and traffic signs.
4. Detect obstacles using LiDAR.
5. Plan a safe trajectory toward GPS waypoints.
6. Generate steering and speed commands.
7. Execute commands through the ESP32.

## Design Principles

- Modular ROS 2 nodes.
- Separation of high-level and low-level control.
- Hardware abstraction.
- Testability.
- Safe fallback behavior.
