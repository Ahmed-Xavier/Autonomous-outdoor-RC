# Autonomous Outdoor RC Car

An autonomous outdoor RC car prototype built as a distributed robotics platform and intended as a foundation for future autonomous vehicle and Shell Eco-marathon prototype work.

## Project Goals

- GPS waypoint navigation
- Camera-based lane keeping
- Stop sign detection
- LiDAR obstacle avoidance
- Sensor fusion and localization
- ESP32 low-level motor, encoder, and steering control
- ROS 2 Jazzy on Raspberry Pi 4
- Safe fallback behavior between high-level autonomy and low-level control

## System Architecture

The vehicle is split into two main computing layers:

### Raspberry Pi 4 — High-Level Autonomy

- Ubuntu 24.04
- ROS 2 Jazzy
- LiDAR driver and obstacle perception
- Hikvision camera processing
- Lane detection
- Stop sign detection
- GT-U7 GPS/GNSS integration
- Localization and waypoint navigation
- Path planning and obstacle avoidance
- High-level speed and steering commands

### ESP32 — Low-Level Control

- Motor control through an L298N
- JGA25-370 motor control
- Quadrature encoder acquisition
- Steering servo control
- BNO055 IMU integration
- Low-level safety and command timeout handling
- Wi-Fi micro-ROS communication with the Raspberry Pi

## Current Hardware

| Component | Hardware | Role |
|---|---|---|
| Main computer | Raspberry Pi 4 Model B | ROS 2 and autonomy |
| Low-level controller | ESP32 | Motor, encoder, servo and low-level I/O |
| Motor | JGA25-370 | Vehicle propulsion |
| Motor driver | L298N | H-bridge motor control |
| Encoder | Integrated JGA25-370 encoder | Wheel/motor feedback |
| GPS | GT-U7 | Outdoor position and navigation |
| IMU | BNO055 | Orientation and motion sensing |
| Camera | Hikvision USB camera | Vision and perception |
| LiDAR | LiDAR sensor | Obstacle detection and mapping |
| Steering | RC steering servo | Front-wheel steering |

## ESP32 Communication

The ESP32 communicates with the Raspberry Pi over Wi-Fi using **micro-ROS**.

The current firmware provides:

- `/esp32/motor_cmd`
- `/esp32/servo_cmd`
- `/esp32/encoder`

The encoder is currently published at approximately 20 Hz.

## Hardware Wiring

The JGA25-370 motor/encoder cable:

| Wire color | Function |
|---|---|
| Yellow | Encoder A |
| White | Encoder B |
| Blue | Encoder VCC, 5 V |
| Green | Encoder GND |
| Black | Motor GND / negative |
| Red | Motor +12 V / positive |

ESP32 control pins are documented in `docs/hardware.md` and match the current firmware.

## Current Status

### Working / tested

- ESP32 Wi-Fi micro-ROS communication
- L298N motor control
- Steering servo control
- JGA25-370 encoder acquisition
- Encoder publishing at approximately 20 Hz
- Raspberry Pi micro-ROS agent
- ROS 2 topic communication
- Foxglove visualization
- Map visualization/testing in Foxglove using a Google Earth screenshot

### In progress

- ROS 2 ESP32 bridge node
- Standardized vehicle interface messages
- GT-U7 GPS integration
- BNO055 integration
- Camera perception
- LiDAR integration
- Localization and sensor fusion
- Autonomous waypoint navigation
- Lane and traffic-sign perception
- Obstacle avoidance

## Repository Structure

- `firmware/esp32/` — ESP32 low-level firmware
- `ros2_ws/` — ROS 2 Jazzy workspace
- `docs/` — architecture and hardware documentation
- `Notes.txt` — development commands and bring-up notes

## Future Direction

The long-term goal is to evolve this prototype into a modular autonomous vehicle platform suitable for outdoor navigation experiments and, eventually, a Shell Eco-marathon prototype concept.

## Author

Ahmed Benrhouma