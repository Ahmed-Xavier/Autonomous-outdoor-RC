# \# System Architecture

# 

# \## Overview

# 

# The vehicle uses a distributed architecture with a clear separation between high-level autonomy and low-level hardware control.

# 

# \- \*\*Raspberry Pi 4:\*\* high-level autonomy, perception, localization and planning.

# \- \*\*ESP32:\*\* low-level actuator control, sensor acquisition and safety handling.

# 

# This separation keeps timing-sensitive hardware control independent from the higher-level ROS 2 autonomy stack.

# 

# \## Raspberry Pi Responsibilities

# 

# The Raspberry Pi runs Ubuntu 24.04 with ROS 2 Jazzy.

# 

# Responsibilities:

# 

# \- ROS 2 middleware and application nodes

# \- LiDAR driver

# \- Hikvision camera driver and image processing

# \- GT-U7 GPS/GNSS integration

# \- Lane detection

# \- Stop sign detection

# \- Localization and sensor fusion

# \- GPS waypoint navigation

# \- Path planning

# \- Obstacle avoidance

# \- High-level speed and steering commands

# \- Visualization and monitoring through Foxglove

# 

# \## ESP32 Responsibilities

# 

# The ESP32 handles low-level and timing-sensitive functions:

# 

# \- JGA25-370 motor control

# \- L298N motor driver control

# \- Encoder acquisition

# \- Steering servo control

# \- BNO055 IMU acquisition

# \- Low-level safety behavior

# \- Motor command timeout/failsafe

# \- Sensor publishing to ROS 2 through micro-ROS

# 

# \## Communication

# 

# The vehicle currently uses:

# 

# \*\*Wi-Fi transport + micro-ROS + ROS 2\*\*

# 

# The ESP32 connects to the Raspberry Pi over Wi-Fi and communicates with a micro-ROS agent running on the Raspberry Pi.

# 

# The micro-ROS agent currently uses UDP port `8888`.

# 

# The current implementation therefore uses \*\*micro-ROS over Wi-Fi\*\*.

# 

# \## Current ROS 2 Data Flow

# 

# \### ESP32 → Raspberry Pi

# 

# Current:

# 

# \- `/esp32/encoder` — encoder count

# 

# Planned:

# 

# \- wheel odometry

# \- BNO055 IMU data

# \- motor feedback

# \- steering feedback, if available

# 

# \### Raspberry Pi → ESP32

# 

# Current:

# 

# \- `/esp32/motor\_cmd`

# \- `/esp32/servo\_cmd`

# 

# Planned higher-level vehicle interface:

# 

# \- target motor speed

# \- steering angle

# \- emergency stop

# \- vehicle control commands

# 

# \## Autonomy Pipeline

# 

# 1\. Acquire sensor data from the ESP32 and Raspberry Pi sensors.

# 2\. Estimate vehicle position and motion.

# 3\. Detect lanes and traffic signs from the camera.

# 4\. Detect obstacles using LiDAR.

# 5\. Fuse GPS, IMU and wheel information for localization.

# 6\. Plan a safe trajectory toward GPS waypoints.

# 7\. Generate speed and steering commands.

# 8\. Send commands to the ESP32.

# 9\. Execute them through the motor driver and steering servo.

# 10\. Apply low-level safety/failsafe behavior independently of the autonomy stack.

# 

# \## Safety

# 

# The ESP32 must remain capable of stopping the propulsion motor without depending on a high-level autonomy node.

# 

# The current firmware includes a motor command timeout. If motor commands stop arriving for the configured timeout period, the ESP32 stops the motor.

# 

# If the micro-ROS agent connection is lost, the ESP32 also stops the motor before returning to its waiting state.

# 

# \## Visualization

# 

# Foxglove is used during development for ROS 2 visualization and debugging.

# 

# A map/screenshot from Google Earth has also been tested as a visualization source in Foxglove while the navigation stack is being developed.

# 

# \## Design Principles

# 

# \- Modular ROS 2 nodes.

# \- Separation of high-level and low-level control.

# \- Hardware abstraction.

# \- Explicit interfaces between layers.

# \- Testability.

# \- Safe fallback behavior.

# \- Hardware-independent autonomy logic where practical.

