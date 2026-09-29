# Development Roadmap

## Phase 1 — Low-Level Control
**Status:** DONE

- ESP32 motor control
- L298N integration
- JGA25-370 encoder
- Encoder direction detection
- Encoder publishing at ~20 Hz
- Steering servo control
- Motor command timeout / failsafe
- Wi-Fi micro-ROS communication

## Phase 2 — ROS 2 Vehicle Control
**Status:** DONE / IN PROGRESS

- `/cmd_vel` vehicle controller
- ROS 2 → ESP32 motor commands
- ROS 2 → ESP32 steering commands
- Foxglove teleoperation
- Command multiplexer
- Emergency-stop priority
- Standard vehicle command interface

## Phase 3 — Sensor Integration
**Status:** IN PROGRESS

- BNO055 wired and tested
- IMU publishing at ~50 Hz
- GT-U7 GPS integration
- LiDAR integration
- Hikvision camera integration
- Complete ESP32 sensor bridge

## Phase 4 — Localization
**Status:** PLANNED

- Wheel odometry
- IMU processing
- GPS position
- Sensor fusion
- Vehicle TF tree
- Outdoor localization testing

## Phase 5 — Perception
**Status:** PLANNED

- Lane detection
- Lane keeping
- Stop sign detection
- LiDAR obstacle detection
- Obstacle avoidance

## Phase 6 — Autonomous Navigation
**Status:** PLANNED

- GPS waypoint management
- Path planning
- Autonomous speed control
- Autonomous steering
- Navigation behavior manager
- Full autonomy test

## Phase 7 — Safety & Integration
**Status:** PLANNED

- Command arbitration
- Emergency stop
- Communication-loss handling
- Sensor failure handling
- Safe-stop behavior
- Full-system bringup

## Phase 8 — Outdoor Testing
**Status:** PLANNED

- Manual driving tests
- Sensor validation
- GPS tests
- Localization tests
- Perception tests
- Autonomous driving tests
- Long-duration testing

## Phase 9 — Future Platform
**Status:** FUTURE

- Improved vehicle hardware
- More robust localization
- Improved autonomy
- Energy-efficient vehicle design
- Shell Eco-marathon prototype direction