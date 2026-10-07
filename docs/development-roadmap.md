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
**Status:** DONE

- BNO055 IMU wired, tested, publishing at ~50 Hz
- GT-U7 GPS integration
- LiDAR integration
- Complete ESP32 sensor bridge

## Phase 4 — Localization
**Status:** DONE

- Wheel odometry
- IMU processing
- GPS position
- EKF sensor fusion (local + global)
- Vehicle TF tree
- navsat_transform (lat/lon → map frame)
- Outdoor localization tested

## Phase 5 — Simulation
**Status:** IN PROGRESS

Replicating the full real-car stack in Gazebo (headless, WSL) with Foxglove.

- ✅ Real car URDF running in Gazebo headless
- ✅ Teleop control via `/cmd_vel` (steering + speed)
- 🔄 Sensor simulation: GPS, IMU, LiDAR publishing in Gazebo
- 🔄 Navigation stack validated in simulation
- ⬜ Camera simulation
- ⬜ `/cmd_vel` arbiter (selects between teleop and autonomy)

## Phase 6 — Autonomous Navigation
**Status:** DONE (real car)

- GPS waypoint management
- Path planning (local planner)
- Autonomous speed control
- Autonomous steering
- Safety stop (LiDAR-based)
- Full autonomy tested on real car

## Phase 7 — Perception
**Status:** PLANNED

- Lane detection
- Lane keeping
- Stop sign detection
- LiDAR obstacle avoidance

## Phase 8 — Command Arbitration
**Status:** PLANNED

- `/cmd_vel` arbiter node (teleop / autonomy / e-stop priority)
- Emergency stop
- Communication-loss handling
- Sensor failure handling
- Safe-stop behavior

## Phase 9 — Outdoor Testing
**Status:** PLANNED

- Full simulation validation before deployment
- Sensor validation on real car
- GPS and localization tests
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