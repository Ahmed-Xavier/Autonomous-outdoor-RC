# System Architecture

## Overview

The vehicle uses a distributed architecture with a clear separation between high-level autonomy and low-level hardware control.

- **Raspberry Pi 4:** High-level autonomy, perception, localization, and planning.
- **ESP32:** Low-level actuator control, sensor acquisition, and safety handling.

This separation keeps timing-sensitive hardware control independent from the higher-level ROS 2 autonomy stack.

---

## Raspberry Pi Responsibilities

The Raspberry Pi runs Ubuntu 24.04 with ROS 2 Jazzy.

Responsibilities:

- ROS 2 middleware and application nodes
- LiDAR driver
- Hikvision camera driver and image processing
- GT-U7 GPS/GNSS integration
- Lane detection
- Stop sign detection
- Localization and sensor fusion
- GPS waypoint navigation
- Path planning
- Obstacle avoidance
- High-level speed and steering commands
- Visualization and monitoring through Foxglove

---

## ESP32 Responsibilities

The ESP32 handles low-level and timing-sensitive functions:

- JGA25-370 motor control
- L298N motor driver control
- Encoder acquisition
- Steering servo control
- BNO055 IMU acquisition
- Low-level safety behavior
- Motor command timeout/failsafe
- Sensor publishing to ROS 2 through micro-ROS

---

## ROS 2 Project Structure & Package Anatomy

### Root Layout & Configuration Files
- **`README.md`**: Outlines what the project is, along with quick-start steps to build and run.
- **`.gitignore`**: Excludes build files (`build/`, `install/`, `log/`, `__pycache__/`, `*.egg-info`) from version control.
- **`docs/`**: Contains `architecture.md` (layers, topics, TF tree), `hardware.md` (wiring, pins, dimensions), and `development-roadmap.md` (future milestones).
- **`firmware/esp32/microros.ino`**: Runs on the ESP32 outside ROS 2 to drive motors, servos, and encoders, read the BNO055, and communicate with the Pi via micro-ROS (its README contains flash steps, library requirements, and agent IP configurations).

### Core Package Anatomy
Files that repeat across individual packages serve specific roles:
- **`package.xml`**: The package ID card detailing its name, version, and dependencies, which ROS reads to determine build order.
- **`CMakeLists.txt`**: Used in C++ packages to instruct the build system to copy `config/`, `launch/`, and `urdf/` folders to the install directory. Unlisted folders are invisible to ROS after building.
- **`setup.py`**: Used in Python packages to manage installations and entry points (e.g., turning Python scripts into commands like `ros2 run navigation waypoint_manager`).
- **`setup.cfg`**: Tells the build system where to place executable binaries.
- **`resource/<package_name>`**: An empty marker file required for ROS package discovery.
- **`__init__.py`**: Marks subfolders as valid Python code packages.
- **`config/*.yaml`**: Stores tunable parameters (one configuration file per node).
- **`launch/*.launch.py`**: Initiates nodes and loads their respective YAML configuration files.

---

## ROS 2 Software Layers

### Layer 0: Definitions (`vehicle_description`)
Defines the physical body and structure of the car.
- **`urdf/rc_car.urdf.xacro`**: Specifies mechanical parts (chassis, wheels, `imu_link`, `gps_link`) and their relative placements. The `imu_link` joint sets the mounting rotation of the IMU, establishing the vehicle's forward orientation for ROS.
- **`launch/description.launch.py`**: Starts `robot_state_publisher`, converting the URDF into TF transforms visualized in Foxglove.

### Layer 1: Sensors (`vehicle_drivers`)
Manages sensor inputs, drivers, and communication bridges (contains no custom logic code).
- **`config/gps.yaml`**: Configures serial ports, baud rates, frame names, and covariance scales to calibrate GPS reliability for filters (fixing cases where the EKF under-trusts the GPS).
- **`launch/gps.launch.py`**: Initializes the GPS driver to publish `/fix`.
- **`launch/microros_agent.launch.py`**: Starts the micro-ROS agent bridging ESP32 topics (such as `/esp32/encoder` and IMU data) to the Raspberry Pi.

### Layer 2: Estimation (`localization`)
Performs state estimation, tracking vehicle position and orientation.
- **`ekf_local.yaml`**: Configures a local filter in the `odom` frame combining encoder speed and IMU rotation to produce smooth, non-jumping dead-reckoning output (`/odometry/local`).
- **`ekf_global.yaml`**: Configures a global filter in the `map` frame incorporating GPS for absolute world positioning (`/odometry/global`).
- **`navsat.yaml`**: Settings for `navsat_transform` to convert latitude/longitude into map meters, incorporating heading settings like `yaw_offset` and magnetic declination. Ideal for housing hand-measured offsets to clean up scripts.
- **`launch/localization.launch.py`**: Launches both EKF filters alongside `navsat_transform`.

### Layer 3: Control (`vehicle_control`)
Translates high-level commands into low-level actuator signals.
- **`vehicle_controller_node.py`**: Reads `/cmd_vel` and publishes `/esp32/motor_cmd` and `/esp32/servo_cmd` using parameters from `vehicle_controller.yaml` (servo center, steering gain, sign configurations, max speed).
- **`vehicle_state_node.py`**: Reads servo commands and encoders to publish `/joint_states` for URDF wheel and steering synchronization, using parameters from `vehicle_state.yaml` (ticks per meter, encoder/steering signs, wheelbase). *Note: In the target design, this node stops publishing the `odom` $\rightarrow$ `base_footprint` TF since the EKF owns it, keeping wheel odometry purely as an input for the EKF.*
- **`launch/vehicle_control.launch.py`**: Starts both control nodes with their corresponding YAML configurations.

### Layer 4: Behavior (`navigation`)
Handles path planning and directional steering.
- **`waypoint_manager_node.py`**: Collects multiple waypoints from GPS fixes, map clicks, and `/waypoints` services, then follows the route using refreshed map coordinates.
- **`waypoint_manager.yaml`**: Holds the waypoint manager's tunable values.
- **`launch/navigation.launch.py`**: Starts waypoint management, the local planner, the command gate, and the safety stop.

### Layer 5: System Bringup (`vehicle_bringup`)
- **`bringup.launch.py`**: Integrates all system layers in order: description, drivers, localization, control, and navigation.
- **`indoor.launch.py`**: Runs a GPS-free, local-filter-only setup for bench testing and debugging sensor frames inside Foxglove (rotate the car by hand and watch the URDF move).

---

## Communication

The vehicle currently uses **Wi-Fi transport + micro-ROS + ROS 2**.

The ESP32 connects to the Raspberry Pi over Wi-Fi and communicates with a micro-ROS agent running on the Raspberry Pi. The micro-ROS agent uses UDP port `8888`.

---

## Data Flow

### Overall Pipeline
```text
ESP32 / GPS  ->  ekf_local / ekf_global  ->  waypoint_manager  ->  local_planner / cmd_vel_gate / safety_stop  ->  vehicle_controller  ->  ESP32
 (layer 1)           (layer 2)                  (layer 4)              (layer 3)
```

### ESP32 → Raspberry Pi
- **Current:** `/esp32/encoder`
- **Planned:** Wheel odometry, BNO055 IMU data, motor feedback, steering feedback

### Raspberry Pi → ESP32
- **Current:** `/esp32/motor_cmd`, `/esp32/servo_cmd`
- **Planned Higher-Level Interface:** Target motor speed, steering angle, emergency stop, vehicle control commands

---

## Autonomy Pipeline

1. Acquire sensor data from the ESP32 and Raspberry Pi sensors.
2. Estimate vehicle position and motion.
3. Detect lanes and traffic signs from the camera.
4. Detect obstacles using LiDAR.
5. Fuse GPS, IMU, and wheel information for localization.
6. Plan a safe trajectory toward GPS waypoints.
7. Generate speed and steering commands.
8. Send commands to the ESP32.
9. Execute them through the motor driver and steering servo.
10. Apply low-level safety/failsafe behavior independently of the autonomy stack.

---

## Safety

The ESP32 must remain capable of stopping the propulsion motor without depending on a high-level autonomy node.

- Includes a motor command timeout: if motor commands stop arriving for the configured timeout period, the ESP32 halts the motor.
- If the micro-ROS agent connection drops, the ESP32 automatically stops the motor before resetting to a waiting state.

---

## Visualization

- **Foxglove:** Used during development for real-time ROS 2 visualization and debugging.
- **Google Earth:** A map/screenshot has been tested as an auxiliary visualization source in Foxglove while developing the navigation stack.

---

## Design Principles

- Modular ROS 2 nodes.
- Separation of high-level and low-level control.
- Hardware abstraction.
- Explicit interfaces between layers.
- Testability.
- Safe fallback behavior.
- Hardware-independent autonomy logic where practical.
