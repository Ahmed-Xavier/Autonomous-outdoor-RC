# ENIMIA simulation

Gazebo-only assets and launch configuration live here.

The vehicle model itself is **not** duplicated here. It is provided by
`vehicle_description`, which is the canonical ENIMIA URDF package.

## Launch

```bash
ros2 launch simulation simulation.launch.py
```

This package provides the Gazebo startup/spawn skeleton. Simulated IMU, GPS,
and LiDAR plugins/bridges should be added here rather than to the canonical
vehicle description.
