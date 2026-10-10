# Simulation worlds

Supports two selectable Gazebo Sim worlds.

## Full Silesia Ring
Run `ros2 launch simulation simulation.launch.py world:=silesia_ring`.

## Autonomous challenge test course
Run `ros2 launch simulation simulation.launch.py world:=autonomous_challenge`.

Loads a lightweight 60 x 30 m rectangular loop with 6 m roadway, yellow start/finish line, orange stop line, stop-sign target and static obstacle replicas. Vehicle spawn: (-5, -15), heading +X.

**This is an engineering test fixture, not an official Shell Eco-marathon course map.** Replace geometry when the organisers publish the applicable course.

## Foxglove
Connect to `ws://localhost:8765`, add a 3D panel, set fixed frame to `map`, and add `/world_markers`, `/tf`, `/odom` and `/scan`. Gazebo remains server-only with headless rendering.