# Simulation worlds

Supports two selectable Gazebo Sim worlds.

## Silesia Ring (scaled approximation)

Run:

```bash
ros2 launch simulation simulation.launch.py world:=silesia_ring
```

The placeholder square mesh has been replaced with a closed road-ribbon mesh. Its centreline is scaled to **3,636 m** and the nominal road width is **12 m**, matching the published overall circuit length and minimum width. The start straight is 560 m. Source for the reference dimensions: [official Silesia Ring circuit information](https://silesiaring.pl/tor-3/).

**Important limitation:** this is a hand-built *approximation*, not a surveyed or geographically traced reproduction of the real circuit. The total centreline length and nominal width are scaled, but corner positions, exact curvature, track boundaries, elevation, run-off, pit lane, and detailed geometry are not verified against official CAD/GIS data. Do not use it as a precision map for GPS/waypoint validation. The track ribbon has matching static collision geometry; the surrounding terrain is a separate flat surface.

## Autonomous challenge test course

Run:

```bash
ros2 launch simulation simulation.launch.py world:=autonomous_challenge
```

Loads a lightweight 60 x 30 m rectangular loop with 6 m roadway, yellow start/finish line, orange stop line, stop-sign target and static obstacle replicas. Vehicle spawn: (-15, -15), heading +X, behind the start/finish line.

**This is an engineering test fixture, not an official Shell Eco-marathon course map.** Replace geometry when the organisers publish the applicable course.

## Foxglove

Connect to `ws://localhost:8765`, add a 3D panel, set fixed frame to `map`, and add `/world_markers`, `/tf`, `/odom` and `/scan`. Gazebo remains server-only with headless rendering.
