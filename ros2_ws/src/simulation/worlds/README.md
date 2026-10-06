# Silesia Ring Gazebo World

This is a first-pass Gazebo Sim world generated from the supplied Silesia Ring
track image.

## Real-world reference

- Main circuit length: **3636 m**
- Turns: **15 (9 right, 6 left)**
- Track width: **12–15 m**
- Start/finish straight: **560 m**
- Longest straight: **730 m**

The official Silesia Ring information gives the above dimensions.

## Important accuracy note

The supplied image is a schematic map, not a surveyed CAD/GIS drawing. Therefore,
the world is **layout-accurate but not survey-accurate**. The image was scaled so
the extracted circuit centerline is approximately 3636 m long. The rendered map
should be treated as a simulation reference, not centimeter-accurate geography.

The track image is used as the visual ground map. Low boundary walls are generated
from the detected asphalt edges so LiDAR can see physical boundaries.

## Files

- `silesia_ring.sdf` — Gazebo world
- `models/silesia_ring/model.sdf` — track model
- `models/silesia_ring/meshes/silesia_ring.obj` — textured ground/map
- `models/silesia_ring/meshes/track_boundaries.obj` — generated low boundaries
- `models/silesia_ring/materials/textures/silesia_ring.png` — supplied image

## Run

From the directory containing this package:

```bash
export GZ_SIM_RESOURCE_PATH=$PWD/models:$GZ_SIM_RESOURCE_PATH
gz sim silesia_ring.sdf
```

For your Autonomous-outdoor-RC project, the next step should be to replace the
schematic scaling with the real Silesia Ring GIS/OSM centerline and then add
terrain/elevation, barriers, GPS coordinates, and the vehicle spawn point.
