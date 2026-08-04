# RC Car Vision Pipeline

Camera-based perception stack for an autonomous RC car prototype, built ahead of an autonomous entry to the **Shell Eco-marathon**. This is the proof-of-concept phase — if the approach works on the RC car, the stack moves to better hardware.

> Part of a larger autonomous car project (waypoint following → obstacle avoidance → lane following → sign recognition). This repo covers the **vision** piece only: lane keeping, stop line / finish line detection, and stop sign detection.

## Status

| Module | Status |
|---|---|
| OAK-D Lite bring-up | 🚧 In progress |
| Lane keeping (classical CV) | 🚧 In progress |
| Stop line / finish line detection | 🚧 In progress |
| Stop sign detection | ✅ Tested |
| Obstacle (pin) detection | 🚧 In progress |
| Full pipeline integration | ⬜ Not started |

## Hardware

- **OAK-D Lite** — primary camera. Has an onboard Myriad X VPU, so inference (once/if a trained model is needed) runs on-camera rather than on the host CPU — important since the final target board is a Pi 4 with no GPU.
- RGB 4K camera — backup/secondary, not the main path.

## Dev Environment

Developed on **Windows** first for faster iteration (more CPU/GPU headroom for training/testing), then ported unchanged to a **Raspberry Pi** later — same Python + `depthai` + `OpenCV` code runs on both.

- Camera + LiDAR processing runs on the Pi (Pi 5 for now, Pi 4 is the eventual target board).
- Motor control and sensor fusion (GPS, IMU) are handled separately by an ESP32 — outside the scope of this repo.

```bash
pip install depthai opencv-python numpy ultralytics
```
```bash
pip install ultralytics
pip install depthai opencv-python ultralytics
```
then isolate in virtual envirenment (.venv): 

```bash
cd ~/Bureau/RC/sign\ detection (change with ur folder path)
python3 -m venv venv
source venv/bin/activate
pip install ultralytics opencv-python depthai
```
## Pipeline Overview

### 1. Lane Keeping (classical OpenCV, no training)

1. Grab RGB frame from the OAK-D Lite
2. ROI mask — trapezoid crop to keep only the road area
3. HSV color threshold and/or Canny edge detection on the ROI
4. Perspective warp → bird's-eye view (`cv2.getPerspectiveTransform` + `warpPerspective`)
5. `cv2.HoughLinesP` → detect line segments → split left/right by slope sign
6. `np.polyfit` (degree 1 or 2) per side → fitted lane lines
7. Lane center = midpoint of left/right lines at the bottom of the frame
8. Offset = `lane_center_x − frame_center_x`
9. Steering command: `steering_angle = Kp * offset` (Kd term from offset history planned as a later refinement)

**Test order:** static images → recorded video → live OAK-D feed.

**Known failure modes:** shadows/lighting causing false edges; dashed lines causing jumpy fits (mitigated with a moving average of fit coefficients); sharp curves needing a degree-2 polyfit instead of degree-1.

### 2. Stop Sign + Obstacle Detection

Per Shell Eco-marathon 2026 rules, only 2 classes are needed — both high-contrast, fixed-color, fixed-shape:

- **Stop sign** — red octagon on a yellow board, elevated 0.5–1m
- **Obstacle** — blue inflatable pin, 1.10m tall, 0.45m diameter

Two implementation options, cheapest first:

- **Option A (no training):** HSV color threshold + contour shape check (polygon vertex count for the octagon, aspect ratio/circularity for the pin). ✅ This is what's implemented and tested for the stop sign.
- **Option B (fallback):** YOLOv8-nano / YOLOv5-nano, 2 classes, trained on Windows, exported to ONNX → `.blob` (via Luxonis' blob converter) for on-camera inference on the OAK-D's VPU. Only needed if Option A proves unreliable across lighting/distance.

**Behavior once detected:**
- Stop sign within threshold distance → decelerate, stop before the stop line, hold, resume
- Obstacle detected → trigger avoidance routing (pairs with LiDAR later; the camera confirms/classifies what LiDAR flags)

### 3. Stop Line / Finish Line Detection

Reuses the lane-keeping color-threshold logic, retuned:

- Stop line: 0.15m orange line, full track width
- Finish line: 0.15m yellow line, full track width

Detected as a horizontal color band near the bottom of the frame, rather than two side lines — simpler than lane detection since it's just "does this color band exist, how far away is it."

## Build Order

1. Environment setup (Windows)
2. OAK-D Lite bring-up — confirm RGB stream with a minimal `depthai` script
3. Lane keeping — get it fully working on recorded/live footage
4. Stop line / finish line detection — same approach, different colors
5. **Stop sign + obstacle detection — Option A tested ✅**
6. Integration test — run all 3 modules on one feed, confirm no cross-detection conflicts (e.g. stop sign red vs. stop line orange)
7. Port to Pi — same code, same `.blob` if the YOLO route was taken; verify FPS on the Pi 4

## How This Fits the Bigger Picture

This vision stack is decoupled from the GPS/ESP32/Nav2 navigation core, so it can be developed in parallel (or handed to a teammate). Eventually its outputs — `steering_angle` from lane keeping, and stop/go decisions from sign detection — need to merge with or override the GPS-waypoint-following steering commands. That fusion logic is a later integration step and isn't designed yet.

## Roadmap

- [ ] Finish lane keeping on live feed
- [ ] Finish stop/finish line detection
- [ ] Validate stop sign detection across more lighting/distance conditions
- [ ] Add obstacle (pin) detection
- [ ] Full pipeline integration test
- [ ] Port to Pi 4, verify FPS
- [ ] Fuse with GPS-waypoint steering once nav core is ready

## Future Work — Rest of the Autonomous RC Car
 
This repo is only the camera/vision slice of a larger autonomous car project. The rest of the system, roughly in build order:
 
1. **Waypoint following** — GPS (NEO-6M/V2) + IMU (BNO055) based navigation, handled by an ESP32 alongside motor control. First milestone for the overall car: drive a set of GPS waypoints autonomously.
2. **Obstacle avoidance** — RPLidar on the Pi, fused with the camera's obstacle (pin) detection from this repo to trigger avoidance maneuvers.
3. **Lane following** — integrate this repo's lane-keeping module so it can override/blend with waypoint-following steering when lane markings are present.
4. **Sign recognition** — integrate this repo's stop sign detection into the full driving loop (decelerate/stop/resume behavior tied to actual motor control, not just detection).
5. **Sensor fusion / system integration** — ESP32 (motors, GPS, IMU) ↔ Pi (LiDAR, camera) communication, merging/prioritizing steering commands from GPS-waypoint-following, obstacle avoidance, and lane keeping into one control loop.
6. **Encoder** — not yet on the car; needs a mounting position before it can be added for better odometry.
7. **Hardware migration** — once the RC car proof-of-concept validates the approach, port the stack to the final competition hardware/platform.


## License

All rights reserved.
<<<<<<< HEAD

=======
>>>>>>> 2ef83efe92a8011feefe7eadc0f2511e46d38438
