dependencies:

""pip install ultralytics
pip install depthai opencv-python ultralytics""

then isolate in virtual envirenment (.venv): 

""cd ~/Bureau/RC/sign\ detection (change with ur folder path)
python3 -m venv venv
source venv/bin/activate
pip install ultralytics opencv-python depthai""

here's the plan:

Project: RC Car Vision Pipeline (Shell Eco-marathon prep)

Hardware: OAK-D Lite (primary — has onboard Myriad X VPU for on-camera inference, critical since the eventual target is a Pi 4 with no GPU). RGB 4K camera as backup/secondary, not the main path.
Dev environment: Windows first (faster iteration, more CPU/GPU headroom for training), then port unchanged to Pi 4 later — same Python + depthai/OpenCV code runs on both.

Task breakdown (3 sub-systems, roughly independent)
1. Lane Keeping (classical OpenCV, no training needed)

Pipeline stages (build and visualize each stage separately, don't jump to final output):

Grab RGB frame from OAK-D Lite
ROI mask — trapezoid crop, keep only road area
Color threshold (HSV) and/or Canny edge detection on the ROI — combine both if needed
Perspective warp → bird's-eye view (cv2.getPerspectiveTransform + warpPerspective)
cv2.HoughLinesP → detect line segments → split into left/right by slope sign
np.polyfit (degree 1 or 2) each side → fitted lane lines
Compute lane center = midpoint of left/right lines at bottom of frame
Offset = lane_center_x − frame_center_x
Steering command: steering_angle = Kp * offset (add Kd term later using offset history if needed)

Test order: static images → recorded video → live OAK-D feed.
Known failure modes to expect: shadows/lighting causing false edges, dashed lines causing jumpy fits (smooth with moving average of fit coefficients), sharp curves needing degree-2 polyfit instead of degree-1.

2. Stop Sign + Obstacle Detection (2-class, small YOLO model)

Per Shell Eco-marathon 2026 rules, you only need 2 classes:

Stop sign: red octagon on yellow board, elevated 0.5–1m
Obstacle: blue inflatable pin, 1.10m tall, 0.45m diameter

Both are high-contrast, fixed-color, fixed-shape — easier than generic traffic signs.

Two implementation options, cheapest first:

Option A (fast, no training): HSV color threshold + contour shape check (approximate polygon vertex count for octagon, or aspect ratio/circularity for the pin). Try this first — may be sufficient given how distinct these colors/shapes are.
Option B (if A isn't robust enough): YOLOv8-nano or YOLOv5-nano, 2 classes. Collect/photograph or render ~50-100 images per class from multiple angles/distances/lighting. Train on Windows PC (GPU), export to ONNX, convert to .blob via Luxonis' blob converter for the OAK-D's VPU, so inference runs on-camera not on Pi CPU.

Behavior once detected:

Stop sign within threshold distance → decelerate, stop before stop line (see task 3), hold, resume
Obstacle detected → trigger avoidance routing (pairs with LiDAR later, camera just confirms/classifies what LiDAR flags)
3. Stop Line / Finish Line Detection

Reuse the lane-keeping color-threshold logic, just retuned:

Stop line: 0.15m orange line, full track width
Finish line: 0.15m yellow line, full track width
Detect as a horizontal band crossing the frame rather than two side lines — simpler than lane detection since you're just looking for "does this color band exist near the bottom of frame, how far away is it"
Build order (do this when you get home)
Environment setup: pip install depthai opencv-python numpy ultralytics on Windows
OAK-D Lite bring-up: confirm RGB stream grabs correctly with a minimal depthai script before touching any CV logic
Lane keeping first — get task 1 fully working on recorded/live footage (tape lines on your floor if no track access yet)
Stop/finish line detection — same color-threshold approach, different colors, much quicker since it reuses task 1's groundwork
Stop sign + obstacle detection — try Option A (color/contour) first; only move to YOLO training if A proves unreliable across lighting/distance variation
Integration test: run all 3 simultaneously on one video feed, confirm no conflicts (e.g., stop sign's red doesn't get misdetected as a stop line, etc.)
Later, on the Pi: same code, same .blob file if you went the YOLO route — swap which machine the OAK-D is plugged into, verify FPS is acceptable on Pi 4 (should be, since inference runs on the camera's VPU, not the Pi CPU)
How this fits the bigger picture

This camera workstream stays decoupled from the GPS/ESP32/Nav2 core work — good candidate to hand to a teammate, or just run in parallel yourself. Eventually the output (steering_angle from lane keeping, stop/go decisions from sign detection) needs to merge with or override the GPS-waypoint-following steering commands — but that fusion logic is a later integration step, not something to design now.

