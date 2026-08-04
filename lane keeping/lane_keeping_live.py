"""
Lane Keeping - live OAK-D Lite feed with HUD overlay
Shell Eco-marathon autonomous RC car - vision pipeline

Pipeline: ROI mask -> HSV/Canny threshold -> bird's-eye warp -> HoughLinesP
          -> split left/right -> polyfit -> lane center -> offset -> steering_angle

Run:      python lane_keeping_live.py
Controls: 'q' quit, 'd' toggle debug windows (threshold mask + warped binary)
"""

import cv2
import numpy as np
import depthai as dai

# ---------------- Config ----------------
FRAME_W, FRAME_H = 640, 400              # OAK-D preview size

ROI_TOP_Y_FRAC = 0.55                    # trapezoid top, as fraction of frame height
ROI_TOP_HALF_W_FRAC = 0.20               # trapezoid top half-width, fraction of frame width
ROI_BOT_HALF_W_FRAC = 0.48               # trapezoid bottom half-width

WARP_W, WARP_H = 400, 500                # bird's-eye output size

HSV_LOWER_WHITE = np.array([0, 0, 180])
HSV_UPPER_WHITE = np.array([180, 60, 255])
HSV_LOWER_YELLOW = np.array([15, 80, 120])
HSV_UPPER_YELLOW = np.array([35, 255, 255])

CANNY_LOW, CANNY_HIGH = 50, 150

HOUGH_THRESH = 20
HOUGH_MIN_LEN = 20
HOUGH_MAX_GAP = 60
MIN_SLOPE = 0.3                          # discard near-horizontal segments

FIT_DEGREE = 1                           # bump to 2 for sharp curves later
SMOOTH_ALPHA = 0.3                       # exponential smoothing on fit coeffs
ASSUMED_LANE_WIDTH_PX = 250              # warped-space fallback if only one side found

STRAIGHT_DEADZONE = 20                   # warped-px offset within this -> "Straight"
Kp = 0.01                                # placeholder steering gain, tune later

# ---------------- Geometry setup ----------------
def build_roi_points():
    top_y = FRAME_H * ROI_TOP_Y_FRAC
    bottom_y = FRAME_H
    top_half_w = FRAME_W * ROI_TOP_HALF_W_FRAC
    bottom_half_w = FRAME_W * ROI_BOT_HALF_W_FRAC
    cx = FRAME_W / 2
    return np.array([
        [cx - bottom_half_w, bottom_y],
        [cx - top_half_w, top_y],
        [cx + top_half_w, top_y],
        [cx + bottom_half_w, bottom_y],
    ], dtype=np.float32)


ROI_PTS = build_roi_points()
DST_PTS = np.array([[0, WARP_H], [0, 0], [WARP_W, 0], [WARP_W, WARP_H]], dtype=np.float32)
M = cv2.getPerspectiveTransform(ROI_PTS, DST_PTS)
M_INV = cv2.getPerspectiveTransform(DST_PTS, ROI_PTS)
ROI_BOUNDING_RECT = cv2.boundingRect(ROI_PTS.astype(np.int32))  # for the HUD box


# ---------------- Pipeline stages ----------------
def threshold_frame(frame):
    """ROI mask + HSV color threshold + adaptive threshold, combined."""
    roi_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    cv2.fillPoly(roi_mask, [ROI_PTS.astype(np.int32)], 255)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, HSV_LOWER_WHITE, HSV_UPPER_WHITE)
    yellow = cv2.inRange(hsv, HSV_LOWER_YELLOW, HSV_UPPER_YELLOW)
    color_mask = cv2.bitwise_or(white, yellow)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    adaptive = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        blockSize=25,
        C=-10
    )

    combined = cv2.bitwise_or(color_mask, adaptive)
    combined = cv2.bitwise_and(combined, combined, mask=roi_mask)
    return combined, color_mask, adaptive


def find_lane_pixels_sliding_window(warped_binary, n_windows=9, margin=50, minpix=30):
    """Sliding window search: returns (left_x, left_y, right_x, right_y) pixel arrays."""
    histogram = np.sum(warped_binary[warped_binary.shape[0]//2:, :], axis=0)
    midpoint = WARP_W // 2
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    window_height = WARP_H // n_windows
    nonzero = warped_binary.nonzero()
    nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])

    leftx_current, rightx_current = leftx_base, rightx_base
    left_lane_inds, right_lane_inds = [], []

    for window in range(n_windows):
        y_low = WARP_H - (window + 1) * window_height
        y_high = WARP_H - window * window_height

        xleft_low, xleft_high = leftx_current - margin, leftx_current + margin
        xright_low, xright_high = rightx_current - margin, rightx_current + margin

        good_left = ((nonzeroy >= y_low) & (nonzeroy < y_high) &
                     (nonzerox >= xleft_low) & (nonzerox < xleft_high)).nonzero()[0]
        good_right = ((nonzeroy >= y_low) & (nonzeroy < y_high) &
                      (nonzerox >= xright_low) & (nonzerox < xright_high)).nonzero()[0]

        left_lane_inds.append(good_left)
        right_lane_inds.append(good_right)

        if len(good_left) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left]))
        if len(good_right) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right]))

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    return (nonzerox[left_lane_inds], nonzeroy[left_lane_inds],
            nonzerox[right_lane_inds], nonzeroy[right_lane_inds])


def fit_line(points, degree=FIT_DEGREE):
    if len(points) < 4:
        return None
    pts = np.array(points)
    ys, xs = pts[:, 1], pts[:, 0]
    try:
        return np.polyfit(ys, xs, degree)
    except np.linalg.LinAlgError:
        return None


def smooth_fit(prev, new, alpha=SMOOTH_ALPHA):
    if new is None:
        return prev
    if prev is None or len(prev) != len(new):
        return new
    return alpha * new + (1 - alpha) * prev


def unwarp_point(x, y):
    """Map a single bird's-eye (x, y) point back to original frame coords."""
    pt = np.array([[[x, y]]], dtype=np.float32)
    back = cv2.perspectiveTransform(pt, M_INV)
    return int(back[0, 0, 0]), int(back[0, 0, 1])


# ---------------- HUD drawing ----------------
def draw_hud(frame, left_fit, right_fit, offset, steering_angle):
    hud = frame.copy()
    bx, by, bw, bh = ROI_BOUNDING_RECT
    cv2.rectangle(hud, (bx, by), (bx + bw, by + bh), (255, 255, 0), 2)  # cyan ROI box

    def draw_side_marker(fit, color=(0, 200, 0)):
        if fit is None:
            return None

        y_samples = np.linspace(WARP_H * 0.35, WARP_H - 1, 25)
        x_samples = np.polyval(fit, y_samples)

        pts = []
        for x, y in zip(x_samples, y_samples):
            px, py = unwarp_point(x, y)
            pts.append((px, py))

        pts = np.array(pts, dtype=np.int32)
        cv2.polylines(hud, [pts], isClosed=False, color=color, thickness=6)

        mid_idx = len(pts) // 2
        cx, cy = pts[mid_idx]
        cap_y = cy + 45
        cv2.line(hud, (cx - 60, cap_y), (cx + 60, cap_y), (0, 220, 255), 2)
        cv2.line(hud, (cx - 60, cap_y - 8), (cx - 60, cap_y + 8), (0, 220, 255), 2)
        cv2.line(hud, (cx + 60, cap_y - 8), (cx + 60, cap_y + 8), (0, 220, 255), 2)
        return cx

    draw_side_marker(left_fit)
    draw_side_marker(right_fit)

    veh_cx = bx + bw // 2
    veh_cy = by + int(bh * 0.5)
    cv2.line(hud, (veh_cx, veh_cy - 30), (veh_cx, veh_cy + 30), (255, 255, 255), 3)

    if offset is not None:
        screen_offset = int(np.clip(offset * 0.6, -80, 80))
        target_x = veh_cx + screen_offset
        cv2.line(hud, (target_x, veh_cy - 20), (target_x, veh_cy + 20), (0, 0, 255), 3)
        cv2.line(hud, (veh_cx, veh_cy), (target_x, veh_cy), (255, 0, 0), 2)

        if offset > STRAIGHT_DEADZONE:
            command = "Turn Left"
        elif offset < -STRAIGHT_DEADZONE:
            command = "Turn Right"
        else:
            command = "Straight"
        cv2.putText(hud, command, (bx + 10, by + bh - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2)
        cv2.putText(hud, f"offset={offset:.1f}px  steer={steering_angle:.3f}",
                    (bx + 10, by - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    else:
        cv2.putText(hud, "No lane detected", (bx + 10, by + bh - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

    return hud


# ---------------- Main loop ----------------
def main():
    debug = False
    left_fit_smooth, right_fit_smooth = None, None

    device = dai.Device()
    with dai.Pipeline(device) as pipeline:
        cam = pipeline.create(dai.node.Camera).build()
        cam_out = cam.requestOutput(
            (FRAME_W, FRAME_H), type=dai.ImgFrame.Type.BGR888p, fps=30
        )
        q_rgb = cam_out.createOutputQueue(maxSize=4, blocking=False)

        pipeline.start()
        print("Running. 'q' to quit, 'd' to toggle debug windows.")

        while pipeline.isRunning():
            in_rgb = q_rgb.get()
            frame = in_rgb.getCvFrame()

            if frame is None or frame.size == 0:
                continue  # skip this iteration, camera not ready yet

            combined, color_mask, edges = threshold_frame(frame)
            warped = cv2.warpPerspective(combined, M, (WARP_W, WARP_H))

            left_x_pts, left_y_pts, right_x_pts, right_y_pts = find_lane_pixels_sliding_window(warped)
            left_fit = np.polyfit(left_y_pts, left_x_pts, FIT_DEGREE) if len(left_y_pts) > 50 else None
            right_fit = np.polyfit(right_y_pts, right_x_pts, FIT_DEGREE) if len(right_y_pts) > 50 else None
            left_fit_smooth = smooth_fit(left_fit_smooth, left_fit)
            right_fit_smooth = smooth_fit(right_fit_smooth, right_fit)

            left_x = np.polyval(left_fit_smooth, WARP_H - 1) if left_fit_smooth is not None else None
            right_x = np.polyval(right_fit_smooth, WARP_H - 1) if right_fit_smooth is not None else None

            if left_x is not None and right_x is not None:
                lane_center_x = (left_x + right_x) / 2
            elif left_x is not None:
                lane_center_x = left_x + ASSUMED_LANE_WIDTH_PX / 2
            elif right_x is not None:
                lane_center_x = right_x - ASSUMED_LANE_WIDTH_PX / 2
            else:
                lane_center_x = None

            if lane_center_x is not None:
                offset = lane_center_x - (WARP_W / 2)
                steering_angle = Kp * offset
            else:
                offset, steering_angle = None, 0.0

            hud = draw_hud(frame, left_fit_smooth, right_fit_smooth, offset, steering_angle)
            print(f"frame shape: {frame.shape}, hud shape: {hud.shape if hud is not None else None}")
            cv2.imshow("Lane Keeping", hud)

            if debug:
                warped_vis = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
                warped_vis[left_y_pts, left_x_pts] = [0, 0, 255]
                warped_vis[right_y_pts, right_x_pts] = [0, 165, 255]
                cv2.imshow("Threshold (color+adaptive, ROI masked)", combined)
                cv2.imshow("Bird's-eye + sliding window pixels", warped_vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('d'):
                debug = not debug
                if not debug:
                    cv2.destroyWindow("Threshold (color+adaptive, ROI masked)")
                    cv2.destroyWindow("Bird's-eye + sliding window pixels")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()