"""
Phase 1b: Color tracker with Kalman filter prediction.

WHAT'S NEW vs track_color.py:
- Kalman filter maintains an estimate of position AND velocity
- When the object is detected, the filter "corrects" its estimate
- When detection is lost (occlusion, blur), the filter "predicts" where
  the object should be based on its last known velocity
- You'll see a GREEN crosshair (measurement) and a YELLOW crosshair
  (Kalman prediction) — watch how they diverge when you move fast or
  hide the object

HOW THE KALMAN FILTER WORKS (simplified):
  The filter models the object as having a "state": [x, y, vx, vy]
  (position + velocity). Each frame, two things happen:

  1. PREDICT: Using the current state, estimate where the object will
     be next frame. x_new = x + vx * dt. This propagates forward even
     if we can't see the object.

  2. UPDATE (correct): If we got a detection this frame, compare the
     predicted position to the measured position. The difference is the
     "residual." The filter blends the prediction and measurement based
     on how much it trusts each one (the Kalman gain). Noisy measurements
     → trust the prediction more. Noisy model → trust the measurement more.

  The filter also tracks uncertainty (covariance). When predicting without
  measurements, uncertainty grows. When a measurement arrives, uncertainty
  shrinks. This is how it "knows" how confident it is.

USAGE:
    python track_kalman.py                # uses default HSV (calibrate first!)
    python track_kalman.py --calibrate    # open HSV calibration trackbars

CONTROLS:
    q — quit
    c — toggle calibration mode
    s — save HSV values
    k — toggle Kalman visualization on/off
    r — reset Kalman filter state

TIP: Try these experiments to see the Kalman filter in action:
  1. Move the object smoothly — green and yellow crosshairs should nearly overlap
  2. Move the object FAST — yellow (prediction) will lag, then catch up
  3. Hide the object behind your hand — yellow keeps predicting its path!
  4. Throw an object in an arc — watch the prediction follow the trajectory
"""

import argparse
import time
import cv2
import numpy as np
from filterpy.kalman import KalmanFilter


# ---------------------------------------------------------------------------
# HSV RANGE — paste your calibrated values here!
# ---------------------------------------------------------------------------
DEFAULT_LOWER = np.array([160, 180, 0])
DEFAULT_UPPER = np.array([179, 255, 255])

MIN_CONTOUR_AREA = 500


# ---------------------------------------------------------------------------
# KALMAN FILTER SETUP
# ---------------------------------------------------------------------------
def create_tracker_filter():
    """
    Create a Kalman filter for 2D position + velocity tracking.

    State vector (what we're estimating):
        [x, y, vx, vy]

    Measurement vector (what we observe):
        [x, y]

    This is a "constant velocity" model — it assumes the object moves
    at roughly constant velocity between frames. Acceleration (like
    gravity on a rocket) shows up as process noise.

    LEARNING NOTES:
    ───────────────
    The Kalman filter has several matrices you need to understand:

    F (state transition): How the state evolves from one step to the next.
        x_new = x + vx * dt
        y_new = y + vy * dt
        This is just basic kinematics.

    H (measurement): Maps state → measurement. We can only measure
        position [x, y], not velocity, so H picks out the first two
        elements of the state.

    R (measurement noise): How noisy our camera detections are.
        Higher R → filter trusts predictions more, measurements less.
        Lower R  → filter trusts measurements more, predictions less.

    Q (process noise): How much the object might deviate from our
        constant-velocity model (e.g., due to acceleration, wind, etc).
        Higher Q → filter adapts faster but is jittery.
        Lower Q  → filter is smoother but slower to react to changes.

    P (covariance): The filter's uncertainty about its current estimate.
        Grows during prediction (we're less sure over time).
        Shrinks during update (measurement reduces uncertainty).
    """
    kf = KalmanFilter(dim_x=4, dim_z=2)

    # State transition matrix (constant velocity model)
    # Will be updated each frame with actual dt
    kf.F = np.array([
        [1, 0, 1, 0],   # x  = x + vx*dt  (dt=1 placeholder)
        [0, 1, 0, 1],   # y  = y + vy*dt
        [0, 0, 1, 0],   # vx = vx  (constant velocity)
        [0, 0, 0, 1],   # vy = vy
    ], dtype=float)

    # Measurement matrix — we only observe [x, y], not velocity
    kf.H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
    ], dtype=float)

    # Measurement noise covariance — how noisy are our detections?
    # These values are in pixels². 25 px² ≈ ±5 pixel uncertainty,
    # which is reasonable for color blob detection.
    kf.R = np.array([
        [25, 0],
        [0, 25],
    ], dtype=float)

    # Process noise covariance — how much do we expect the object
    # to deviate from constant velocity? Higher = more responsive
    # but noisier. We use filterpy's helper to generate this from
    # a noise magnitude parameter.
    #
    # For now we set it manually. The diagonal values represent
    # uncertainty in [x, y, vx, vy]. Larger velocity noise means
    # we expect more acceleration/direction changes.
    kf.Q = np.array([
        [4,  0,  0,  0],    # position noise (small)
        [0,  4,  0,  0],
        [0,  0, 16,  0],    # velocity noise (larger — we expect speed changes)
        [0,  0,  0, 16],
    ], dtype=float)

    # Initial covariance — start with high uncertainty
    kf.P *= 1000

    return kf


def update_dt(kf, dt):
    """
    Update the state transition matrix F with the actual time step.

    This is important! If frames arrive at uneven intervals (which they
    will — your webcam isn't perfectly steady), using the real dt makes
    predictions much more accurate than assuming a fixed frame rate.
    """
    kf.F[0, 2] = dt  # x += vx * dt
    kf.F[1, 3] = dt  # y += vy * dt


# ---------------------------------------------------------------------------
# DETECTION (same as track_color.py)
# ---------------------------------------------------------------------------
def nothing(x):
    pass


def create_trackbars(window_name, lower, upper):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("H Low", window_name, int(lower[0]), 179, nothing)
    cv2.createTrackbar("S Low", window_name, int(lower[1]), 255, nothing)
    cv2.createTrackbar("V Low", window_name, int(lower[2]), 255, nothing)
    cv2.createTrackbar("H High", window_name, int(upper[0]), 179, nothing)
    cv2.createTrackbar("S High", window_name, int(upper[1]), 255, nothing)
    cv2.createTrackbar("V High", window_name, int(upper[2]), 255, nothing)


def read_trackbars(window_name):
    h_lo = cv2.getTrackbarPos("H Low", window_name)
    s_lo = cv2.getTrackbarPos("S Low", window_name)
    v_lo = cv2.getTrackbarPos("V Low", window_name)
    h_hi = cv2.getTrackbarPos("H High", window_name)
    s_hi = cv2.getTrackbarPos("S High", window_name)
    v_hi = cv2.getTrackbarPos("V High", window_name)
    return np.array([h_lo, s_lo, v_lo]), np.array([h_hi, s_hi, v_hi])


def detect_object(frame, lower_hsv, upper_hsv):
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return None, mask

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_CONTOUR_AREA:
        return None, mask

    (x, y), radius = cv2.minEnclosingCircle(largest)
    return (int(x), int(y), int(radius)), mask


# ---------------------------------------------------------------------------
# VISUALIZATION
# ---------------------------------------------------------------------------
def draw_overlay(frame, detection, kalman_pos, kalman_vel, show_kalman,
                 measurement_trail, prediction_trail, fps, frames_without_detection):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2

    # Draw measurement trail (green, fading)
    for i in range(1, len(measurement_trail)):
        if measurement_trail[i - 1] is None or measurement_trail[i] is None:
            continue
        alpha = i / len(measurement_trail)
        thickness = int(alpha * 2) + 1
        color = (0, int(200 * alpha), 0)
        cv2.line(frame, measurement_trail[i - 1], measurement_trail[i], color, thickness)

    # Draw prediction trail (yellow, fading)
    if show_kalman:
        for i in range(1, len(prediction_trail)):
            if prediction_trail[i - 1] is None or prediction_trail[i] is None:
                continue
            alpha = i / len(prediction_trail)
            thickness = int(alpha * 2) + 1
            color = (0, int(200 * alpha), int(255 * alpha))
            cv2.line(frame, prediction_trail[i - 1], prediction_trail[i], color, thickness)

    # --- Measurement crosshair (GREEN) ---
    if detection is not None:
        x, y, radius = detection
        cv2.circle(frame, (x, y), radius, (0, 200, 0), 2)
        cv2.line(frame, (x - 12, y), (x + 12, y), (0, 255, 0), 2)
        cv2.line(frame, (x, y - 12), (x, y + 12), (0, 255, 0), 2)
        cv2.putText(frame, "MEAS", (x + 15, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    # --- Kalman prediction crosshair (YELLOW) ---
    if show_kalman and kalman_pos is not None:
        kx, ky = int(kalman_pos[0]), int(kalman_pos[1])

        # Draw velocity vector (shows predicted direction)
        if kalman_vel is not None:
            vx, vy = kalman_vel
            scale = 5  # scale up so it's visible
            end_x = int(kx + vx * scale)
            end_y = int(ky + vy * scale)
            cv2.arrowedLine(frame, (kx, ky), (end_x, end_y), (0, 180, 255), 2, tipLength=0.3)

        # Prediction crosshair
        cv2.line(frame, (kx - 14, ky), (kx + 14, ky), (0, 255, 255), 2)
        cv2.line(frame, (kx, ky - 14), (kx, ky + 14), (0, 255, 255), 2)
        cv2.circle(frame, (kx, ky), 18, (0, 255, 255), 1)
        cv2.putText(frame, "PRED", (kx + 15, ky + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        # Offset from center (this is the error signal for gimbal control)
        offset_x, offset_y = kx - cx, ky - cy
        cv2.putText(frame, f"Kalman offset: ({offset_x:+d}, {offset_y:+d})",
                    (10, h - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # --- Status info ---
    # Detection status
    if detection is not None:
        status = "TRACKING"
        status_color = (0, 255, 0)
    elif kalman_pos is not None and frames_without_detection < 30:
        status = f"PREDICTING (lost {frames_without_detection} frames)"
        status_color = (0, 255, 255)
    else:
        status = "LOST"
        status_color = (0, 0, 255)

    cv2.putText(frame, status, (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

    # FPS
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Kalman state
    if show_kalman and kalman_vel is not None:
        speed = np.sqrt(kalman_vel[0]**2 + kalman_vel[1]**2)
        cv2.putText(frame, f"Speed: {speed:.1f} px/frame", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # Legend
    if show_kalman:
        cv2.putText(frame, "GREEN=measurement  YELLOW=prediction  ORANGE=velocity",
                    (10, h - 85), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

    return frame


# ---------------------------------------------------------------------------
# MAIN LOOP
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Phase 1b: Color tracker + Kalman filter")
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        print("ERROR: Could not open camera")
        return

    print("Camera opened!")
    print("Controls: q=quit, c=calibrate, s=save HSV, k=toggle Kalman, r=reset filter")
    print()
    print("EXPERIMENT IDEAS:")
    print("  1. Move object slowly — green & yellow should overlap")
    print("  2. Move FAST — watch yellow lag then catch up")
    print("  3. Hide the object — yellow keeps predicting!")
    print("  4. Toss an object — watch the velocity arrow")

    # State
    lower_hsv = DEFAULT_LOWER.copy()
    upper_hsv = DEFAULT_UPPER.copy()
    calibrating = args.calibrate
    show_kalman = True
    kf = create_tracker_filter()
    kf_initialized = False
    frames_without_detection = 0

    measurement_trail = []
    prediction_trail = []
    max_trail_len = 50

    if calibrating:
        create_trackbars("Calibration", lower_hsv, upper_hsv)

    prev_time = time.time()
    fps = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        now = time.time()
        dt = now - prev_time
        fps = 0.9 * fps + 0.1 * (1.0 / max(dt, 0.001))
        prev_time = now

        if calibrating:
            lower_hsv, upper_hsv = read_trackbars("Calibration")

        # --- DETECT ---
        detection, mask = detect_object(frame, lower_hsv, upper_hsv)

        # --- KALMAN FILTER ---
        kalman_pos = None
        kalman_vel = None

        if detection is not None:
            mx, my = detection[0], detection[1]
            measurement = np.array([[mx], [my]], dtype=float)

            if not kf_initialized:
                # First detection — initialize the filter state
                kf.x = np.array([[mx], [my], [0], [0]], dtype=float)
                kf_initialized = True
                frames_without_detection = 0
            else:
                # Normal operation: predict, then correct with measurement
                update_dt(kf, dt)
                kf.predict()
                kf.update(measurement)
                frames_without_detection = 0

            kalman_pos = (kf.x[0, 0], kf.x[1, 0])
            kalman_vel = (kf.x[2, 0], kf.x[3, 0])

        elif kf_initialized:
            # No detection — predict only (this is where the magic happens!)
            # The filter coasts on its velocity estimate
            update_dt(kf, dt)
            kf.predict()
            frames_without_detection += 1

            # Only show prediction for a limited time (uncertainty grows)
            if frames_without_detection < 60:  # ~2 seconds at 30fps
                kalman_pos = (kf.x[0, 0], kf.x[1, 0])
                kalman_vel = (kf.x[2, 0], kf.x[3, 0])
            else:
                # Lost for too long — reset
                kf_initialized = False

        # --- UPDATE TRAILS ---
        if detection is not None:
            measurement_trail.append((detection[0], detection[1]))
        else:
            measurement_trail.append(None)
        if len(measurement_trail) > max_trail_len:
            measurement_trail.pop(0)

        if kalman_pos is not None:
            prediction_trail.append((int(kalman_pos[0]), int(kalman_pos[1])))
        else:
            prediction_trail.append(None)
        if len(prediction_trail) > max_trail_len:
            prediction_trail.pop(0)

        # --- DRAW ---
        frame = draw_overlay(
            frame, detection, kalman_pos, kalman_vel, show_kalman,
            measurement_trail, prediction_trail, fps, frames_without_detection
        )

        cv2.imshow("Kalman Tracker", frame)
        if calibrating:
            cv2.imshow("Mask", mask)

        # --- KEY HANDLING ---
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("c"):
            calibrating = not calibrating
            if calibrating:
                create_trackbars("Calibration", lower_hsv, upper_hsv)
            else:
                cv2.destroyWindow("Calibration")
                cv2.destroyWindow("Mask")
        elif key == ord("s"):
            print(f"\n--- HSV Range ---")
            print(f"  lower = np.array({lower_hsv.tolist()})")
            print(f"  upper = np.array({upper_hsv.tolist()})\n")
        elif key == ord("k"):
            show_kalman = not show_kalman
            print(f"Kalman visualization: {'ON' if show_kalman else 'OFF'}")
        elif key == ord("r"):
            kf = create_tracker_filter()
            kf_initialized = False
            prediction_trail.clear()
            print("Kalman filter reset!")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
