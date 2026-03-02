"""
Phase 3: Closed-Loop Object Tracking with Pan/Tilt Gimbal

THIS IS WHERE IT ALL COMES TOGETHER:
  Camera → Detection → Kalman Filter → PID Controller → Serial → Arduino → Servos
  
  The camera sees an object. OpenCV detects it. The Kalman filter predicts
  its position. The PID controller calculates how much to adjust the servo
  angles to center the object in the frame. The Arduino moves the servos.
  Next frame, repeat.

HOW THE PID CONTROLLER WORKS:
  The "error" is how far the object is from the center of the frame, in pixels.
  The PID controller converts this pixel error into a servo angle adjustment:

    P (Proportional): Correction proportional to the error RIGHT NOW.
        Object is 100px left of center → turn left proportionally.
        Larger P = more aggressive response, but can overshoot.

    I (Integral): Correction based on ACCUMULATED past error.
        If the object has been slightly left for many frames, I builds up
        and pushes harder. Fixes steady-state offset.
        Larger I = eliminates persistent error, but can cause oscillation.

    D (Derivative): Correction based on how fast the error is CHANGING.
        If the object is moving left quickly, D applies extra correction.
        Acts as a damper — reduces overshoot and jitter.
        Larger D = smoother, but can slow response.

  output = P * error + I * sum(past_errors) + D * (error - prev_error)

USAGE:
    python track_gimbal.py --camera 1
    python track_gimbal.py --camera 1 --port /dev/tty.usbmodem1101
    python track_gimbal.py --camera 1 --calibrate

CONTROLS:
    q       — quit
    c       — toggle HSV calibration
    s       — save HSV values
    k       — toggle Kalman visualization
    p       — pause/resume tracking (servos hold position)
    h       — home servos to center
    1/2/3   — PID presets (conservative / balanced / aggressive)
    +/-     — fine-tune P gain up/down

TIP: Start with the object close to center, then slowly move it around.
     If the gimbal oscillates (shakes back and forth), reduce P gain with '-'.
"""

import argparse
import time
import sys
import glob
import cv2
import numpy as np
from filterpy.kalman import KalmanFilter
import serial


# ---------------------------------------------------------------------------
# HSV RANGE — paste your calibrated values from Phase 1
# ---------------------------------------------------------------------------
DEFAULT_LOWER = np.array([0, 180, 50])
DEFAULT_UPPER = np.array([10, 255, 255])

MIN_CONTOUR_AREA = 500

# ---------------------------------------------------------------------------
# SERVO CONFIGURATION
# ---------------------------------------------------------------------------
# Adjust HOME values so the gimbal points straight ahead at startup.
# If your gimbal is slightly right/up at home, tweak these:
SERVO_PAN_HOME  = 90    # Decrease to point more left, increase for right
SERVO_TILT_HOME = 90    # Decrease to tilt down, increase to tilt up
SERVO_MIN       = 10
SERVO_MAX       = 170

# ---------------------------------------------------------------------------
# DETECTION SMOOTHING
# ---------------------------------------------------------------------------
# Rolling average of the last N centroids to reduce jitter.
# Higher = smoother but adds latency. 3-5 is a good range.
SMOOTH_WINDOW = 3

# ---------------------------------------------------------------------------
# DIRECTION INVERSION
# ---------------------------------------------------------------------------
# These flip the correction direction for each axis.
# If the gimbal runs away when it sees the object, flip the relevant axis.
# Press 'x' to flip pan direction, 'y' to flip tilt direction.
PAN_INVERT  = 1     # 1 or -1
TILT_INVERT = -1    # 1 or -1

# ---------------------------------------------------------------------------
# PID PRESETS
# ---------------------------------------------------------------------------
# These map pixel error → servo angle adjustment per frame.
# Start conservative and increase if tracking is too sluggish.
#
# The values are tuned for 640x480 resolution where the max error
# from center is ~320px horizontally, ~240px vertically.

PID_PRESETS = {
    "conservative": {"P": 0.015, "I": 0.0, "D": 0.0},
    "balanced":     {"P": 0.03,  "I": 0.0, "D": 0.0},
    "aggressive":   {"P": 0.06,  "I": 0.0, "D": 0.0},
}

DEFAULT_PID = "balanced"


# ---------------------------------------------------------------------------
# PID CONTROLLER
# ---------------------------------------------------------------------------
class PIDController:
    """
    Simple PID controller for one axis (pan or tilt).

    Input:  pixel error (object position - frame center)
    Output: servo angle adjustment (degrees per frame)

    LEARNING NOTES:
    ───────────────
    A PID controller is a feedback loop. It measures the error between
    where we ARE and where we WANT TO BE, then outputs a correction.

    The three terms work together:
    - P gets you close quickly (but overshoots)
    - D slows you down as you approach (reduces overshoot)
    - I cleans up any remaining offset over time

    Without D, the gimbal will oscillate around the target.
    Without I, it might settle with a small persistent offset.
    Without P, it barely moves at all.

    The trick is tuning: too much P = oscillation, too much I = slow
    oscillation that builds up, too much D = sluggish and jittery.
    """

    def __init__(self, kp, ki, kd, output_limit=1.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit  # Max degrees per frame

        self.prev_error = 0.0
        self.integral = 0.0
        self.integral_limit = 100.0  # Anti-windup clamp

    def update(self, error, dt):
        """
        Compute PID output given current error and time step.

        Returns a servo angle adjustment in degrees.
        """
        # Proportional: react to current error
        p_term = self.kp * error

        # Integral: accumulate past error (with anti-windup)
        self.integral += error * dt
        self.integral = max(-self.integral_limit,
                           min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral

        # Derivative: react to rate of change
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative

        self.prev_error = error

        # Sum and clamp
        output = p_term + i_term + d_term
        output = max(-self.output_limit, min(self.output_limit, output))

        return output

    def reset(self):
        """Reset accumulated state."""
        self.prev_error = 0.0
        self.integral = 0.0


# ---------------------------------------------------------------------------
# KALMAN FILTER (same as Phase 1b)
# ---------------------------------------------------------------------------
def create_tracker_filter():
    kf = KalmanFilter(dim_x=4, dim_z=2)
    kf.F = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], dtype=float)
    kf.H = np.array([[1,0,0,0],[0,1,0,0]], dtype=float)
    kf.R = np.array([[25,0],[0,25]], dtype=float)
    kf.Q = np.array([[4,0,0,0],[0,4,0,0],[0,0,16,0],[0,0,0,16]], dtype=float)
    kf.P *= 1000
    return kf


def update_kf_dt(kf, dt):
    kf.F[0, 2] = dt
    kf.F[1, 3] = dt


# ---------------------------------------------------------------------------
# DETECTION (same as Phase 1)
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


# --- PERSON DETECTION (multi-strategy) ---
# Uses three detectors in priority order:
#   1. Upper body cascade — works when camera sees torso and head
#   2. Face cascade — works when only head/shoulders visible
#   3. HOG full body — fallback when full body is in frame
#
# This handles the low-camera problem: HOG needs head-to-knees,
# but from a table height you often only see torso-up.

_upper_body = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_upperbody.xml"
)
_face = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
)
_hog = cv2.HOGDescriptor()
_hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Persistence filter: require CONFIRM_FRAMES consecutive detections
# within CONFIRM_RADIUS pixels before accepting a new target.
# Once confirmed, single-frame detections are accepted (for responsiveness).
# Resets after LOST_FRAMES with no detection.
CONFIRM_FRAMES = 3      # Frames needed to confirm a new detection
CONFIRM_RADIUS = 80     # Max pixel distance to count as "same spot"
LOST_FRAMES = 10        # Frames without detection before requiring re-confirm

_confirm_buffer = []     # Recent unconfirmed detections
_confirmed = False       # Whether we have a confirmed target
_frames_since_detect = 0

def _reset_persistence():
    global _confirm_buffer, _confirmed, _frames_since_detect
    _confirm_buffer = []
    _confirmed = False
    _frames_since_detect = 0

def _check_persistence(detection):
    """
    Filter out transient false positives.
    Returns detection if confirmed, None if still unconfirmed.
    """
    global _confirm_buffer, _confirmed, _frames_since_detect

    if detection is None:
        _frames_since_detect += 1
        if _frames_since_detect > LOST_FRAMES:
            _confirmed = False
            _confirm_buffer = []
        return None

    _frames_since_detect = 0
    cx, cy = detection[0], detection[1]

    # Already tracking — accept immediately (person moves around)
    if _confirmed:
        return detection

    # Not yet confirmed — check if this is consistent with recent detections
    _confirm_buffer.append((cx, cy))
    if len(_confirm_buffer) > CONFIRM_FRAMES:
        _confirm_buffer.pop(0)

    if len(_confirm_buffer) >= CONFIRM_FRAMES:
        # Check all recent detections are near each other
        avg_x = sum(p[0] for p in _confirm_buffer) / len(_confirm_buffer)
        avg_y = sum(p[1] for p in _confirm_buffer) / len(_confirm_buffer)
        all_close = all(
            abs(p[0] - avg_x) < CONFIRM_RADIUS and abs(p[1] - avg_y) < CONFIRM_RADIUS
            for p in _confirm_buffer
        )
        if all_close:
            _confirmed = True
            return detection

    return None  # Not yet confirmed


def detect_person(frame):
    """
    Detect a person using cascading strategies + persistence filter.
    Returns same format as detect_object: (x, y, radius), mask_or_None
    """
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    raw_detection = None

    # --- Strategy 1: Upper body cascade ---
    upper_rects = _upper_body.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,        # was 3 — tighter to reduce false positives
        minSize=(80, 80)       # was 60 — ignore small false matches
    )
    if len(upper_rects) > 0:
        areas = [rw * rh for (_, _, rw, rh) in upper_rects]
        best = upper_rects[areas.index(max(areas))]
        rx, ry, rw, rh = best
        cx = rx + rw // 2
        cy = ry + rh // 2
        radius = max(rw, rh) // 2
        raw_detection = (cx, cy, radius)

    # --- Strategy 2: Face cascade ---
    if raw_detection is None:
        face_rects = _face.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,    # was 4
            minSize=(50, 50)   # was 40
        )
        if len(face_rects) > 0:
            areas = [rw * rh for (_, _, rw, rh) in face_rects]
            best = face_rects[areas.index(max(areas))]
            rx, ry, rw, rh = best
            cx = rx + rw // 2
            cy = ry + rh // 2
            radius = max(rw, rh) // 2
            raw_detection = (cx, cy, radius)

    # --- Strategy 3: HOG full body (fallback) ---
    if raw_detection is None:
        scale = 320.0 / w
        small = cv2.resize(frame, (320, int(h * scale)))
        rects, weights = _hog.detectMultiScale(
            small, winStride=(8, 8), padding=(4, 4), scale=1.05
        )
        if len(rects) > 0:
            best_idx = weights.argmax()
            rx, ry, rw, rh = rects[best_idx]
            rx = int(rx / scale)
            ry = int(ry / scale)
            rw = int(rw / scale)
            rh = int(rh / scale)
            cx = rx + rw // 2
            cy = ry + int(rh * 0.35)
            radius = max(rw, rh) // 2
            raw_detection = (cx, cy, radius)

    # --- Persistence filter ---
    confirmed = _check_persistence(raw_detection)
    return confirmed, None


# ---------------------------------------------------------------------------
# SERIAL CONNECTION
# ---------------------------------------------------------------------------
def find_serial_port():
    for pattern in ["/dev/tty.usbmodem*", "/dev/tty.usbserial*",
                    "/dev/ttyACM*", "/dev/ttyUSB*"]:
        ports = glob.glob(pattern)
        if ports:
            return ports[0]
    return None


class GimbalSerial:
    """Lightweight serial interface to the Arduino servo controller."""

    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=0.1)
        time.sleep(2)  # Wait for Arduino reset
        # Flush startup messages
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # Verify the connection actually works
        self.ser.timeout = 1  # Longer timeout for test
        self.ser.write(b"?\n")
        time.sleep(0.1)
        response = b""
        while self.ser.in_waiting:
            response += self.ser.readline()
        self.ser.timeout = 0.05  # Restore fast timeout

        if response:
            print(f"  Arduino responded: {response.decode(errors='replace').strip()[:60]}...")
        else:
            print("  WARNING: Arduino did not respond to test command!")
            print("  Check that servo_controller.ino is uploaded.")

    def send_angles(self, pan, tilt):
        """Send both angles in a single command."""
        pan = int(max(SERVO_MIN, min(SERVO_MAX, pan)))
        tilt = int(max(SERVO_MIN, min(SERVO_MAX, tilt)))
        try:
            # Drain any pending responses so the buffer doesn't fill up
            if self.ser.in_waiting:
                self.ser.reset_input_buffer()
            self.ser.write(f"B{pan},{tilt}\n".encode())
        except serial.SerialTimeoutException:
            pass  # Skip this frame if write times out

    def home(self):
        """Move to configured home position (not firmware's hardcoded 90/90)."""
        self.send_angles(SERVO_PAN_HOME, SERVO_TILT_HOME)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


# ---------------------------------------------------------------------------
# VISUALIZATION
# ---------------------------------------------------------------------------
def draw_overlay(frame, detection, kalman_pos, pan, tilt, pid_preset_name,
                 pan_pid, tilt_pid, fps, tracking_active, detection_mode="color"):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2

    # Center crosshair (where we WANT the object to be)
    cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (100, 100, 100), 1)
    cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (100, 100, 100), 1)

    # Detection
    if detection is not None:
        x, y, radius = detection
        cv2.circle(frame, (x, y), radius, (0, 200, 0), 2)
        cv2.line(frame, (x - 10, y), (x + 10, y), (0, 255, 0), 2)
        cv2.line(frame, (x, y - 10), (x, y + 10), (0, 255, 0), 2)

        # Error line from center to object
        cv2.line(frame, (cx, cy), (x, y), (0, 0, 255), 1)

        # Error values
        err_x = x - cx
        err_y = y - cy
        cv2.putText(frame, f"Error: ({err_x:+d}, {err_y:+d})",
                    (10, h - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 150, 255), 1)

    # Kalman prediction
    if kalman_pos is not None:
        kx, ky = int(kalman_pos[0]), int(kalman_pos[1])
        cv2.circle(frame, (kx, ky), 16, (0, 255, 255), 1)
        cv2.line(frame, (kx - 8, ky), (kx + 8, ky), (0, 255, 255), 1)
        cv2.line(frame, (kx, ky - 8), (kx, ky + 8), (0, 255, 255), 1)

    # Status bar
    status = "TRACKING" if tracking_active else "PAUSED"
    status_color = (0, 255, 0) if tracking_active else (0, 100, 255)
    cv2.putText(frame, status, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

    cv2.putText(frame, f"FPS: {fps:.0f}", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Detection mode
    mode_color = (200, 200, 0) if detection_mode == "color" else (0, 200, 200)
    cv2.putText(frame, f"Mode: {detection_mode.upper()}", (10, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 1)

    # Servo angles
    cv2.putText(frame, f"Pan: {pan:.0f}  Tilt: {tilt:.0f}",
                (10, h - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 200, 0), 1)

    # PID info
    cv2.putText(frame, f"PID: {pid_preset_name} (P={pan_pid.kp:.3f})",
                (10, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (200, 200, 200), 1)

    # Controls hint
    cv2.putText(frame, "q=quit h=home p=pause 1/2/3=PID +/-=P x/y=flip m=mode",
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (120, 120, 120), 1)

    return frame


# ---------------------------------------------------------------------------
# MAIN LOOP
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Phase 3: Closed-loop gimbal tracking")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--port", type=str, help="Arduino serial port (auto-detect if omitted)")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument("--no-serial", action="store_true",
                        help="Run without Arduino (test vision + PID only)")
    args = parser.parse_args()

    # --- Camera ---
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print("ERROR: Could not open camera")
        return

    # --- Serial ---
    gimbal = None
    if not args.no_serial:
        port = args.port or find_serial_port()
        if port:
            try:
                gimbal = GimbalSerial(port)
                print(f"Connected to gimbal on {port}")
            except serial.SerialException as e:
                print(f"WARNING: Could not connect to gimbal: {e}")
                print("Running in vision-only mode (use --no-serial to suppress)")
        else:
            print("WARNING: No Arduino found. Running in vision-only mode.")
    else:
        print("Running without serial (--no-serial)")

    # --- State ---
    lower_hsv = DEFAULT_LOWER.copy()
    upper_hsv = DEFAULT_UPPER.copy()
    calibrating = args.calibrate

    kf = create_tracker_filter()
    kf_initialized = False
    frames_without_detection = 0

    # PID controllers (one per axis)
    pid_name = DEFAULT_PID
    pid_params = PID_PRESETS[pid_name]
    pan_pid = PIDController(pid_params["P"], pid_params["I"], pid_params["D"])
    tilt_pid = PIDController(pid_params["P"], pid_params["I"], pid_params["D"])

    # Current servo positions (start at home)
    pan_angle = float(SERVO_PAN_HOME)
    tilt_angle = float(SERVO_TILT_HOME)

    tracking_active = True

    if calibrating:
        create_trackbars("Calibration", lower_hsv, upper_hsv)

    if gimbal:
        gimbal.home()
        time.sleep(0.5)

    prev_time = time.time()
    fps = 0.0

    # Rate limit serial sends (~30Hz is plenty)
    last_serial_send = 0.0
    serial_send_interval = 0.033  # ~30 Hz

    # Detection smoothing buffer (rolling average of last N centroids)
    smooth_buffer = []

    print("\nTracking started!")
    print("Controls: q=quit, h=home, p=pause, 1/2/3=PID preset, +/-=tune P")
    print("          x=flip pan direction, y=flip tilt direction")
    print("          m=switch detection mode (color / person)\n")

    global PAN_INVERT, TILT_INVERT

    # Detection mode: "color" (HSV) or "person" (HOG)
    detection_mode = "color"

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)  # Mirror — remove this line if camera faces outward
        # If tracking still runs away after flipping with x/y keys,
        # try commenting out the line above
        now = time.time()
        dt = now - prev_time
        fps = 0.9 * fps + 0.1 * (1.0 / max(dt, 0.001))
        prev_time = now

        h_frame, w_frame = frame.shape[:2]
        cx, cy = w_frame // 2, h_frame // 2

        if calibrating:
            lower_hsv, upper_hsv = read_trackbars("Calibration")

        # --- DETECT ---
        if detection_mode == "color":
            detection, mask = detect_object(frame, lower_hsv, upper_hsv)
        else:
            detection, mask = detect_person(frame)

        # --- SMOOTH DETECTION ---
        # Rolling average of recent centroids to reduce jitter.
        # Raw detection still feeds Kalman (it has its own smoothing).
        # Smoothed detection feeds PID.
        smoothed_detection = None
        if detection is not None:
            smooth_buffer.append((float(detection[0]), float(detection[1])))
            if len(smooth_buffer) > SMOOTH_WINDOW:
                smooth_buffer.pop(0)
            avg_x = sum(p[0] for p in smooth_buffer) / len(smooth_buffer)
            avg_y = sum(p[1] for p in smooth_buffer) / len(smooth_buffer)
            smoothed_detection = (avg_x, avg_y)
        else:
            smooth_buffer.clear()  # Reset on detection loss

        # --- KALMAN FILTER ---
        kalman_pos = None

        if detection is not None:
            mx, my = detection[0], detection[1]
            measurement = np.array([[mx], [my]], dtype=float)

            if not kf_initialized:
                kf.x = np.array([[mx], [my], [0], [0]], dtype=float)
                kf_initialized = True
                frames_without_detection = 0
            else:
                update_kf_dt(kf, dt)
                kf.predict()
                kf.update(measurement)
                frames_without_detection = 0

            kalman_pos = (kf.x[0, 0], kf.x[1, 0])

        elif kf_initialized:
            update_kf_dt(kf, dt)
            kf.predict()
            frames_without_detection += 1
            if frames_without_detection < 30:
                kalman_pos = (kf.x[0, 0], kf.x[1, 0])
            else:
                kf_initialized = False

        # --- PID CONTROL ---
        # Use MEASURED position when we can see the object.
        # Only fall back to Kalman prediction when detection is lost.
        # Why: the Kalman velocity estimate is unreliable in a closed-loop
        # gimbal system because apparent frame motion mixes the object's
        # real velocity with the gimbal's own movement.
        pid_target = None

        if tracking_active and smoothed_detection is not None:
            # We can see it — use the smoothed measurement
            pid_target = smoothed_detection
        elif tracking_active and kalman_pos is not None:
            # Can't see it — coast on Kalman prediction
            pid_target = kalman_pos

        if pid_target is not None:
            # Error = object position - frame center (in pixels)
            error_x = pid_target[0] - cx
            error_y = pid_target[1] - cy

            # Dead zone: ignore errors smaller than 15 pixels
            # This prevents chasing detection noise when nearly centered
            if abs(error_x) < 15:
                error_x = 0
            if abs(error_y) < 15:
                error_y = 0

            # PID outputs are servo angle adjustments (degrees)
            pan_adjustment = PAN_INVERT * pan_pid.update(error_x, dt)
            tilt_adjustment = TILT_INVERT * tilt_pid.update(error_y, dt)

            # Apply adjustments to current servo angles
            pan_angle += pan_adjustment
            tilt_angle += tilt_adjustment

            # Clamp to servo limits
            pan_angle = max(SERVO_MIN, min(SERVO_MAX, pan_angle))
            tilt_angle = max(SERVO_MIN, min(SERVO_MAX, tilt_angle))

            # Send to Arduino (rate-limited)
            if gimbal and (now - last_serial_send) >= serial_send_interval:
                gimbal.send_angles(pan_angle, tilt_angle)
                last_serial_send = now
                # Debug: print every ~1 second so we can see what's happening
                if int(now * 2) % 2 == 0:
                    print(f"  err=({error_x:+.0f},{error_y:+.0f})  "
                          f"adj=({pan_adjustment:+.2f},{tilt_adjustment:+.2f})  "
                          f"servo=({pan_angle:.0f},{tilt_angle:.0f})")

        # --- DRAW ---
        frame = draw_overlay(frame, detection, kalman_pos, pan_angle, tilt_angle,
                            pid_name, pan_pid, tilt_pid, fps, tracking_active,
                            detection_mode)

        cv2.imshow("Gimbal Tracker", frame)
        if calibrating and mask is not None:
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
            print(f"  lower = np.array({lower_hsv.tolist()})")
            print(f"  upper = np.array({upper_hsv.tolist()})")

        elif key == ord("k"):
            pass  # Kalman vis already shown via crosshair

        elif key == ord("p"):
            tracking_active = not tracking_active
            if not tracking_active:
                pan_pid.reset()
                tilt_pid.reset()
            print(f"Tracking: {'ACTIVE' if tracking_active else 'PAUSED'}")

        elif key == ord("h"):
            pan_angle = float(SERVO_PAN_HOME)
            tilt_angle = float(SERVO_TILT_HOME)
            pan_pid.reset()
            tilt_pid.reset()
            if gimbal:
                gimbal.home()
            print("Homed to center")

        elif key == ord("1"):
            pid_name = "conservative"
            p = PID_PRESETS[pid_name]
            pan_pid = PIDController(p["P"], p["I"], p["D"])
            tilt_pid = PIDController(p["P"], p["I"], p["D"])
            print(f"PID: {pid_name} (P={p['P']})")

        elif key == ord("2"):
            pid_name = "balanced"
            p = PID_PRESETS[pid_name]
            pan_pid = PIDController(p["P"], p["I"], p["D"])
            tilt_pid = PIDController(p["P"], p["I"], p["D"])
            print(f"PID: {pid_name} (P={p['P']})")

        elif key == ord("3"):
            pid_name = "aggressive"
            p = PID_PRESETS[pid_name]
            pan_pid = PIDController(p["P"], p["I"], p["D"])
            tilt_pid = PIDController(p["P"], p["I"], p["D"])
            print(f"PID: {pid_name} (P={p['P']})")

        elif key == ord("+") or key == ord("="):
            pan_pid.kp *= 1.2
            tilt_pid.kp *= 1.2
            print(f"P gain: {pan_pid.kp:.4f}")

        elif key == ord("-"):
            pan_pid.kp *= 0.8
            tilt_pid.kp *= 0.8
            print(f"P gain: {pan_pid.kp:.4f}")

        elif key == ord("x"):
            PAN_INVERT *= -1
            pan_pid.reset()
            print(f"Pan direction flipped (now {'normal' if PAN_INVERT == 1 else 'inverted'})")

        elif key == ord("y"):
            TILT_INVERT *= -1
            tilt_pid.reset()
            print(f"Tilt direction flipped (now {'normal' if TILT_INVERT == 1 else 'inverted'})")

        elif key == ord("m"):
            detection_mode = "person" if detection_mode == "color" else "color"
            smooth_buffer.clear()
            pan_pid.reset()
            tilt_pid.reset()
            _reset_persistence()
            print(f"Detection mode: {detection_mode.upper()}"
                  f"{' (slower — ~10-15fps)' if detection_mode == 'person' else ''}")

    # Cleanup
    if gimbal:
        gimbal.home()
        time.sleep(0.5)
        gimbal.close()
    cap.release()
    cv2.destroyAllWindows()
    print("Done!")


if __name__ == "__main__":
    main()
