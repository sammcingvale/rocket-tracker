"""
Phase 1: Real-time color-based object tracking with OpenCV.

HOW IT WORKS:
1. Capture frames from your webcam
2. Convert each frame from BGR → HSV color space
   (HSV makes it much easier to isolate a color regardless of lighting)
3. Create a binary mask: white where the color is, black everywhere else
4. Find contours (outlines) in the mask
5. Pick the largest contour → that's our object
6. Draw a bounding circle + crosshair on screen
7. (Later) Feed this position into a Kalman filter for prediction

USAGE:
    python track_color.py              # uses default orange/red range
    python track_color.py --calibrate  # opens trackbars to find your color

CONTROLS:
    q     — quit
    c     — toggle calibration mode on/off at runtime
    s     — save current HSV range to console (for copying into code)

TIP: Start with something bright and saturated — an orange ping pong ball,
a green tennis ball, or a red marker cap works great.
"""

import argparse
import time
import cv2
import numpy as np

# ---------------------------------------------------------------------------
# DEFAULT HSV RANGE
# ---------------------------------------------------------------------------
# This range targets orange/red objects. You'll almost certainly need to
# calibrate for your specific object and lighting. Use --calibrate mode.
#
# HSV refresher:
#   H (Hue):        0-179 in OpenCV (not 0-360!)
#   S (Saturation):  0-255  (0 = grey, 255 = pure color)
#   V (Value):       0-255  (0 = black, 255 = bright)

DEFAULT_LOWER = np.array([160, 180, 0])
DEFAULT_UPPER = np.array([179, 255, 255])

# Minimum contour area in pixels — filters out noise
MIN_CONTOUR_AREA = 500


def nothing(x):
    """Dummy callback for OpenCV trackbars."""
    pass


def create_trackbars(window_name, lower, upper):
    """Create HSV range trackbars for interactive calibration."""
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("H Low", window_name, int(lower[0]), 179, nothing)
    cv2.createTrackbar("S Low", window_name, int(lower[1]), 255, nothing)
    cv2.createTrackbar("V Low", window_name, int(lower[2]), 255, nothing)
    cv2.createTrackbar("H High", window_name, int(upper[0]), 179, nothing)
    cv2.createTrackbar("S High", window_name, int(upper[1]), 255, nothing)
    cv2.createTrackbar("V High", window_name, int(upper[2]), 255, nothing)


def read_trackbars(window_name):
    """Read current trackbar positions and return (lower, upper) HSV arrays."""
    h_lo = cv2.getTrackbarPos("H Low", window_name)
    s_lo = cv2.getTrackbarPos("S Low", window_name)
    v_lo = cv2.getTrackbarPos("V Low", window_name)
    h_hi = cv2.getTrackbarPos("H High", window_name)
    s_hi = cv2.getTrackbarPos("S High", window_name)
    v_hi = cv2.getTrackbarPos("V High", window_name)
    return np.array([h_lo, s_lo, v_lo]), np.array([h_hi, s_hi, v_hi])


def detect_object(frame, lower_hsv, upper_hsv):
    """
    Detect the largest color blob in the frame.

    Returns:
        (x, y, radius) of the enclosing circle, or None if nothing found.

    LEARNING NOTES:
    - We blur the frame first to reduce noise (fewer false detections)
    - cv2.inRange() creates a binary mask: 255 where color matches, 0 elsewhere
    - Morphological operations (erode/dilate) clean up the mask edges
    - cv2.findContours() finds the outlines of white regions in the mask
    - We pick the biggest contour and fit a minimum enclosing circle
    """
    # Blur to reduce noise
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)

    # Convert BGR → HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Create binary mask for our target color
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # Clean up the mask with morphological operations:
    #   erode  = shrink white regions (removes small noise specks)
    #   dilate = grow white regions back (restores object size after erode)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if len(contours) == 0:
        return None, mask

    # Find the largest contour by area
    largest = max(contours, key=cv2.contourArea)

    # Filter out tiny detections (noise)
    if cv2.contourArea(largest) < MIN_CONTOUR_AREA:
        return None, mask

    # Fit a minimum enclosing circle around it
    (x, y), radius = cv2.minEnclosingCircle(largest)

    return (int(x), int(y), int(radius)), mask


def draw_overlay(frame, detection, trail, fps):
    """Draw tracking visualization on the frame."""
    h, w = frame.shape[:2]

    # Draw the motion trail (fading older points)
    for i in range(1, len(trail)):
        if trail[i - 1] is None or trail[i] is None:
            continue
        # Fade from thin+dim to thick+bright
        alpha = i / len(trail)
        thickness = int(alpha * 3) + 1
        color = (0, int(255 * alpha), int(255 * (1 - alpha)))  # green → red
        cv2.line(frame, trail[i - 1], trail[i], color, thickness)

    if detection is not None:
        x, y, radius = detection

        # Bounding circle
        cv2.circle(frame, (x, y), radius, (0, 255, 255), 2)

        # Crosshair
        cv2.line(frame, (x - 15, y), (x + 15, y), (0, 255, 0), 2)
        cv2.line(frame, (x, y - 15), (x, y + 15), (0, 255, 0), 2)

        # Position text
        cv2.putText(
            frame,
            f"({x}, {y}) r={radius}",
            (x + radius + 10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )

        # Offset from frame center (useful later for gimbal control)
        cx, cy = w // 2, h // 2
        offset_x, offset_y = x - cx, y - cy
        cv2.putText(
            frame,
            f"Offset: ({offset_x:+d}, {offset_y:+d})",
            (10, h - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )
    else:
        cv2.putText(
            frame,
            "NO DETECTION",
            (10, h - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
        )

    # FPS counter
    cv2.putText(
        frame,
        f"FPS: {fps:.1f}",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
    )

    return frame


def main():
    parser = argparse.ArgumentParser(description="Phase 1: Color-based object tracker")
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Open HSV calibration trackbars",
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Camera index (default: 0)",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=640,
        help="Frame width (default: 640)",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=480,
        help="Frame height (default: 480)",
    )
    args = parser.parse_args()

    # Open the camera
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        print("ERROR: Could not open camera. Check your camera index with --camera N")
        return

    print("Camera opened successfully!")
    print("Controls: q=quit, c=toggle calibration, s=save HSV values")

    # State
    lower_hsv = DEFAULT_LOWER.copy()
    upper_hsv = DEFAULT_UPPER.copy()
    calibrating = args.calibrate
    trail = []  # list of (x, y) points for motion trail
    max_trail_len = 40

    if calibrating:
        create_trackbars("Calibration", lower_hsv, upper_hsv)

    # FPS tracking
    prev_time = time.time()
    fps = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("ERROR: Failed to read frame")
            break

        # Flip horizontally so it feels like a mirror (more intuitive)
        frame = cv2.flip(frame, 1)

        # Read calibration values if in calibration mode
        if calibrating:
            lower_hsv, upper_hsv = read_trackbars("Calibration")

        # --- CORE TRACKING ---
        detection, mask = detect_object(frame, lower_hsv, upper_hsv)

        # Update trail
        if detection is not None:
            trail.append((detection[0], detection[1]))
        else:
            trail.append(None)
        if len(trail) > max_trail_len:
            trail.pop(0)

        # Calculate FPS
        now = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / max(now - prev_time, 0.001))
        prev_time = now

        # Draw overlay
        frame = draw_overlay(frame, detection, trail, fps)

        # Show windows
        cv2.imshow("Tracker", frame)

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
                print("Calibration mode ON")
            else:
                cv2.destroyWindow("Calibration")
                cv2.destroyWindow("Mask")
                print("Calibration mode OFF")

        elif key == ord("s"):
            print(f"\n--- Saved HSV Range ---")
            print(f"Lower: {lower_hsv.tolist()}")
            print(f"Upper: {upper_hsv.tolist()}")
            print(f"Code:")
            print(f"  lower = np.array({lower_hsv.tolist()})")
            print(f"  upper = np.array({upper_hsv.tolist()})")
            print()

    cap.release()
    cv2.destroyAllWindows()
    print("Done!")


if __name__ == "__main__":
    main()
