# Engineering Notebook

## Project: Rocket Tracker
**Goal:** Build a system that visually tracks model rockets in flight and points a laser at them.

---

## Phase 1: Software Tracking

### 1a: Color Detection (`vision/track_color.py`)
- HSV-based color detection for a red marker cap
- Gaussian blur → HSV conversion → binary mask → morphological ops → contour detection
- Interactive calibration mode with trackbars
- Key learning: HSV separates color from brightness, making detection robust to lighting changes
- Challenge: white backgrounds wash out saturation via auto-exposure; fast motion causes blur

### 1b: Kalman Filter (`vision/track_kalman.py`)
- Added constant-velocity Kalman filter (state: [x, y, vx, vy])
- Predict step extrapolates position using velocity model
- Update step blends prediction with measurement
- Coasts on velocity estimate when detection is lost
- Parameters: R=25 (measurement noise), Q_pos=4, Q_vel=16 (process noise)

### Interactive Learning Resources
- `docs/camera_sensor_diagram.html` — how camera sensors work (Bayer filter, demosaicing)
- `docs/kalman_deep_dive.html` — step-by-step Kalman filter visualization

---

## Phase 2: Pan/Tilt Gimbal

### Hardware
- Arduino UNO R4 WiFi
- 2x MG995 metal gear servos (pan on D9, tilt on D10)
- Bolsen aluminum 2-DOF pan/tilt bracket
- 6V 2A external power supply (servos powered separately from Arduino)
- Common ground between Arduino GND and servo power supply GND

### Firmware (`firmware/servo_controller/servo_controller.ino`)
- Serial command protocol: P<angle>, T<angle>, B<pan,tilt>, H (home), S (sweep), C (query)
- Smooth interpolation (STEP_SIZE=2, STEP_DELAY=5ms) to reduce jerk
- Baud rate: 115200
- Key fix: `while(!Serial)` must have a 3-second timeout on R4 WiFi, otherwise it blocks when connecting from pyserial (vs Arduino IDE Serial Monitor)

### Serial Interface (`control/servo_serial.py`)
- Auto-detects Arduino serial port across Mac/Linux/Windows
- Interactive mode + automated demo sequence
- 2-second delay after connection for Arduino bootloader reset

### Lessons Learned
- UNO R4 WiFi uses USB CDC (software serial), not hardware UART — `while(!Serial)` behaves differently than classic UNO
- Always power servos externally, never from Arduino 5V pin
- Common ground is mandatory for PWM signal integrity

---

## Phase 3: Closed-Loop Tracking

### System (`control/track_gimbal.py`)
- Camera mounted on gimbal creates true closed-loop feedback
- Detection → raw pixel error → PID → servo angle adjustment → serial → Arduino
- Kalman filter used only for coasting (when detection lost), NOT for PID input
- Key insight: Kalman velocity estimate is unreliable in closed-loop because apparent frame motion mixes object velocity with gimbal movement

### PID Tuning Journey
1. Started with P=0.06, I=0.002, D=0.02 — violent oscillation
2. Discovered output was saturating at ±5°/frame every frame (not proportional at all)
3. Reduced output limit to 1.5°/frame — proportional behavior restored
4. Zeroed I term — was causing integral windup and circling
5. Zeroed D term — was amplifying frame-to-frame detection noise (÷ by dt=0.033 → huge derivatives)
6. Added ±15px dead zone — stops chasing detection noise when nearly centered
7. Final: P-only control with output clamping and dead zone

### Final PID Settings
```
Conservative:  P=0.015  I=0  D=0  (best for stable tracking)
Balanced:      P=0.03   I=0  D=0  (slight circling on noisy backgrounds)
Aggressive:    P=0.06   I=0  D=0  (for fast targets)
Output limit:  1.5°/frame (45°/sec max at 30fps)
Dead zone:     ±15 pixels
```

### Direction Inversion
- Pan and tilt correction direction depends on physical servo mounting
- Runtime toggle: 'x' flips pan, 'y' flips tilt
- Current config: PAN_INVERT=1, TILT_INVERT=-1
- Also removed cv2.flip() mirror since camera is outward-facing on gimbal

### Serial Reliability
- Added write_timeout=0.1 to prevent blocking when Arduino buffer fills
- Added reset_input_buffer() before each write to drain responses
- Connection test: sends '?' and checks for response on startup

### HSV Calibration
- Original (Phase 1): lower=[160, 180, 0], upper=[179, 255, 255]
- Updated (Phase 3): lower=[0, 180, 50], upper=[10, 255, 255]
- Different shade of red (hue 0-10 vs 160-179), high saturation floor rejects white backgrounds

---

## Phase 4: Hardware Upgrade (planned)

### Components Ordered
- MKS Dual FOC V3.2 + ESP32 (dual brushless motor driver)
- 2x 2804 brushless gimbal motor + AS5600 encoder
- 12V 3A power supply
- 5mW 650nm red laser module

### Why Brushless
- MG995 servos: ~400°/sec, ~1° accuracy, gear backlash
- 2804 brushless + AS5600: direct drive, 0.087° accuracy, no backlash, faster repositioning
- SimpleFOC library provides Arduino-compatible FOC control

### Key Challenge
- Two AS5600 encoders share same I2C address (0x36)
- Solution: one on I2C, one on analog/PWM output

### Frame
- Need custom 3D-printed frame for 2804 motors (Bolsen bracket is servo-specific)
- Access to makerspace for printing

---

## Phase 5: Outdoor Rocket Tracking (planned)

### Camera Considerations
- C920 may be sufficient at distance (rocket = few pixels, slow apparent motion)
- OV9281 global shutter module (720p@120fps) available as upgrade if needed (~$30-50)
- Decision: test with C920 first, upgrade based on real data

### Additional Challenges
- Sky backgrounds (blue/white) — will need different detection strategy
- Rocket is small at distance — few pixels to detect
- Motor burn exhaust — thermal signature as alternative detection
- Gravity — add gravity term to Kalman filter for ballistic prediction
