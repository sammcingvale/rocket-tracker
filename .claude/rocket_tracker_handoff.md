# Rocket Tracker — Claude Code Handoff

## Project Overview
A pan/tilt gimbal system that tracks objects (currently people/colored markers) with a camera and will eventually track model rockets in flight and shine a laser at them.

**Repo:** `rocket-tracker/`
**Owner:** Sam
**Hardware lead:** Sam (with Claude.ai for planning/debugging)
**Software lead:** Claude Code (firmware, Python scripts, OpenSCAD)

## Repo Structure
```
rocket-tracker/
├── vision/              # Python CV scripts
│   ├── track_color.py   # Phase 1: HSV color tracking
│   └── track_kalman.py  # Phase 1: Kalman filter overlay
├── control/
│   ├── track_gimbal.py  # Phase 3: Main closed-loop tracker (THE primary script)
│   └── servo_serial.py  # Serial interface module
├── firmware/
│   ├── servo_controller.ino  # Phase 2-3: Arduino UNO R4 servo firmware
│   └── phase4_bench_test.ino # Phase 4: SimpleFOC single motor test (NEW)
├── hardware/
│   ├── wiring_diagram.svg
│   └── gimbal_frame.scad     # Phase 4: OpenSCAD parametric gimbal frame
├── docs/
│   ├── notebook.md           # Engineering notebook (full build log)
│   ├── assembly_guide.md
│   ├── phase4_hardware.md
│   ├── phase4_wiring_guide.md
│   ├── pid_dashboard.jsx     # Interactive PID tuning simulator
│   └── system_diagram.jsx    # 9-stage pipeline visualization
├── experiments/
├── requirements.txt
├── .gitignore
└── README.md
```

## Current Hardware

### Phase 2-3 (working, servo-based)
- Arduino UNO R4 WiFi
- 2x MG995 servo (pan + tilt) on Bolsen bracket
- Logitech C920 webcam (640×480 @ 30fps)
- USB serial at 115200 baud

### Phase 4 (in progress, brushless upgrade)
- MKS Dual FOC V3.2 + ESP32 module (needs header soldering)
- MKS ESP32 FOC V1.0 integrated board (backup, on order)
- 4x 2804 hollow shaft outrunner brushless motors + AS5600 encoders
  - Motor body: 34.5mm dia × 15mm height
  - Bolt circle: 16mm, 4x M2 holes (max 3mm screw depth)
  - Hollow shaft: 6.5mm
  - AS5600 encoder: I2C address 0x36, 12-bit (0.087° resolution)
- 12V 3A power supply
- 3D printed gimbal frame (base, yoke, camera mount) — printed and ready
- 5mW 650nm red laser module (future)

### MKS Dual FOC V3.2 Pin Map
- Motor 0 PWM: GPIO 32, 33, 25
- Motor 1 PWM: GPIO 26, 27, 14
- I2C bus 0 (encoder 0): SDA=19, SCL=18
- I2C bus 1 (encoder 1): SDA=23, SCL=5

## Phase History

### Phase 1 ✅ — Computer Vision
- HSV color detection → contour finding → centroid extraction
- Kalman filter (filterpy) for position/velocity estimation
- OpenCV, runs on Mac

### Phase 2 ✅ — Hardware Integration
- Arduino firmware: parses "Bpan,tilt\n" serial commands
- Python serial interface (pyserial) with rate limiting
- Pan/tilt servo control via PWM

### Phase 3 ✅ — Closed-Loop Tracking
Key lessons learned (IMPORTANT — don't repeat these mistakes):

**PID tuning:**
- P-only control works best for this system. I causes windup/circling. D amplifies noise.
- Current working config: P=0.015, I=0, D=0, output limit=1.5°/frame, dead zone=15px
- Three presets: conservative (P=0.015), balanced (P=0.03), aggressive (P=0.06)

**Direction inversion:**
- Camera on gimbal means correction direction can be inverted
- Runtime toggles: PAN_INVERT and TILT_INVERT (keys 'x' and 'y')

**Kalman + PID interaction:**
- PID uses RAW (smoothed) detection, NOT Kalman output
- Kalman velocity is wrong in closed-loop (can't separate object motion from camera motion)
- Kalman only feeds PID when detection is lost (coasting, max 30 frames)

**Detection smoothing:**
- Rolling average of last 3 centroids (SMOOTH_WINDOW) between detection and PID
- Raw detection still feeds Kalman (it has its own smoothing)

**Person detection mode (press 'm'):**
- Three cascading detectors: upper body Haar → face Haar → HOG full body
- Persistence filter: 3 consecutive detections within 80px before accepting
- minNeighbors=5 on cascades to reduce false positives
- Face tracking works best from a low camera angle

**Home position:**
- home() calls send_angles(SERVO_PAN_HOME, SERVO_TILT_HOME), NOT firmware 'H' command
- SERVO_TILT_HOME=80 for upward camera angle from table height

### Phase 4 🔧 — Brushless Motor Upgrade (CURRENT)
**Status:** Hardware arrived, gimbal frame printed, firmware written but untested
**Blocker:** Need to solder ESP32 headers to MKS board (soldering kit on order)

**Next steps (in order):**
1. Solder ESP32 to MKS Dual FOC V3.2
2. Single motor bench test (phase4_bench_test.ino)
3. Verify pole pair count (try 7, 11, 5 — motor vibrates with wrong count)
4. Add second motor on I2C bus 1 (SDA=23, SCL=5)
5. Write dual-motor position control firmware
6. Assemble gimbal with 3D printed frame
7. Port track_gimbal.py to use ESP32 serial instead of Arduino

### Phase 5 (future) — Field Testing
- Laser pointer integration
- Global shutter camera upgrade (OV9281) if C920 limits tracking
- Gimbal-compensated Kalman filter (subtract gimbal motion from pixel coords)
- Gravity model in Kalman for ballistic trajectory prediction
- Two-loop architecture: slow vision (30Hz laptop) + fast motor (1kHz ESP32)

## Coding Conventions
- Python: OpenCV (cv2), filterpy, pyserial, numpy
- Firmware: Arduino IDE, SimpleFOC library, ESP32 Dev Module board
- All coordinates: (0,0) = frame center for PID error, top-left for OpenCV
- Serial protocol: "Bpan,tilt\n" where pan/tilt are integer degrees
- Sam is learning — comment thoroughly, explain magic numbers

## What Claude Code Handles
- Writing and editing Python scripts (vision/, control/)
- Writing and editing Arduino/ESP32 firmware (firmware/)
- OpenSCAD parametric models (hardware/)
- Requirements, .gitignore, repo maintenance

## What Claude.ai Handles
- Hardware planning and debugging
- Interactive diagrams and dashboards (JSX artifacts)
- System architecture discussions
- PID tuning strategy
- Wiring guides and build instructions

## Environment
- Mac (Apple Silicon)
- Python 3.x with OpenCV, filterpy, pyserial, numpy
- Arduino IDE with ESP32 board support and SimpleFOC library
- Logitech C920 webcam
- Bambu P1S 3D printer with AMS 2 (currently away from printer for 2 weeks)
