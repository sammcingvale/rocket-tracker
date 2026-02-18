# 🚀 Rocket Tracker

A high-speed object tracking system that follows model rockets in flight and "shoots them down" by painting them with a laser in real time. Step 1 to universe domination!

## Project Goals

- Detect and track fast-moving objects using computer vision
- Build a pan/tilt gimbal with super low-latency servo control
- Close the loop: camera → detection → prediction → motor command
- Mount a laser pointer and track model rockets outdoors

## Project Phases

| Phase | Focus | Status |
|-------|-------|--------|
| 1 | Software tracking (webcam + OpenCV) | 🔧 In Progress |
| 2 | Pan/tilt gimbal hardware | ⬜ Not Started |
| 3 | Closed-loop tracking (camera + gimbal) | ⬜ Not Started |
| 4 | Laser mount + speed improvements | ⬜ Not Started |
| 5 | Outdoor rocket tracking | ⬜ Not Started |

## Repo Structure

```
rocket-tracker/
├── docs/           # Design notes, diagrams, research
├── vision/         # Detection & tracking code
├── control/        # PID loops, gimbal interface
├── firmware/       # Arduino/embedded code
├── hardware/       # CAD files, wiring diagrams
├── experiments/    # Quick test scripts, benchmarks
└── README.md
```

## Tech Stack

- **Vision:** Python, OpenCV, filterpy (Kalman filters)
- **Control:** Python → C++ as latency demands grow
- **Firmware:** Arduino (servo control)
- **Compute:** Raspberry Pi 5 / Jetson Nano (later phases)
- **CAD:** FreeCAD or Fusion 360

## Getting Started

```bash
# Create a virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run the first tracking demo
python experiments/track-color.py
```

## Wiring Diagrams
![Hardware Wiring Diagram](hardware/phase2_wiring_diagram.svg)

- **Wiring connections** — every wire is color-coded to match real servo wiring conventions (red = power, brown = ground, orange = signal). The critical common ground connection between the Arduino and servo power bus is highlighted in purple dashes.
- **Physical assembly** — the right side shows how the pan/tilt bracket goes together: pan servo in the base for horizontal rotation, tilt servo on the vertical arm, with the camera and (eventually) laser mounted on top.
- **Signal flow** — the bottom-right traces the full data path: camera → OpenCV detection → Kalman filter → PID controller → serial to Arduino → PWM to servos.

## Engineering Notebook

See [`Rocket Tracker Notion`](https://www.notion.so/Model-Rocket-Tracker-3057645ef4b0804eb3d3d9b7e91a4bc6?showMoveTo=true&saveParent=true) for ongoing notes, decisions, and lessons learned.
