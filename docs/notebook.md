# Engineering Notebook

Running log of decisions, experiments, and lessons learned.

---

## 2026-02-12 — Project Kickoff

**Goal:** Build a system that can visually track a model rocket in flight and point a laser at it in real time.

**Key challenges identified:**
- Latency: the full loop (detect → predict → move) needs to be fast enough for a rocket moving at 50-100+ m/s
- Prediction: can't just follow — need to lead the target using state estimation (Kalman filter)
- Mechanical speed: hobby servos may not be fast enough; may need brushless gimbal motors
- Outdoor conditions: sky backgrounds, sun glare, distance

**First step:** Get basic color tracking working with a webcam and OpenCV.

---

<!-- Add new entries above this line -->
