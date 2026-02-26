# Phase 4 Hardware Shopping List

## Why SimpleFOC Instead of BGC?

BGC (SimpleBGC/AlexMos) controllers are designed for camera *stabilization* — they use IMU data to keep a camera level despite external movement. That's the opposite of what we want. We need *active tracking*: "point at this angle, now this angle, now this angle" at high speed.

SimpleFOC is an open-source Arduino FOC (Field Oriented Control) library. It gives us direct position/velocity control of brushless motors using the same Arduino IDE workflow we already have. Same serial command interface, same development flow — just dramatically faster, smoother motors.

---

## The Shopping List

### 1. Motor Controller — MKS Dual FOC V3.2 + ESP32 (~$30-40)

**What:** A single board that drives TWO brushless motors, with an ESP32 microcontroller built in.

**Search:** "MKS Dual FOC V3.2 ESP32" on Amazon

**Why this one:**
- Drives both pan AND tilt motors from one board (no need for two drivers)
- ESP32 has WiFi + Bluetooth (wireless control later!)
- Built-in current sensors for FOC
- Directly compatible with the SimpleFOC Arduino library
- Programs through Arduino IDE just like your UNO R4
- 12V input, which matches the motors below

**Note:** Make sure you get the version WITH the ESP32 module (sometimes sold separately). The listing should say "Dual FOC Plus-ESP32" or similar.

---

### 2. Gimbal Motors (x2) — 2804 with AS5600 Encoder (~$15-20 each)

**What:** Brushless DC motors specifically designed for gimbals, with a magnetic position encoder built in.

**Search:** "2804 brushless gimbal motor AS5600 encoder" on Amazon
  - or "Makerbase gimbal motor 2804 AS5600"
  - DFRobot also sells a nice one (search "DFRobot 2804 BLDC")

**Why these:**
- The AS5600 encoder gives 12-bit resolution = 0.087° accuracy (vs ~1° with servos)
- SimpleFOC library has native AS5600 support over I2C
- Low KV rating = high torque at low speeds (perfect for gimbal, NOT for propellers)
- Compact: ~34mm diameter
- Hollow shaft design allows cable routing through the motor

**You need 2:** one for pan, one for tilt.

**Why brushless motors are faster than servos:**
Your MG995 servos have internal gears and top out at ~400°/sec with significant backlash. Brushless gimbal motors are direct-drive (no gears), so there's zero backlash and they can reposition in milliseconds. The motor *is* the joint.

---

### 3. Power Supply — 12V 3A DC Adapter (~$10-12)

**What:** Wall adapter to power the motors and controller.

**Search:** "12V 3A DC power supply barrel jack" on Amazon

**Why:** The 2804 motors are rated for 12V. Your existing 6V adapter won't work. 3A gives enough headroom for both motors under load.

**Alternative:** A 3S LiPo battery (11.1V) works too if you want it portable for outdoor testing later. But a wall adapter is simpler for now.

---

### 4. Laser Module — 5mW 650nm Red Dot (~$8-12)

**What:** A small focusable laser diode that mounts on the gimbal platform alongside the camera.

**Search:** "focusable 650nm 5mW red dot laser module 5V" on Amazon
  - The Adafruit one (product #1054) is reliable: "Adafruit laser diode 5mW 650nm"
  - Amazon generic: "adjustable focus 5mW 650nm laser diode module 3-5V"

**Specs to look for:**
- 650nm wavelength (red, highly visible)
- 5mW power (Class IIIa — visible but eye-safe at reasonable distances)
- 3-5V operating voltage (can be powered from the ESP32's 3.3V or 5V pin)
- Focusable lens (adjustable dot size for different distances)
- Wire leads (red +, black -)

**DO NOT buy anything over 5mW** — higher power lasers are dangerous and unnecessary. 5mW is bright enough to see the dot on a rocket at distance.

**Safety:** Never look directly into the beam or point it at people/aircraft. We'll add a software safety interlock (laser only activates when tracking is active).

---

### 5. Breadboard + Jumper Wires (~$8)

**What:** You still need a breadboard from Phase 2, and it'll be useful here too for prototyping connections.

**Search:** "mini breadboard jumper wire kit" on Amazon

---

### 6. Gimbal Frame — 3D Printed or Fabricated

**This is the one item you can't just buy off the shelf,** because the Bolsen bracket was designed for standard servos, not brushless motors.

**Options (pick one):**
- **3D print:** If you have access to a 3D printer (library, makerspace, friend), I can generate an STL file designed for the 2804 motors
- **Thingiverse:** Search "SimpleFOC 2804 pan tilt" — there are several community designs
- **Fabricate from aluminum L-brackets:** Buy some aluminum angle from a hardware store and drill mounting holes. Less pretty, but functional
- **Adapt the Bolsen bracket:** With creative mounting (epoxy, zip ties, 3D printed adapter plates), you might be able to attach 2804 motors to the existing bracket

We can figure this out once the electronic parts arrive. The software work can happen immediately.

---

## Summary

| Item | Est. Price | Notes |
|------|-----------|-------|
| MKS Dual FOC V3.2 + ESP32 | ~$30-40 | One board drives both motors |
| 2804 Gimbal Motor + AS5600 (x2) | ~$30-40 | Position accuracy: 0.087° |
| 12V 3A Power Supply | ~$10-12 | Wall adapter for bench testing |
| 5mW 650nm Laser Module | ~$8-12 | Focusable, 3-5V, wire leads |
| Mini Breadboard + Wires | ~$8 | If you don't have one yet |
| Gimbal Frame | $0-30 | 3D print or fabricate |
| **TOTAL** | **~$86-140** | |

---

## What Changes in the Software

The great news: not much! The control flow stays the same:

```
Camera → OpenCV → Kalman → PID → Serial → Motor Controller → Motors
```

What changes:
- Arduino firmware switches from Servo library → SimpleFOC library
- Serial commands send precise float angles instead of integer degrees
- ESP32 replaces Arduino UNO R4 (but same Arduino IDE, same workflow)
- PID gains will need re-tuning (faster motors = less aggressive P needed)
- Add laser on/off control (digital pin HIGH/LOW)

Your UNO R4 + servos become your backup/test rig, which is handy.

---

## What To Do While Hardware Ships

1. **Mount the C920 on the current servo gimbal** and test closed-loop tracking
2. **Tune PID** on the servo system — this knowledge transfers directly
3. **I'll write the SimpleFOC firmware** so it's ready when parts arrive
4. **Build/find a gimbal frame design** for the 2804 motors
