# Phase 2: Assembly & Wiring Guide

Step-by-step instructions for building the pan/tilt gimbal and wiring everything together.

**Time estimate:** 30–45 minutes

---

## What You Have

| Item | Quantity Needed |
|------|----------------|
| Bolsen aluminum pan/tilt bracket | 1 set (you have 2) |
| MG995 servos | 2 (you have 4) |
| Arduino UNO R4 WiFi | 1 |
| 6V 2A power adapter | 1 |
| Logitech C920 webcam | 1 |
| Dupont jumper wires | Several M-to-M and M-to-F |
| USB-C cable | 1 |
| Screws & servo horns (came with servos + bracket) | Various |

### What You Might Still Need

| Item | Why | Workaround |
|------|-----|------------|
| Mini breadboard (~$5) | Easiest way to share power between 2 servos + ground | Can twist wires together instead (see Step 3 Option B) |
| Small Phillips screwdriver | For the bracket screws | — |

---

## STEP 1: Assemble the Pan/Tilt Bracket

The bracket kit has two main pieces of aluminum and a bag of screws/bearings. Here's how they go together.

### Understand the Parts

Open one set of the Bolsen bracket kit. You should find:

```
┌─────────────────────────────────┐
│  1x "L-shaped" or "U-shaped"   │   ← This is the TILT bracket
│     vertical bracket            │      (holds the camera platform)
│                                 │
│  1x Base plate / pan platform   │   ← This is the PAN bracket
│     (flat piece with holes)     │      (sits on the table)
│                                 │
│  1x Ball bearing                │   ← Goes between pan servo and base
│                                 │
│  1x Metal servo horn (round)    │   ← Attaches servo shaft to bracket
│                                 │
│  Bag of small screws            │   ← Various sizes for assembly
└─────────────────────────────────┘
```

### Assembly Order

**1a. Mount the PAN servo into the base plate.**

The base plate has a rectangular cutout that fits the MG995 servo body. The servo slides in from below so the output shaft (the nubby gear on top) pokes up through the hole. Use the screws that came with the bracket to secure the servo body to the base plate through the mounting tabs on either side of the servo.

```
        TOP VIEW (base plate)
    ┌──────────────────────────┐
    │                          │
    │    ┌──────────────┐      │
    │    │  servo body   │      │
    │    │   ┌──┐       │      │
    │    │   │⚙️│ shaft  │      │
    │    │   └──┘       │      │
    │    └──────────────┘      │
    │   screw↑          ↑screw │
    └──────────────────────────┘
```

**The servo wire should exit toward the back/bottom** — route it so it doesn't get pinched when the bracket rotates.

**1b. Attach the metal servo horn to the vertical bracket.**

Take the round metal servo horn disc. Screw it onto the bottom of the vertical/U-shaped bracket using the small screws. The horn has a splined center hole (star-shaped teeth) that will mate with the pan servo's output shaft.

**1c. Press the vertical bracket onto the pan servo shaft.**

Push the servo horn (now attached to the vertical bracket) down onto the pan servo's output shaft. The splines should mesh. You want this at approximately the center of the servo's range — so first power up and center the servo (we'll do this in Step 4), OR just eyeball it so the vertical bracket is roughly perpendicular to the base plate.

Secure with the small screw that goes into the top of the servo shaft (came with the servo).

**1d. Mount the TILT servo into the vertical bracket.**

The vertical U-shaped bracket has a rectangular cutout on one side. Mount the second MG995 servo into this cutout, with its output shaft pointing to the side. Screw it in using the bracket screws.

```
        SIDE VIEW (assembled)

          ┌───────────┐  ← camera/laser mount here
          │  TILT     │
          │  servo ⚙️──┤  ← tilt shaft points sideways
          │           │
          │  vertical │
          │  bracket  │
          │           │
          └─────┬─────┘
                │
           ┌────⚙️────┐   ← pan shaft points up
           │ PAN servo │
           └──────────┘
           ════════════   ← base plate (sits on table)
```

**1e. Attach a servo horn to the tilt output shaft.**

Use one of the servo horn "arms" (the cross-shaped or single-arm piece that came with the MG995 servos). Screw it onto the tilt servo's output shaft. This is where you'll eventually mount the camera. For now, just having the horn attached is fine.

**Don't mount the webcam yet** — let's verify the servos work first.

---

## STEP 2: Understand the Wiring

Before plugging anything in, let's understand what connects to what and why.

### The Three Circuits

Your system has three separate circuits that need to work together:

```
CIRCUIT 1: SERVO POWER                    CIRCUIT 2: ARDUINO
(6V adapter → servos)                     (USB → Arduino)

  ┌──────────┐    ┌─────────┐             ┌──────┐    ┌─────────┐
  │ 6V Power │───→│ Servo 1 │             │Laptop│───→│ Arduino │
  │ Adapter  │───→│ Servo 2 │             │ USB  │    │ UNO R4  │
  └──────────┘    └─────────┘             └──────┘    └─────────┘
       Has its own GND ←──────────────────────→ Has its own GND

                        ↑
                        │
              These two GNDs MUST be
              connected together!
              (That's the "common ground")


CIRCUIT 3: SIGNAL (connects 1 and 2)

  Arduino Pin 9  ──── orange wire ────→ Pan servo signal
  Arduino Pin 10 ──── orange wire ────→ Tilt servo signal
  Arduino GND    ──── purple wire ────→ Power supply GND
                                        (common ground)
```

### Why Common Ground Matters

The Arduino sends a PWM signal (a pulse of voltage) on pins 9 and 10. The servo reads this pulse to know what angle to move to. But voltage is *relative* — it's the difference between two wires. The servo measures the signal voltage relative to its own ground. If the servo's ground and the Arduino's ground aren't connected, they could be at completely different voltage levels, and the servo would misread the signal.

Think of it like this: if I say "the shelf is 3 feet up," you need to know *from what*. From the floor? From the table? The common ground is the "floor" that both devices agree on.

### The Servo Wire Colors (MG995)

Each MG995 servo has a 3-pin connector with three wires:

```
┌────────────────────────────────────┐
│  Brown wire  = GND  (ground)       │
│  Red wire    = +V   (power, 5-6V)  │
│  Orange wire = SIG  (PWM signal)   │
└────────────────────────────────────┘
```

---

## STEP 3: Wire It Up

### Option A: With a Breadboard (recommended)

A breadboard makes this clean and easy. The two long rails on the sides are the power bus.

```
BREADBOARD LAYOUT
═══════════════════════════════════════════════

  (+) ●───●───●───●───●───●───●───●  ← Red rail (+6V)
  (-) ●───●───●───●───●───●───●───●  ← Blue rail (GND)

  Row connections in the middle:
  a  ●   ●   ●   ●   ●   ●   ●   ●
  b  ●   ●   ●   ●   ●   ●   ●   ●
  c  ●   ●   ●   ●   ●   ●   ●   ●
  d  ●   ●   ●   ●   ●   ●   ●   ●
  e  ●   ●   ●   ●   ●   ●   ●   ●

═══════════════════════════════════════════════
```

**Step 3A-1: Connect the power adapter to the breadboard power rails.**

Your 6V adapter has a barrel jack. You need to get the +6V and GND wires onto the breadboard. Options:
- **Barrel jack breakout board** (if you bought one) — plug adapter in, screw terminals give you +/- wires
- **Cut the barrel jack off** — strip the wires (center wire = +6V, outer braid = GND). Use a multimeter to verify polarity if you're unsure!
- **If your adapter came with screw terminal tips** (check the interchangeable tips in your kit) — one of those tips might have bare wire leads

Connect:
- **+6V** (red wire from adapter) → breadboard **red (+) rail**
- **GND** (black wire from adapter) → breadboard **blue (-) rail**

⚠️ **Don't plug the adapter into the wall yet!** Wire everything first, double-check, then apply power last.

**Step 3A-2: Connect both servos to the breadboard.**

For EACH servo (pan and tilt):

| Servo Wire | Connects To |
|-----------|-------------|
| Red (+V) | Breadboard red (+) rail |
| Brown (GND) | Breadboard blue (-) rail |
| Orange (SIG) | Leave free for now (goes to Arduino) |

You'll need to use dupont jumper wires to bridge from the servo's 3-pin connector to the breadboard. **Male-to-male** wires work for this: push one end into the servo connector pin, the other into the breadboard.

**Step 3A-3: Connect the common ground.**

Use a dupont jumper wire to connect:
- Breadboard **blue (-) rail** → Arduino **GND pin**

This is the common ground! It connects the servo power supply's ground to the Arduino's ground.

```
BREADBOARD WIRING DIAGRAM

         (+) rail ←── 6V adapter (+)
         (─) rail ←── 6V adapter (-)
              │
              ├── Pan servo RED wire
              ├── Tilt servo RED wire    (both to + rail)
              │
         (─) rail
              ├── Pan servo BROWN wire
              ├── Tilt servo BROWN wire  (both to - rail)
              │
              └── jumper wire ──→ Arduino GND pin
                  (COMMON GROUND)
```

**Step 3A-4: Connect signal wires to Arduino.**

Use dupont jumper wires:

| From | To | Wire Color (suggestion) |
|------|----|------------------------|
| Pan servo ORANGE wire | Arduino pin **D9** | Use a yellow/orange dupont wire |
| Tilt servo ORANGE wire | Arduino pin **D10** | Use a different color so you can tell them apart |

### Option B: Without a Breadboard (twisted wires)

If you want to start right now and don't have a breadboard:

1. Take 3 male-to-male dupont wires
2. Twist/connect the **red** wires from both servos together, plus the +6V from your power adapter → all three reds joined
3. Twist/connect the **brown** wires from both servos together, plus the GND from your power adapter, plus a wire going to Arduino GND → all four browns/blacks joined
4. You can use electrical tape to insulate the twisted connections

This works but is fragile. Get a breadboard when you can.

---

## STEP 4: Connect the Arduino

**4-1.** Plug the USB-C cable into the Arduino and into your laptop. The Arduino should power on (you'll see an LED light up).

**4-2.** Double-check signal wire connections:
- Pin D9 → Pan servo signal (orange)
- Pin D10 → Tilt servo signal (orange)
- GND pin → Common ground (to breadboard - rail or servo ground junction)

**4-3.** Do NOT connect anything to the Arduino's 5V pin. The servos get their power from the external 6V supply, not from the Arduino.

---

## STEP 5: Flash and Test

**5-1. Install Arduino IDE** if you haven't already:
- Download from https://www.arduino.cc/en/software
- Install the **Arduino UNO R4 Boards** package: Tools → Board → Boards Manager → search "UNO R4" → Install

**5-2. Open the firmware:**
- Open `firmware/servo_controller/servo_controller.ino` in Arduino IDE

**5-3. Select your board and port:**
- Tools → Board → Arduino UNO R4 Boards → **Arduino UNO R4 WiFi**
- Tools → Port → select the one that says "Arduino UNO R4 WiFi" (on Mac it'll be something like `/dev/cu.usbmodemXXXX`)

**5-4. Upload:**
- Click the Upload button (→ arrow)
- Wait for "Done uploading"

**5-5. Open Serial Monitor:**
- Tools → Serial Monitor
- Set baud rate to **115200** (bottom-right dropdown)
- Set line ending to **Newline** (dropdown next to baud rate)

**5-6. NOW plug in the 6V power adapter.**

You should see the startup message in the serial monitor:
```
=== Rocket Tracker Servo Controller ===
Type '?' for help
Ready at 90,90
```

The servos should twitch slightly as they move to the 90° home position.

**5-7. Test commands:**

Type each of these in the Serial Monitor input box and press Enter:

```
H          → Both servos center to 90° (you should see/hear them move)
P30        → Pan servo rotates to 30°
P150       → Pan servo rotates to 150°
T30        → Tilt servo moves to 30°
T150       → Tilt servo moves to 150°
B90,90     → Both return to center
S          → Start sweep test (both slowly sweep back and forth)
S          → Stop sweep test
```

---

## STEP 6: Test from Python

Close the Arduino IDE Serial Monitor first (only one program can use the serial port at a time).

```bash
cd rocket-tracker
source venv/bin/activate
pip install pyserial
python control/servo_serial.py --demo
```

You should see the gimbal cycle through a sequence: center, look left, look right, look up, look down, and back to center.

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Servos don't move at all | No power to servos | Check 6V adapter is plugged in, check red/brown wires on breadboard |
| Servos jitter/twitch randomly | Missing common ground | Add jumper wire from breadboard GND rail to Arduino GND |
| Arduino resets when servos move | Servos drawing power from Arduino | Make sure servo red wires go to 6V adapter, NOT to Arduino 5V pin |
| Only one servo works | Signal wire on wrong pin | Check that pan=D9, tilt=D10 |
| Serial monitor shows garbage | Wrong baud rate | Set to 115200 |
| Python can't find serial port | Wrong port or Serial Monitor still open | Close Serial Monitor, try `ls /dev/tty.usb*` to find the port |
| Servo moves to wrong angle | Servo horn mounted at wrong starting position | Remove horn, center servo with `H` command, reattach horn at 90° |

---

## What's Next

Once you've verified both servos respond correctly to commands:
1. Mount the webcam on the tilt platform (zip ties, velcro, or 3D printed bracket)
2. We'll write the Phase 3 code that connects the tracking script to the gimbal — the camera detects the object, Kalman filter predicts its position, and the servo controller points the gimbal to follow it
