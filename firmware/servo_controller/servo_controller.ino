/*
 * ROCKET TRACKER — Phase 2: Pan/Tilt Servo Controller
 * =====================================================
 *
 * This firmware runs on the Arduino UNO R4 WiFi and controls two MG995
 * servos (pan + tilt) based on commands received over USB serial.
 *
 * WIRING:
 *   Pan servo  signal → Pin 9
 *   Tilt servo signal → Pin 10
 *   Servo power       → External 6V supply (NOT from Arduino!)
 *   Common ground     → Arduino GND ↔ Servo power supply GND
 *
 * SERIAL PROTOCOL:
 *   Commands are simple text lines ending with newline (\n):
 *
 *   "P<angle>"       — Set pan angle  (e.g., "P90" → pan to 90°)
 *   "T<angle>"       — Set tilt angle (e.g., "T45" → tilt to 45°)
 *   "B<pan>,<tilt>"  — Set both at once (e.g., "B90,45")
 *   "C"              — Report current angles (response: "C<pan>,<tilt>")
 *   "H"              — Home position (center both servos to 90°)
 *   "S"              — Sweep test (pan and tilt cycle through range)
 *   "?"              — Print help
 *
 *   Angles are clamped to [MIN_ANGLE, MAX_ANGLE] for safety.
 *
 * TESTING WITHOUT PYTHON:
 *   1. Upload this sketch to your Arduino
 *   2. Open Serial Monitor (Tools → Serial Monitor)
 *   3. Set baud rate to 115200, line ending to "Newline"
 *   4. Type "H" to center servos
 *   5. Type "S" to run sweep test
 *   6. Type "P45" or "T120" to move individual servos
 *
 * LEARNING NOTES:
 *   Servos are controlled via PWM (Pulse Width Modulation). The Arduino
 *   Servo library handles the details, but under the hood:
 *     - A pulse is sent every 20ms (50 Hz)
 *     - Pulse width of ~1000μs = 0°
 *     - Pulse width of ~1500μs = 90° (center)
 *     - Pulse width of ~2000μs = 180°
 *   The servo's internal circuitry reads this pulse width and drives the
 *   motor to the corresponding angle. This is why we need hardware PWM
 *   pins (9 and 10 on the UNO) — they can generate precise pulse timing.
 */

#include <Servo.h>

// ─── PIN CONFIGURATION ──────────────────────────────────────────
#define PAN_PIN   9
#define TILT_PIN  10

// ─── ANGLE LIMITS ───────────────────────────────────────────────
// Safety clamps — adjust these based on your physical setup to
// prevent the gimbal from hitting its own frame or wires.
#define PAN_MIN   10
#define PAN_MAX   170
#define TILT_MIN  10
#define TILT_MAX  170

// Default (home) position — center of range
#define PAN_HOME  90
#define TILT_HOME 90

// ─── SMOOTHING ──────────────────────────────────────────────────
// Instead of jumping instantly to a target angle (which causes
// jerky motion and high current spikes), we interpolate smoothly.
// STEP_DELAY_MS controls how fast the servo moves (smaller = faster).
// STEP_SIZE controls how far it moves per step (larger = faster but jerkier).
#define STEP_DELAY_MS  5
#define STEP_SIZE      2

// ─── SERIAL ─────────────────────────────────────────────────────
#define BAUD_RATE  115200
#define BUF_SIZE   32

// ─── GLOBALS ────────────────────────────────────────────────────
Servo panServo;
Servo tiltServo;

int panAngle  = PAN_HOME;
int tiltAngle = TILT_HOME;

int panTarget  = PAN_HOME;
int tiltTarget = TILT_HOME;

char inputBuffer[BUF_SIZE];
int bufferIndex = 0;

bool sweepMode = false;
int sweepPanDir = 1;
int sweepTiltDir = 1;

// ─── SETUP ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ; // Wait for USB serial port to connect (needed on UNO R4 WiFi)
  }
  delay(200); // Extra settle time

  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  // Start at home position
  panServo.write(PAN_HOME);
  tiltServo.write(TILT_HOME);
  panAngle = PAN_HOME;
  tiltAngle = TILT_HOME;
  panTarget = PAN_HOME;
  tiltTarget = TILT_HOME;

  delay(500); // Let servos reach home position

  Serial.println("=== Rocket Tracker Servo Controller ===");
  Serial.println("Type '?' for help");
  Serial.print("Ready at ");
  Serial.print(PAN_HOME);
  Serial.print(",");
  Serial.println(TILT_HOME);
}

// ─── MAIN LOOP ──────────────────────────────────────────────────
void loop() {
  // Check for serial commands
  readSerial();

  // If sweep test is active, run it
  if (sweepMode) {
    runSweep();
  }

  // Smoothly move servos toward their targets
  updateServos();
}

// ─── SERIAL READING ─────────────────────────────────────────────
void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        inputBuffer[bufferIndex] = '\0'; // Null-terminate
        processCommand(inputBuffer);
        bufferIndex = 0;
      }
    } else if (bufferIndex < BUF_SIZE - 1) {
      inputBuffer[bufferIndex++] = c;
    }
  }
}

// ─── COMMAND PROCESSING ─────────────────────────────────────────
void processCommand(char* cmd) {
  switch (cmd[0]) {

    case 'P': case 'p': {
      // Pan to angle: "P90"
      int angle = atoi(&cmd[1]);
      panTarget = constrain(angle, PAN_MIN, PAN_MAX);
      sweepMode = false;
      Serial.print("PAN → ");
      Serial.println(panTarget);
      break;
    }

    case 'T': case 't': {
      // Tilt to angle: "T45"
      int angle = atoi(&cmd[1]);
      tiltTarget = constrain(angle, TILT_MIN, TILT_MAX);
      sweepMode = false;
      Serial.print("TILT → ");
      Serial.println(tiltTarget);
      break;
    }

    case 'B': case 'b': {
      // Both: "B90,45"
      int pan, tilt;
      if (sscanf(&cmd[1], "%d,%d", &pan, &tilt) == 2) {
        panTarget  = constrain(pan, PAN_MIN, PAN_MAX);
        tiltTarget = constrain(tilt, TILT_MIN, TILT_MAX);
        sweepMode = false;
        Serial.print("BOTH → ");
        Serial.print(panTarget);
        Serial.print(",");
        Serial.println(tiltTarget);
      } else {
        Serial.println("ERR: use B<pan>,<tilt>");
      }
      break;
    }

    case 'C': case 'c': {
      // Report current position
      Serial.print("C");
      Serial.print(panAngle);
      Serial.print(",");
      Serial.println(tiltAngle);
      break;
    }

    case 'H': case 'h': {
      // Home
      panTarget  = PAN_HOME;
      tiltTarget = TILT_HOME;
      sweepMode = false;
      Serial.println("HOME → 90,90");
      break;
    }

    case 'S': case 's': {
      // Toggle sweep test
      sweepMode = !sweepMode;
      if (sweepMode) {
        Serial.println("SWEEP ON");
      } else {
        Serial.println("SWEEP OFF");
      }
      break;
    }

    case '?': {
      printHelp();
      break;
    }

    default:
      Serial.print("ERR: unknown command '");
      Serial.print(cmd[0]);
      Serial.println("' — type '?' for help");
      break;
  }
}

// ─── SMOOTH SERVO UPDATE ────────────────────────────────────────
/*
 * Instead of jumping to the target angle instantly, we step toward
 * it gradually. This:
 *   1. Reduces mechanical stress on the gears
 *   2. Lowers current spikes (less brownout risk)
 *   3. Looks smoother
 *
 * In Phase 3, we'll replace this with a proper PID controller that
 * receives continuous position updates from the tracking loop.
 * For now, simple interpolation is fine for testing.
 */
void updateServos() {
  static unsigned long lastStep = 0;
  unsigned long now = millis();

  if (now - lastStep < STEP_DELAY_MS) return;
  lastStep = now;

  // Step pan toward target
  if (panAngle < panTarget) {
    panAngle = min(panAngle + STEP_SIZE, panTarget);
  } else if (panAngle > panTarget) {
    panAngle = max(panAngle - STEP_SIZE, panTarget);
  }

  // Step tilt toward target
  if (tiltAngle < tiltTarget) {
    tiltAngle = min(tiltAngle + STEP_SIZE, tiltTarget);
  } else if (tiltAngle > tiltTarget) {
    tiltAngle = max(tiltAngle - STEP_SIZE, tiltTarget);
  }

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
}

// ─── SWEEP TEST ─────────────────────────────────────────────────
/*
 * Slowly sweeps both servos back and forth across their full range.
 * Useful for:
 *   - Verifying both servos work and move freely
 *   - Checking that the bracket doesn't bind or hit anything
 *   - Making sure the power supply can handle both moving at once
 */
void runSweep() {
  static unsigned long lastSweep = 0;
  unsigned long now = millis();

  if (now - lastSweep < 30) return;
  lastSweep = now;

  panTarget += sweepPanDir;
  if (panTarget >= PAN_MAX) { panTarget = PAN_MAX; sweepPanDir = -1; }
  if (panTarget <= PAN_MIN) { panTarget = PAN_MIN; sweepPanDir = 1; }

  tiltTarget += sweepTiltDir;
  if (tiltTarget >= TILT_MAX) { tiltTarget = TILT_MAX; sweepTiltDir = -1; }
  if (tiltTarget <= TILT_MIN) { tiltTarget = TILT_MIN; sweepTiltDir = 1; }
}

// ─── HELP TEXT ──────────────────────────────────────────────────
void printHelp() {
  Serial.println("=== Servo Controller Commands ===");
  Serial.println("  P<angle>       Pan to angle (10-170)");
  Serial.println("  T<angle>       Tilt to angle (10-170)");
  Serial.println("  B<pan>,<tilt>  Set both angles");
  Serial.println("  C              Report current angles");
  Serial.println("  H              Home (center to 90,90)");
  Serial.println("  S              Toggle sweep test");
  Serial.println("  ?              Show this help");
  Serial.println("================================");
}
