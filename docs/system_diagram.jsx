import { useState } from "react";

const STAGES = [
  {
    id: "capture",
    label: "CAPTURE",
    icon: "📷",
    hardware: "Logitech C920",
    location: "Laptop (USB)",
    color: "#2dd4bf",
    summary: "Grab a raw frame from the webcam",
    details: {
      what: "OpenCV's VideoCapture reads one BGR frame from the USB webcam at ~30fps. Each frame is a 640×480 grid of pixels, each with Blue, Green, Red values (0-255).",
      data_in: "USB video stream (MJPEG compressed)",
      data_out: "640×480×3 numpy array (BGR, uint8)",
      code: "ret, frame = cap.read()",
      gotchas: [
        "C920 defaults to 30fps — USB bandwidth limits higher rates",
        "Auto-exposure changes brightness frame-to-frame → HSV values shift",
        "Rolling shutter: fast objects smear diagonally across the frame"
      ]
    }
  },
  {
    id: "convert",
    label: "COLOR CONVERT",
    icon: "🎨",
    hardware: "CPU",
    location: "Laptop (OpenCV)",
    color: "#a78bfa",
    summary: "BGR → HSV color space",
    details: {
      what: "Convert from BGR to HSV (Hue, Saturation, Value). HSV separates color identity (Hue) from brightness (Value), making detection robust to lighting changes. A red object stays at hue ~0-10 whether it's in shadow or sunlight.",
      data_in: "640×480×3 BGR array",
      data_out: "640×480×3 HSV array",
      code: "hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)",
      gotchas: [
        "OpenCV uses H:0-179, S:0-255, V:0-255 (not 0-360)",
        "Red wraps around: hue 0-10 AND 160-179 are both 'red'",
        "White has low saturation — high S threshold rejects white backgrounds"
      ]
    }
  },
  {
    id: "mask",
    label: "THRESHOLD",
    icon: "⬛",
    hardware: "CPU",
    location: "Laptop (OpenCV)",
    color: "#f472b6",
    summary: "Create binary mask of red pixels",
    details: {
      what: "Apply HSV range filter: pixels within [H:0-10, S:180-255, V:50-255] become white (255), everything else becomes black (0). Then morphological erode removes tiny noise specks, dilate fills small holes in the detection blob.",
      data_in: "640×480×3 HSV array + threshold values",
      data_out: "640×480 binary mask (single channel, 0 or 255)",
      code: "mask = cv2.inRange(hsv, lower, upper)\nmask = cv2.erode(mask, None, iterations=2)\nmask = cv2.dilate(mask, None, iterations=2)",
      gotchas: [
        "Hard threshold edge → pixels near boundary flicker between frames → detection noise",
        "Erode/dilate iterations control noise vs. detection size tradeoff",
        "Calibration (press 'c') adjusts these thresholds interactively"
      ]
    }
  },
  {
    id: "detect",
    label: "DETECT",
    icon: "🎯",
    hardware: "CPU",
    location: "Laptop (OpenCV)",
    color: "#fb923c",
    summary: "Find the object's (x, y) position",
    details: {
      what: "Find contours (outlines) in the binary mask. Pick the largest contour by area (rejects small noise blobs). Compute the minimum enclosing circle — its center is the detected position in pixel coordinates.",
      data_in: "640×480 binary mask",
      data_out: "detection: (x, y, radius) in pixels, or None",
      code: "contours, _ = cv2.findContours(mask, ...)\nc = max(contours, key=cv2.contourArea)\n((x, y), radius) = cv2.minEnclosingCircle(c)",
      gotchas: [
        "Minimum area filter (radius > 10) rejects tiny noise contours",
        "Centroid jitters by several pixels even on a stationary object",
        "If two red objects exist, we track the biggest one only"
      ]
    }
  },
  {
    id: "kalman",
    label: "KALMAN FILTER",
    icon: "📈",
    hardware: "CPU",
    location: "Laptop (filterpy)",
    color: "#60a5fa",
    summary: "Predict position when detection is lost",
    details: {
      what: "A constant-velocity Kalman filter maintains state [x, y, vx, vy]. Each frame: PREDICT (extrapolate using velocity), then UPDATE (blend prediction with measurement). When detection is lost, it coasts on predicted velocity for up to 30 frames.",
      data_in: "detection (x, y) or None + time delta",
      data_out: "kalman_pos: (x, y) — smoothed or predicted position",
      code: "kf.predict()\nif detection:\n    kf.update(measurement)\n    pid_target = detection  # Use raw, not Kalman!\nelse:\n    pid_target = (kf.x[0], kf.x[1])  # Coast",
      gotchas: [
        "PID uses RAW measurement, not Kalman output (Kalman velocity is wrong in closed-loop)",
        "Kalman only feeds PID during coasting (no detection)",
        "In closed-loop, apparent motion = object motion + gimbal motion — Kalman can't separate these",
        "Future: add gravity term for ballistic rocket trajectory prediction"
      ]
    }
  },
  {
    id: "pid",
    label: "PID CONTROLLER",
    icon: "🎛️",
    hardware: "CPU",
    location: "Laptop (Python)",
    color: "#f43f5e",
    summary: "Convert pixel error → servo angle adjustment",
    details: {
      what: "Computes how many degrees to move each servo. Error = object position minus frame center (pixels). Adjustment = P × error, clamped to ±1.5°/frame. Dead zone: errors < 15px are ignored to prevent chasing noise.",
      data_in: "pid_target (x, y) in pixels + frame center (320, 240)",
      data_out: "pan_adjustment, tilt_adjustment (degrees, float)",
      code: "error_x = pid_target[0] - cx  # pixels\nif abs(error_x) < 15: error_x = 0  # dead zone\npan_adj = PAN_INVERT * P * error_x  # degrees\npan_adj = clamp(pan_adj, -1.5, +1.5)",
      gotchas: [
        "P-only control: I caused windup/circling, D amplified noise",
        "Output limit (1.5°) prevents saturation — without it, adj hits ±5° every frame",
        "Dead zone (15px) breaks the servo-vibration → detection-jitter feedback loop",
        "Direction inversion (PAN_INVERT, TILT_INVERT) depends on physical servo mounting"
      ]
    }
  },
  {
    id: "serial",
    label: "SERIAL TX",
    icon: "🔌",
    hardware: "USB cable",
    location: "Laptop → Arduino",
    color: "#84cc16",
    summary: "Send angle command over USB serial",
    details: {
      what: "Accumulate PID adjustments into running servo angles. Format as 'B<pan>,<tilt>' string command. Send over USB serial at 115200 baud. Rate-limited to ~30Hz to avoid flooding the Arduino's serial buffer.",
      data_in: "pan_angle, tilt_angle (degrees, float)",
      data_out: "Serial bytes: e.g. 'B92,85\\n'",
      code: "pan_angle += pan_adjustment\npan_angle = clamp(pan_angle, 10, 170)\ngimbal.send_angles(pan_angle, tilt_angle)",
      gotchas: [
        "write_timeout=0.1 prevents blocking if Arduino buffer is full",
        "reset_input_buffer() drains Arduino responses to prevent backpressure",
        "2-second delay after connection for Arduino bootloader reset",
        "UNO R4 WiFi: while(!Serial) needs 3-second timeout for pyserial compatibility"
      ]
    }
  },
  {
    id: "firmware",
    label: "FIRMWARE",
    icon: "⚡",
    hardware: "Arduino UNO R4 WiFi",
    location: "Arduino (C++)",
    color: "#e879f9",
    summary: "Parse commands → drive servo PWM",
    details: {
      what: "Arduino receives serial command, parses the target angles, and smoothly interpolates the servos toward the target in 2° steps every 5ms. Uses the Arduino Servo library to generate 50Hz PWM signals on pins D9 (pan) and D10 (tilt).",
      data_in: "Serial command string: 'B92,85'",
      data_out: "PWM signals on D9, D10 (1000-2000μs pulses at 50Hz)",
      code: "// Parse 'B92,85' → pan=92, tilt=85\n// Interpolate in 2° steps\npanServo.write(currentPan);\ntiltServo.write(currentTilt);",
      gotchas: [
        "Smooth interpolation prevents sudden jerks that shake the camera",
        "Angle clamped to 10-170° to protect servos from mechanical limits",
        "Servo library uses hardware timers — D9/D10 are hardware PWM pins",
        "Step delay (5ms) means full 180° sweep takes ~450ms"
      ]
    }
  },
  {
    id: "servos",
    label: "SERVOS",
    icon: "⚙️",
    hardware: "2x MG995 servos",
    location: "Gimbal bracket",
    color: "#fbbf24",
    summary: "Physically rotate the camera",
    details: {
      what: "PWM pulse width encodes the target angle. 1000μs → 0°, 1500μs → 90°, 2000μs → 180°. The servo's internal motor + gearbox + potentiometer feedback loop drives the output shaft to match. Pan servo rotates horizontally, tilt servo rotates vertically.",
      data_in: "PWM signal (50Hz, 1000-2000μs pulse width)",
      data_out: "Physical rotation of camera platform",
      code: "// No code — this is mechanical\n// Pulse width → angle → gear rotation",
      gotchas: [
        "MG995: ~400°/sec max speed, ~1° accuracy, metal gears",
        "Gear backlash: ~1-2° of 'slop' when reversing direction",
        "Servos powered by separate 6V supply (not Arduino 5V)",
        "Common ground between 6V supply and Arduino is mandatory",
        "Holding torque vibration shakes the camera → detection noise"
      ]
    }
  }
];

const FEEDBACK_LOOPS = [
  {
    id: "visual",
    label: "Visual Feedback Loop (the main control loop)",
    color: "#2dd4bf",
    icon: "🔄",
    description: "Camera sees object → PID moves gimbal → camera view changes → object appears closer to center. This is NEGATIVE feedback: error produces a correction that reduces the error. This is what makes tracking work.",
    path: "servos → capture (camera moves, sees new frame)"
  },
  {
    id: "mechanical",
    label: "Mechanical Coupling (parasitic feedback)",
    color: "#fbbf24",
    icon: "📳",
    description: "Servo moves → camera shakes → detection jitters a few pixels → PID reacts → servo moves again. This is the noise loop that caused the 'circling' behavior. The 15px dead zone breaks this loop by ignoring tiny jitter.",
    path: "servos → capture → detect → pid (through vibration)"
  },
  {
    id: "kalman_coast",
    label: "Kalman Coasting (open-loop fallback)",
    color: "#60a5fa",
    icon: "〰️",
    description: "When detection is lost, the Kalman filter predicts where the object should be based on its last known velocity. PID tracks this ghost prediction for up to 30 frames (~1 second). If detection returns, it snaps back to the real measurement.",
    path: "kalman → pid (when detection = None)"
  }
];

function Stage({ stage, isSelected, onClick, index, total }) {
  const isEven = index % 2 === 0;
  return (
    <button
      onClick={onClick}
      style={{
        display: "flex",
        alignItems: "center",
        gap: "10px",
        padding: "10px 14px",
        borderRadius: "10px",
        border: isSelected ? `2px solid ${stage.color}` : "2px solid transparent",
        background: isSelected ? `${stage.color}11` : "#1a1a2e",
        cursor: "pointer",
        width: "100%",
        textAlign: "left",
        transition: "all 0.15s ease",
        position: "relative"
      }}
    >
      <div style={{
        width: 40, height: 40, borderRadius: 8,
        background: `${stage.color}22`,
        display: "flex", alignItems: "center", justifyContent: "center",
        fontSize: 20, flexShrink: 0
      }}>
        {stage.icon}
      </div>
      <div style={{ flex: 1, minWidth: 0 }}>
        <div style={{
          fontSize: 11, fontWeight: 700, letterSpacing: "0.08em",
          color: stage.color, fontFamily: "'JetBrains Mono', monospace"
        }}>
          {index + 1}. {stage.label}
        </div>
        <div style={{ fontSize: 12, color: "#94a3b8", marginTop: 2, lineHeight: 1.3 }}>
          {stage.summary}
        </div>
      </div>
      {index < total - 1 && (
        <div style={{
          position: "absolute", bottom: -14, left: "50%",
          color: "#334155", fontSize: 16, transform: "translateX(-50%)",
          fontFamily: "monospace"
        }}>▼</div>
      )}
    </button>
  );
}

function DetailPanel({ stage }) {
  if (!stage) return (
    <div style={{
      display: "flex", alignItems: "center", justifyContent: "center",
      height: "100%", color: "#475569", fontSize: 14, fontStyle: "italic",
      padding: 40, textAlign: "center", lineHeight: 1.6
    }}>
      ← Click a stage to see what happens at each step, what data flows through, and what can go wrong.
    </div>
  );

  const d = stage.details;

  return (
    <div style={{ padding: "20px 24px", overflowY: "auto", maxHeight: "100%" }}>
      <div style={{
        display: "flex", alignItems: "center", gap: 10, marginBottom: 16
      }}>
        <span style={{ fontSize: 28 }}>{stage.icon}</span>
        <div>
          <div style={{
            fontSize: 16, fontWeight: 700, color: stage.color,
            fontFamily: "'JetBrains Mono', monospace"
          }}>{stage.label}</div>
          <div style={{ fontSize: 12, color: "#64748b" }}>
            {stage.hardware} · {stage.location}
          </div>
        </div>
      </div>

      <Section title="What happens" color={stage.color}>
        <p style={{ fontSize: 13, color: "#cbd5e1", lineHeight: 1.65, margin: 0 }}>{d.what}</p>
      </Section>

      <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 12, margin: "12px 0" }}>
        <DataBox label="DATA IN" value={d.data_in} color="#94a3b8" />
        <DataBox label="DATA OUT" value={d.data_out} color={stage.color} />
      </div>

      <Section title="Code" color={stage.color}>
        <pre style={{
          fontSize: 11.5, color: "#e2e8f0", background: "#0f0f1a",
          padding: 12, borderRadius: 6, overflowX: "auto", margin: 0,
          fontFamily: "'JetBrains Mono', monospace", lineHeight: 1.5,
          border: "1px solid #1e293b"
        }}>{d.code}</pre>
      </Section>

      <Section title="Watch out for" color={stage.color}>
        {d.gotchas.map((g, i) => (
          <div key={i} style={{
            fontSize: 12, color: "#94a3b8", lineHeight: 1.5,
            padding: "4px 0 4px 16px", position: "relative"
          }}>
            <span style={{
              position: "absolute", left: 0, color: "#475569"
            }}>⚠</span>
            {g}
          </div>
        ))}
      </Section>
    </div>
  );
}

function Section({ title, color, children }) {
  return (
    <div style={{ marginBottom: 14 }}>
      <div style={{
        fontSize: 10, fontWeight: 700, letterSpacing: "0.1em",
        color: color, marginBottom: 6, fontFamily: "'JetBrains Mono', monospace",
        textTransform: "uppercase"
      }}>{title}</div>
      {children}
    </div>
  );
}

function DataBox({ label, value, color }) {
  return (
    <div style={{
      background: "#0f0f1a", borderRadius: 6, padding: "8px 10px",
      border: "1px solid #1e293b"
    }}>
      <div style={{
        fontSize: 9, fontWeight: 700, letterSpacing: "0.1em",
        color: "#475569", marginBottom: 4, fontFamily: "'JetBrains Mono', monospace"
      }}>{label}</div>
      <div style={{
        fontSize: 11, color: color, fontFamily: "'JetBrains Mono', monospace",
        lineHeight: 1.4
      }}>{value}</div>
    </div>
  );
}

function FeedbackSection({ selectedLoop, onSelect }) {
  return (
    <div style={{
      background: "#12121f", borderRadius: 12, border: "1px solid #1e293b",
      padding: 16, marginTop: 16
    }}>
      <div style={{
        fontSize: 11, fontWeight: 700, letterSpacing: "0.1em", color: "#64748b",
        marginBottom: 12, fontFamily: "'JetBrains Mono', monospace"
      }}>
        FEEDBACK LOOPS
      </div>
      <div style={{ display: "flex", gap: 8, flexWrap: "wrap", marginBottom: selectedLoop ? 12 : 0 }}>
        {FEEDBACK_LOOPS.map(loop => (
          <button
            key={loop.id}
            onClick={() => onSelect(selectedLoop?.id === loop.id ? null : loop)}
            style={{
              padding: "6px 12px", borderRadius: 6, fontSize: 12,
              border: selectedLoop?.id === loop.id
                ? `1.5px solid ${loop.color}` : "1.5px solid #1e293b",
              background: selectedLoop?.id === loop.id ? `${loop.color}15` : "#1a1a2e",
              color: selectedLoop?.id === loop.id ? loop.color : "#94a3b8",
              cursor: "pointer", fontFamily: "'JetBrains Mono', monospace",
              transition: "all 0.15s ease"
            }}
          >
            {loop.icon} {loop.label.split("(")[0].trim()}
          </button>
        ))}
      </div>
      {selectedLoop && (
        <div style={{
          fontSize: 13, color: "#cbd5e1", lineHeight: 1.65,
          padding: 12, background: "#0f0f1a", borderRadius: 8,
          border: `1px solid ${selectedLoop.color}22`
        }}>
          <div style={{
            fontSize: 11, color: selectedLoop.color, fontWeight: 700,
            marginBottom: 6, fontFamily: "'JetBrains Mono', monospace"
          }}>{selectedLoop.label}</div>
          {selectedLoop.description}
          <div style={{
            marginTop: 8, fontSize: 11, color: "#64748b",
            fontFamily: "'JetBrains Mono', monospace"
          }}>
            Path: {selectedLoop.path}
          </div>
        </div>
      )}
    </div>
  );
}

export default function SystemDiagram() {
  const [selected, setSelected] = useState(null);
  const [selectedLoop, setSelectedLoop] = useState(null);

  return (
    <div style={{
      minHeight: "100vh", background: "#0f0f1a", color: "#e2e8f0",
      fontFamily: "'Inter', -apple-system, sans-serif",
      padding: "24px 20px"
    }}>
      <link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;700&family=Inter:wght@400;500;600;700&display=swap" rel="stylesheet" />

      <div style={{ maxWidth: 900, margin: "0 auto" }}>
        <div style={{ marginBottom: 20 }}>
          <h1 style={{
            fontSize: 22, fontWeight: 700, margin: 0, color: "#f8fafc",
            fontFamily: "'JetBrains Mono', monospace"
          }}>
            🚀 Rocket Tracker — System Architecture
          </h1>
          <p style={{ fontSize: 13, color: "#64748b", margin: "6px 0 0" }}>
            Each frame: capture → detect → predict → correct → move. Click any stage to explore.
          </p>
        </div>

        <div style={{
          display: "grid",
          gridTemplateColumns: "280px 1fr",
          gap: 16,
          alignItems: "start"
        }}>
          {/* Pipeline column */}
          <div style={{ display: "flex", flexDirection: "column", gap: 18 }}>
            {STAGES.map((stage, i) => (
              <Stage
                key={stage.id}
                stage={stage}
                index={i}
                total={STAGES.length}
                isSelected={selected?.id === stage.id}
                onClick={() => setSelected(selected?.id === stage.id ? null : stage)}
              />
            ))}
            {/* Feedback arrow */}
            <div style={{
              textAlign: "center", padding: "4px 0",
              fontSize: 11, color: "#2dd4bf",
              fontFamily: "'JetBrains Mono', monospace",
              borderTop: "1px dashed #2dd4bf33",
              borderRadius: 0
            }}>
              ↑ camera sees result → loop repeats at ~30fps ↑
            </div>
          </div>

          {/* Detail panel */}
          <div style={{
            background: "#12121f", borderRadius: 12,
            border: "1px solid #1e293b",
            minHeight: 500,
            position: "sticky", top: 20
          }}>
            <DetailPanel stage={selected} />
          </div>
        </div>

        <FeedbackSection selectedLoop={selectedLoop} onSelect={setSelectedLoop} />

        {/* Legend */}
        <div style={{
          marginTop: 16, padding: 14,
          background: "#12121f", borderRadius: 12, border: "1px solid #1e293b",
          display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: 12
        }}>
          <LegendItem
            label="Currently active"
            items={["Capture → Detect → PID → Serial → Servos"]}
            color="#2dd4bf"
          />
          <LegendItem
            label="Active but bypassed for PID"
            items={["Kalman (runs parallel, feeds PID only when coasting)"]}
            color="#60a5fa"
          />
          <LegendItem
            label="Not yet used"
            items={["I term (disabled)", "D term (disabled)", "Laser module (Phase 4)"]}
            color="#475569"
          />
        </div>
      </div>
    </div>
  );
}

function LegendItem({ label, items, color }) {
  return (
    <div>
      <div style={{
        fontSize: 9, fontWeight: 700, letterSpacing: "0.1em",
        color, marginBottom: 4, fontFamily: "'JetBrains Mono', monospace",
        textTransform: "uppercase"
      }}>{label}</div>
      {items.map((item, i) => (
        <div key={i} style={{ fontSize: 11, color: "#94a3b8", lineHeight: 1.5 }}>{item}</div>
      ))}
    </div>
  );
}
