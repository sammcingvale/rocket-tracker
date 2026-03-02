import { useState, useRef, useEffect, useCallback } from "react";

// --- SIMULATION ENGINE ---
class PIDSim {
  constructor(kp, ki, kd) {
    this.kp = kp; this.ki = ki; this.kd = kd;
    this.integral = 0; this.prevError = 0;
    this.integralLimit = 500;
  }
  update(error, dt) {
    this.integral += error * dt;
    this.integral = Math.max(-this.integralLimit, Math.min(this.integralLimit, this.integral));
    const derivative = dt > 0 ? (error - this.prevError) / dt : 0;
    this.prevError = error;
    return this.kp * error + this.ki * this.integral + this.kd * derivative;
  }
  reset() { this.integral = 0; this.prevError = 0; }
}

class GimbalSim {
  constructor() {
    this.reset();
  }
  reset() {
    // Camera/gimbal state (in pixels, simulating 640x480 view)
    this.gimbalAngleX = 0; this.gimbalAngleY = 0;
    this.targetWorldX = 0; this.targetWorldY = 0;
    this.pidX = new PIDSim(0.015, 0, 0);
    this.pidY = new PIDSim(0.015, 0, 0);
    this.history = [];
    this.tick = 0;
  }
  setGains(p, i, d) {
    this.pidX.kp = p; this.pidX.ki = i; this.pidX.kd = d;
    this.pidY.kp = p; this.pidY.ki = i; this.pidY.kd = d;
  }
  resetPID() { this.pidX.reset(); this.pidY.reset(); }

  step(targetWorldX, targetWorldY, deadZone, outputLimit, dt) {
    this.targetWorldX = targetWorldX;
    this.targetWorldY = targetWorldY;

    // Object position in camera frame = world position - gimbal angle
    // (simplified: 1 degree ≈ 9.14 pixels at 640px / 70° FOV)
    const pixPerDeg = 640 / 70;
    const objInFrameX = (targetWorldX - this.gimbalAngleX) * pixPerDeg;
    const objInFrameY = (targetWorldY - this.gimbalAngleY) * pixPerDeg;

    // Error from center (0,0 = centered)
    let errorX = objInFrameX;
    let errorY = objInFrameY;

    // Dead zone
    if (Math.abs(errorX) < deadZone) errorX = 0;
    if (Math.abs(errorY) < deadZone) errorY = 0;

    // PID
    let adjX = this.pidX.update(errorX, dt);
    let adjY = this.pidY.update(errorY, dt);

    // Output clamp
    adjX = Math.max(-outputLimit, Math.min(outputLimit, adjX));
    adjY = Math.max(-outputLimit, Math.min(outputLimit, adjY));

    // Apply to gimbal (with simulated servo delay — lerp toward target)
    this.gimbalAngleX += adjX;
    this.gimbalAngleY += adjY;

    this.tick++;

    const record = {
      tick: this.tick,
      errorX, errorY,
      rawErrorX: objInFrameX, rawErrorY: objInFrameY,
      adjX, adjY,
      gimbalX: this.gimbalAngleX, gimbalY: this.gimbalAngleY,
      targetX: targetWorldX, targetY: targetWorldY,
      saturatedX: Math.abs(adjX) >= outputLimit - 0.001,
      saturatedY: Math.abs(adjY) >= outputLimit - 0.001,
    };
    this.history.push(record);
    if (this.history.length > 300) this.history.shift();

    return record;
  }
}

// --- MINI GRAPH COMPONENT ---
function MiniGraph({ data, dataKey, label, color, height = 80, showZero = true, maxVal }) {
  const w = 280, h = height;
  if (data.length < 2) return <div style={{ width: w, height: h }} />;

  const values = data.map(d => d[dataKey]);
  const absMax = maxVal || Math.max(Math.abs(Math.min(...values)), Math.abs(Math.max(...values)), 1);

  const points = values.map((v, i) => {
    const x = (i / (data.length - 1)) * w;
    const y = h / 2 - (v / absMax) * (h / 2 - 4);
    return `${x},${y}`;
  }).join(" ");

  const saturated = data.filter(d => d[`saturated${dataKey.includes("X") ? "X" : "Y"}`]).length;
  const satPct = data.length > 0 ? Math.round(saturated / data.length * 100) : 0;

  return (
    <div style={{ position: "relative" }}>
      <div style={{
        fontSize: 10, fontWeight: 700, color: "#64748b", marginBottom: 3,
        fontFamily: "mono", display: "flex", justifyContent: "space-between"
      }}>
        <span style={{ color }}>{label}</span>
        <span style={{ color: "#475569" }}>
          {values.length > 0 ? `${values[values.length-1].toFixed(1)}` : "—"}
          {satPct > 0 && <span style={{ color: "#ef4444", marginLeft: 6 }}>⚠ {satPct}% saturated</span>}
        </span>
      </div>
      <svg width={w} height={h} style={{ background: "#0a0a14", borderRadius: 6, border: "1px solid #1e293b" }}>
        {showZero && <line x1={0} y1={h/2} x2={w} y2={h/2} stroke="#1e293b" strokeWidth={1} />}
        <polyline points={points} fill="none" stroke={color} strokeWidth={1.5} opacity={0.9} />
      </svg>
    </div>
  );
}

// --- CAMERA VIEW ---
function CameraView({ sim, deadZone, onDrag, targetX, targetY, dragging }) {
  const canvasRef = useRef(null);
  const pixPerDeg = 640 / 70;
  const viewW = 320, viewH = 240;
  const scale = 0.5;

  useEffect(() => {
    const ctx = canvasRef.current?.getContext("2d");
    if (!ctx) return;

    ctx.fillStyle = "#111119";
    ctx.fillRect(0, 0, viewW, viewH);

    // Grid
    ctx.strokeStyle = "#1a1a2a";
    ctx.lineWidth = 0.5;
    for (let i = 0; i <= viewW; i += 40) {
      ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, viewH); ctx.stroke();
    }
    for (let i = 0; i <= viewH; i += 40) {
      ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(viewW, i); ctx.stroke();
    }

    // Dead zone
    const dzPx = deadZone * scale;
    ctx.strokeStyle = "#ffffff12";
    ctx.lineWidth = 1;
    ctx.strokeRect(viewW/2 - dzPx, viewH/2 - dzPx, dzPx*2, dzPx*2);

    // Center crosshair
    ctx.strokeStyle = "#334155";
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(viewW/2, 0); ctx.lineTo(viewW/2, viewH); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, viewH/2); ctx.lineTo(viewW, viewH/2); ctx.stroke();

    // Object in frame
    const objFrameX = (targetX - sim.gimbalAngleX) * pixPerDeg * scale;
    const objFrameY = (targetY - sim.gimbalAngleY) * pixPerDeg * scale;
    const screenX = viewW/2 + objFrameX;
    const screenY = viewH/2 + objFrameY;

    // Error line
    if (screenX >= -20 && screenX <= viewW+20 && screenY >= -20 && screenY <= viewH+20) {
      ctx.strokeStyle = "#f43f5e66";
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      ctx.beginPath(); ctx.moveTo(viewW/2, viewH/2); ctx.lineTo(screenX, screenY); ctx.stroke();
      ctx.setLineDash([]);

      // Object dot
      ctx.fillStyle = dragging ? "#ff6b6b" : "#ef4444";
      ctx.beginPath(); ctx.arc(screenX, screenY, 8, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = "#fca5a5";
      ctx.lineWidth = 1.5;
      ctx.stroke();
    } else {
      // Arrow pointing to off-screen object
      const angle = Math.atan2(screenY - viewH/2, screenX - viewW/2);
      const edgeX = viewW/2 + Math.cos(angle) * 100;
      const edgeY = viewH/2 + Math.sin(angle) * 100;
      ctx.fillStyle = "#ef444488";
      ctx.beginPath();
      ctx.moveTo(edgeX + Math.cos(angle)*12, edgeY + Math.sin(angle)*12);
      ctx.lineTo(edgeX + Math.cos(angle+2.5)*8, edgeY + Math.sin(angle+2.5)*8);
      ctx.lineTo(edgeX + Math.cos(angle-2.5)*8, edgeY + Math.sin(angle-2.5)*8);
      ctx.fill();
    }

    // Labels
    ctx.fillStyle = "#475569";
    ctx.font = "10px monospace";
    ctx.fillText("Camera View (what OpenCV sees)", 8, 14);

    const latest = sim.history[sim.history.length - 1];
    if (latest) {
      ctx.fillStyle = "#64748b";
      ctx.fillText(`err: (${latest.rawErrorX.toFixed(0)}, ${latest.rawErrorY.toFixed(0)})px`, 8, viewH - 24);
      ctx.fillText(`adj: (${latest.adjX.toFixed(2)}, ${latest.adjY.toFixed(2)})°`, 8, viewH - 12);
    }
  });

  const handleMouse = useCallback((e) => {
    const rect = canvasRef.current.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;
    // Convert screen position to world angle
    const worldX = sim.gimbalAngleX + (mx - viewW/2) / (pixPerDeg * scale);
    const worldY = sim.gimbalAngleY + (my - viewH/2) / (pixPerDeg * scale);
    onDrag(worldX, worldY);
  }, [sim.gimbalAngleX, sim.gimbalAngleY, onDrag]);

  return (
    <canvas
      ref={canvasRef} width={viewW} height={viewH}
      style={{ borderRadius: 8, cursor: "crosshair", border: "1px solid #1e293b" }}
      onMouseDown={(e) => { onDrag._setDragging(true); handleMouse(e); }}
      onMouseMove={(e) => { if (dragging) handleMouse(e); }}
      onMouseUp={() => onDrag._setDragging(false)}
      onMouseLeave={() => onDrag._setDragging(false)}
    />
  );
}

// --- WORLD VIEW ---
function WorldView({ sim, targetX, targetY }) {
  const canvasRef = useRef(null);
  const viewW = 320, viewH = 240;
  const degScale = 3; // pixels per degree

  useEffect(() => {
    const ctx = canvasRef.current?.getContext("2d");
    if (!ctx) return;

    ctx.fillStyle = "#0a0a14";
    ctx.fillRect(0, 0, viewW, viewH);

    // Grid (every 10 degrees)
    ctx.strokeStyle = "#1a1a2a";
    ctx.lineWidth = 0.5;
    for (let d = -50; d <= 50; d += 10) {
      const x = viewW/2 + d * degScale;
      const y = viewH/2 + d * degScale;
      if (x >= 0 && x <= viewW) {
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, viewH); ctx.stroke();
      }
      if (y >= 0 && y <= viewH) {
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(viewW, y); ctx.stroke();
      }
    }

    // Camera FOV cone
    const fovHalf = 35 * degScale;
    const gimX = viewW/2 + sim.gimbalAngleX * degScale;
    const gimY = viewH/2 + sim.gimbalAngleY * degScale;
    ctx.fillStyle = "#2dd4bf08";
    ctx.strokeStyle = "#2dd4bf22";
    ctx.lineWidth = 1;
    ctx.fillRect(gimX - fovHalf, gimY - fovHalf * (480/640), fovHalf*2, fovHalf*2*(480/640));
    ctx.strokeRect(gimX - fovHalf, gimY - fovHalf * (480/640), fovHalf*2, fovHalf*2*(480/640));

    // Gimbal direction crosshair
    ctx.strokeStyle = "#2dd4bf44";
    ctx.lineWidth = 1;
    ctx.setLineDash([3,3]);
    ctx.beginPath(); ctx.moveTo(gimX-10, gimY); ctx.lineTo(gimX+10, gimY); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(gimX, gimY-10); ctx.lineTo(gimX, gimY+10); ctx.stroke();
    ctx.setLineDash([]);

    // Trail
    const trail = sim.history.slice(-100);
    if (trail.length > 1) {
      ctx.strokeStyle = "#f43f5e22";
      ctx.lineWidth = 1;
      ctx.beginPath();
      trail.forEach((r, i) => {
        const x = viewW/2 + r.targetX * degScale;
        const y = viewH/2 + r.targetY * degScale;
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      });
      ctx.stroke();
    }

    // Target
    const tx = viewW/2 + targetX * degScale;
    const ty = viewH/2 + targetY * degScale;
    ctx.fillStyle = "#ef4444";
    ctx.beginPath(); ctx.arc(tx, ty, 6, 0, Math.PI*2); ctx.fill();

    // Labels
    ctx.fillStyle = "#475569";
    ctx.font = "10px monospace";
    ctx.fillText("World View (degrees)", 8, 14);
    ctx.fillText(`gimbal: (${sim.gimbalAngleX.toFixed(1)}°, ${sim.gimbalAngleY.toFixed(1)}°)`, 8, viewH - 12);
  });

  return (
    <canvas ref={canvasRef} width={viewW} height={viewH}
      style={{ borderRadius: 8, border: "1px solid #1e293b" }} />
  );
}

// --- SLIDER ---
function Slider({ label, value, onChange, min, max, step, color, unit = "" }) {
  return (
    <div style={{ marginBottom: 10 }}>
      <div style={{
        display: "flex", justifyContent: "space-between", marginBottom: 2
      }}>
        <span style={{ fontSize: 11, color: "#94a3b8", fontFamily: "mono" }}>{label}</span>
        <span style={{ fontSize: 11, color, fontFamily: "mono", fontWeight: 700 }}>
          {typeof value === 'number' ? (value < 0.01 && value > 0 ? value.toFixed(4) : value.toFixed(3)) : value}{unit}
        </span>
      </div>
      <input type="range" min={min} max={max} step={step} value={value}
        onChange={e => onChange(parseFloat(e.target.value))}
        style={{ width: "100%", accentColor: color, height: 4 }}
      />
    </div>
  );
}

// --- PRESET BUTTON ---
function Preset({ label, active, onClick, color }) {
  return (
    <button onClick={onClick} style={{
      padding: "5px 10px", borderRadius: 5, fontSize: 11,
      fontFamily: "mono", cursor: "pointer", fontWeight: active ? 700 : 400,
      border: active ? `1.5px solid ${color}` : "1.5px solid #1e293b",
      background: active ? `${color}18` : "#0f0f1a",
      color: active ? color : "#64748b"
    }}>{label}</button>
  );
}

// --- MAIN DASHBOARD ---
export default function PIDDashboard() {
  const simRef = useRef(new GimbalSim());
  const animRef = useRef(null);
  const [, forceRender] = useState(0);

  const [P, setP] = useState(0.015);
  const [I, setI] = useState(0);
  const [D, setD] = useState(0);
  const [deadZone, setDeadZone] = useState(15);
  const [outputLimit, setOutputLimit] = useState(1.5);
  const [running, setRunning] = useState(true);
  const [preset, setPreset] = useState("conservative");

  const [targetX, setTargetX] = useState(8);
  const [targetY, setTargetY] = useState(-5);
  const [dragging, setDragging] = useState(false);

  const targetRef = useRef({ x: 8, y: -5 });

  const onDrag = useCallback((x, y) => {
    setTargetX(x);
    setTargetY(y);
    targetRef.current = { x, y };
  }, []);
  onDrag._setDragging = setDragging;

  const applyPreset = useCallback((name) => {
    setPreset(name);
    const presets = {
      conservative: { p: 0.015, i: 0, d: 0 },
      balanced: { p: 0.03, i: 0, d: 0 },
      aggressive: { p: 0.06, i: 0, d: 0 },
      with_I: { p: 0.015, i: 0.001, d: 0 },
      with_D: { p: 0.015, i: 0, d: 0.008 },
    };
    const v = presets[name];
    if (v) { setP(v.p); setI(v.i); setD(v.d); }
  }, []);

  // Simulation loop
  useEffect(() => {
    const sim = simRef.current;
    let lastTime = performance.now();

    const loop = () => {
      const now = performance.now();
      const dt = Math.min((now - lastTime) / 1000, 0.05);
      lastTime = now;

      if (running) {
        sim.setGains(P, I, D);
        sim.step(targetRef.current.x, targetRef.current.y, deadZone, outputLimit, dt);
        forceRender(n => n + 1);
      }
      animRef.current = requestAnimationFrame(loop);
    };
    animRef.current = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(animRef.current);
  }, [P, I, D, deadZone, outputLimit, running]);

  const sim = simRef.current;
  const history = sim.history;
  const latest = history[history.length - 1];

  const handleReset = () => {
    sim.reset();
    sim.setGains(P, I, D);
    setTargetX(8); setTargetY(-5);
    targetRef.current = { x: 8, y: -5 };
  };

  return (
    <div style={{
      minHeight: "100vh", background: "#0d0d18", color: "#e2e8f0",
      fontFamily: "'Inter', -apple-system, sans-serif", padding: "16px 14px"
    }}>
      <link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;700&family=Inter:wght@400;500;600;700&display=swap" rel="stylesheet" />

      <div style={{ maxWidth: 960, margin: "0 auto" }}>
        {/* Header */}
        <div style={{ marginBottom: 14 }}>
          <h1 style={{
            fontSize: 18, fontWeight: 700, margin: 0,
            fontFamily: "'JetBrains Mono', mono", color: "#f8fafc"
          }}>🎛️ PID Tuning Dashboard</h1>
          <p style={{ fontSize: 12, color: "#64748b", margin: "4px 0 0" }}>
            Drag the red dot in the camera view. Watch how PID parameters affect tracking behavior.
          </p>
        </div>

        <div style={{ display: "grid", gridTemplateColumns: "320px 1fr", gap: 14 }}>
          {/* Left: Controls */}
          <div style={{ display: "flex", flexDirection: "column", gap: 12 }}>
            {/* Presets */}
            <div style={{
              background: "#12121f", borderRadius: 10, padding: 12,
              border: "1px solid #1e293b"
            }}>
              <div style={{
                fontSize: 10, fontWeight: 700, letterSpacing: "0.08em",
                color: "#64748b", marginBottom: 8, fontFamily: "mono"
              }}>PRESETS</div>
              <div style={{ display: "flex", gap: 6, flexWrap: "wrap" }}>
                <Preset label="Conservative" active={preset === "conservative"} onClick={() => applyPreset("conservative")} color="#2dd4bf" />
                <Preset label="Balanced" active={preset === "balanced"} onClick={() => applyPreset("balanced")} color="#60a5fa" />
                <Preset label="Aggressive" active={preset === "aggressive"} onClick={() => applyPreset("aggressive")} color="#f43f5e" />
                <Preset label="Try I" active={preset === "with_I"} onClick={() => applyPreset("with_I")} color="#fbbf24" />
                <Preset label="Try D" active={preset === "with_D"} onClick={() => applyPreset("with_D")} color="#a78bfa" />
              </div>
            </div>

            {/* PID Gains */}
            <div style={{
              background: "#12121f", borderRadius: 10, padding: 12,
              border: "1px solid #1e293b"
            }}>
              <div style={{
                fontSize: 10, fontWeight: 700, letterSpacing: "0.08em",
                color: "#64748b", marginBottom: 8, fontFamily: "mono"
              }}>PID GAINS</div>
              <Slider label="P (proportional)" value={P} onChange={v => { setP(v); setPreset("custom"); }} min={0} max={0.15} step={0.001} color="#f43f5e" />
              <Slider label="I (integral)" value={I} onChange={v => { setI(v); setPreset("custom"); }} min={0} max={0.01} step={0.0001} color="#fbbf24" />
              <Slider label="D (derivative)" value={D} onChange={v => { setD(v); setPreset("custom"); }} min={0} max={0.05} step={0.001} color="#a78bfa" />
            </div>

            {/* System params */}
            <div style={{
              background: "#12121f", borderRadius: 10, padding: 12,
              border: "1px solid #1e293b"
            }}>
              <div style={{
                fontSize: 10, fontWeight: 700, letterSpacing: "0.08em",
                color: "#64748b", marginBottom: 8, fontFamily: "mono"
              }}>SYSTEM PARAMETERS</div>
              <Slider label="Dead zone" value={deadZone} onChange={setDeadZone} min={0} max={40} step={1} color="#84cc16" unit="px" />
              <Slider label="Output limit" value={outputLimit} onChange={setOutputLimit} min={0.5} max={10} step={0.1} color="#fb923c" unit="°/frame" />
            </div>

            {/* Actions */}
            <div style={{ display: "flex", gap: 8 }}>
              <button onClick={() => setRunning(!running)} style={{
                flex: 1, padding: "8px 0", borderRadius: 6, border: "1px solid #1e293b",
                background: running ? "#1e293b" : "#2dd4bf22", color: running ? "#94a3b8" : "#2dd4bf",
                cursor: "pointer", fontSize: 12, fontFamily: "mono", fontWeight: 700
              }}>{running ? "⏸ Pause" : "▶ Run"}</button>
              <button onClick={handleReset} style={{
                flex: 1, padding: "8px 0", borderRadius: 6, border: "1px solid #1e293b",
                background: "#0f0f1a", color: "#94a3b8", cursor: "pointer",
                fontSize: 12, fontFamily: "mono"
              }}>↺ Reset</button>
              <button onClick={() => { sim.resetPID(); }} style={{
                flex: 1, padding: "8px 0", borderRadius: 6, border: "1px solid #1e293b",
                background: "#0f0f1a", color: "#94a3b8", cursor: "pointer",
                fontSize: 12, fontFamily: "mono"
              }}>Clear PID</button>
            </div>

            {/* Live values */}
            {latest && (
              <div style={{
                background: "#12121f", borderRadius: 10, padding: 12,
                border: "1px solid #1e293b", fontFamily: "mono", fontSize: 11
              }}>
                <div style={{
                  fontSize: 10, fontWeight: 700, letterSpacing: "0.08em",
                  color: "#64748b", marginBottom: 6
                }}>LIVE VALUES</div>
                <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 4, color: "#94a3b8" }}>
                  <span>Error X:</span><span style={{ color: "#f43f5e" }}>{latest.rawErrorX.toFixed(1)}px</span>
                  <span>Error Y:</span><span style={{ color: "#f43f5e" }}>{latest.rawErrorY.toFixed(1)}px</span>
                  <span>Adj X:</span><span style={{ color: latest.saturatedX ? "#ef4444" : "#84cc16" }}>{latest.adjX.toFixed(3)}°{latest.saturatedX ? " ⚠" : ""}</span>
                  <span>Adj Y:</span><span style={{ color: latest.saturatedY ? "#ef4444" : "#84cc16" }}>{latest.adjY.toFixed(3)}°{latest.saturatedY ? " ⚠" : ""}</span>
                  <span>Gimbal:</span><span style={{ color: "#60a5fa" }}>({latest.gimbalX.toFixed(1)}°, {latest.gimbalY.toFixed(1)}°)</span>
                </div>
              </div>
            )}
          </div>

          {/* Right: Visualizations */}
          <div style={{ display: "flex", flexDirection: "column", gap: 12 }}>
            {/* Camera + World views */}
            <div style={{ display: "flex", gap: 12 }}>
              <div>
                <CameraView sim={sim} deadZone={deadZone}
                  onDrag={onDrag} targetX={targetX} targetY={targetY} dragging={dragging} />
                <div style={{ fontSize: 10, color: "#475569", marginTop: 4, fontFamily: "mono", textAlign: "center" }}>
                  Click & drag to move target
                </div>
              </div>
              <div>
                <WorldView sim={sim} targetX={targetX} targetY={targetY} />
                <div style={{ fontSize: 10, color: "#475569", marginTop: 4, fontFamily: "mono", textAlign: "center" }}>
                  Teal box = camera FOV
                </div>
              </div>
            </div>

            {/* Graphs */}
            <div style={{
              background: "#12121f", borderRadius: 10, padding: 12,
              border: "1px solid #1e293b",
              display: "grid", gridTemplateColumns: "1fr 1fr", gap: 12
            }}>
              <MiniGraph data={history} dataKey="rawErrorX" label="Error X (px)" color="#f43f5e" maxVal={320} />
              <MiniGraph data={history} dataKey="rawErrorY" label="Error Y (px)" color="#fb923c" maxVal={240} />
              <MiniGraph data={history} dataKey="adjX" label="Adjustment X (°)" color="#84cc16" maxVal={outputLimit * 1.2} />
              <MiniGraph data={history} dataKey="adjY" label="Adjustment Y (°)" color="#2dd4bf" maxVal={outputLimit * 1.2} />
              <MiniGraph data={history} dataKey="gimbalX" label="Gimbal X (°)" color="#60a5fa" />
              <MiniGraph data={history} dataKey="gimbalY" label="Gimbal Y (°)" color="#a78bfa" />
            </div>

            {/* Explanation */}
            <div style={{
              background: "#12121f", borderRadius: 10, padding: 12,
              border: "1px solid #1e293b", fontSize: 12, color: "#64748b", lineHeight: 1.6
            }}>
              <strong style={{ color: "#94a3b8" }}>Try these experiments:</strong>
              <div style={{ marginTop: 6 }}>
                <span style={{ color: "#2dd4bf" }}>1.</span> Set to Conservative, drag the dot away, let go — watch it settle smoothly<br/>
                <span style={{ color: "#2dd4bf" }}>2.</span> Switch to Aggressive — same test, notice the faster response but more overshoot<br/>
                <span style={{ color: "#2dd4bf" }}>3.</span> Click "Try I", then drag the dot just inside the dead zone — watch it slowly creep to center<br/>
                <span style={{ color: "#2dd4bf" }}>4.</span> With I active, drag the dot in circles — watch the ⚠ saturation warnings and circling behavior<br/>
                <span style={{ color: "#2dd4bf" }}>5.</span> Click "Try D", drag the dot quickly — D dampens overshoot but makes response jittery<br/>
                <span style={{ color: "#2dd4bf" }}>6.</span> Crank output limit to 10° — watch the adjustment graph slam to the limit (saturation)<br/>
                <span style={{ color: "#2dd4bf" }}>7.</span> Set dead zone to 0 — watch the gimbal never fully settle (chasing sub-pixel noise)
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
