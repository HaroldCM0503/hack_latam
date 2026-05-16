// useEvents - subscribe to the FastAPI WebSocket and fall back to a synthetic
// event generator if the backend is not reachable. This lets the dashboard
// run standalone for UI work without the Python side being online.
//
// The hook returns two streams of data:
//   - frames:    rolling buffer of per-sensor doppler readings ({node, t, v, amp, snr})
//   - fits:      rolling buffer of completed trajectory fits (one per rock transit)
//   - status:    "connected" | "demo"  | "connecting"
//
// Schema expected from backend:
//   { "type": "frame", "node": 1, "t": 12.345, "v": 24.5, "amp": 800, "snr": 22 }
//   { "type": "fit",
//     "t": 12.345,
//     "speed": 27.3, "bearing": 82.1,
//     "p0": [-0.27, 0.22], "v": [3.2, 23.6],
//     "gate_cross_x": -0.09, "rmse": 0.21, "n_frames": 12 }

import { useEffect, useRef, useState } from "react";

const WS_URL = "ws://127.0.0.1:8000/ws/events";
const FRAME_BUFFER_MS = 4000;      // keep the last 4 s of frames for the doppler chart
const FIT_BUFFER_LEN  = 20;        // remember last N fits for the event log + ghost trails
const DEMO_FALLBACK_MS = 1500;     // if no packet for this long, drop into demo mode

export function useEvents() {
  const [frames, setFrames] = useState([]);  // rolling per-sensor doppler frames
  const [fits,   setFits]   = useState([]);  // completed trajectory fits
  const [status, setStatus] = useState("connecting");
  const lastPacketRef = useRef(Date.now());
  const wsRef         = useRef(null);
  const demoTimerRef  = useRef(null);

  // ---- Real WebSocket -------------------------------------------------
  useEffect(() => {
    let cancelled = false;

    function connect() {
      if (cancelled) return;
      let ws;
      try {
        ws = new WebSocket(WS_URL);
      } catch {
        scheduleReconnect();
        return;
      }
      wsRef.current = ws;

      ws.onopen = () => {
        if (cancelled) return;
        setStatus("connected");
        lastPacketRef.current = Date.now();
      };
      ws.onmessage = (ev) => {
        if (cancelled) return;
        lastPacketRef.current = Date.now();
        try {
          const obj = JSON.parse(ev.data);
          ingest(obj);
        } catch {
          // ignore malformed
        }
      };
      ws.onerror = () => {
        // surface as close
      };
      ws.onclose = () => {
        if (cancelled) return;
        scheduleReconnect();
      };
    }

    function scheduleReconnect() {
      setStatus((s) => (s === "connected" ? "demo" : "demo"));
      setTimeout(connect, 2000);
    }

    connect();
    return () => {
      cancelled = true;
      try { wsRef.current?.close(); } catch {}
    };
  }, []);

  // ---- Heartbeat: if no packet for FALLBACK_MS, fall to demo mode -----
  useEffect(() => {
    const id = setInterval(() => {
      const dt = Date.now() - lastPacketRef.current;
      if (dt > DEMO_FALLBACK_MS) {
        setStatus((s) => (s === "connecting" ? "connecting" : "demo"));
      }
    }, 500);
    return () => clearInterval(id);
  }, []);

  // ---- Demo event generator (only when not connected) -----------------
  useEffect(() => {
    if (status === "connected") {
      if (demoTimerRef.current) clearInterval(demoTimerRef.current);
      demoTimerRef.current = null;
      return;
    }
    if (demoTimerRef.current) return;
    demoTimerRef.current = setInterval(() => {
      const ev = synthEvent();
      ingestDemo(ev);
    }, 220);
    return () => {
      if (demoTimerRef.current) {
        clearInterval(demoTimerRef.current);
        demoTimerRef.current = null;
      }
    };
  }, [status]);

  // ---- Ingestion ------------------------------------------------------
  // Schema (from FastAPI WebSocket / backend):
  //   { type: "csi", rx: 1, t: ..., score: 0.42, rssi: -42, amp?: [...] }
  //   { type: "fit", t: ..., speed, bearing, p0:[..], v:[..],
  //                  gate_cross_x, rmse, n_frames }
  function ingest(obj) {
    if (obj.type === "csi" || obj.type === "frame") {
      pushFrame(obj);
    } else if (obj.type === "fit") {
      pushFit(obj);
    }
  }
  function ingestDemo(burst) {
    burst.frames.forEach(pushFrame);
    if (burst.fit) pushFit(burst.fit);
  }

  function pushFrame(f) {
    const now = performance.now() / 1000;
    setFrames((prev) => {
      const next = [...prev, { ...f, t_recv: now }];
      const cutoff = now - FRAME_BUFFER_MS / 1000;
      while (next.length && next[0].t_recv < cutoff) next.shift();
      return next;
    });
  }
  function pushFit(fit) {
    setFits((prev) => {
      const next = [{ ...fit, t_recv: performance.now() / 1000 }, ...prev];
      if (next.length > FIT_BUFFER_LEN) next.length = FIT_BUFFER_LEN;
      return next;
    });
  }

  return { frames, fits, status };
}

// ---------------------------------------------------------------------
// Synthetic event generator for demo mode (CSI version)
// ---------------------------------------------------------------------
// Geometry matches fusion/config.py: Tx at (0,0), 3 Rx in a triangle.
const DEMO_TX = [0.0, 0.0];
const DEMO_RX = {
  1: [-1.20, 2.40],
  2: [+1.20, 2.40],
  3: [ 0.00, 3.30],
};
const FRESNEL_SIGMA_M = 0.55;
const GATE_Y = 2.0;

let demoPhase = 0;
function synthEvent() {
  // Trigger a "transit" every ~3 seconds: 10 frames per Rx during the transit,
  // then a fit. Between transits, emit faint idle frames.
  demoPhase += 1;
  const inTransit = demoPhase % 16 < 10;

  if (!inTransit) {
    const rx = (demoPhase % 3) + 1;
    return {
      frames: [{
        type:  "csi",
        rx,
        t:     performance.now() / 1000,
        score: 0.01 + Math.random() * 0.02,           // quiet baseline
        rssi:  -42 - Math.random() * 4,
      }],
      fit: null,
    };
  }

  // Active transit: synthesise a rock at a known constant-velocity trajectory.
  const truth = {
    p0: [(-0.6 + Math.random() * 1.2), -0.3],          // x slightly randomised
    v:  [(-2.5 + Math.random() * 5.0), 18 + Math.random() * 8],
  };
  const k = demoPhase % 16;                            // 0..9 in transit
  const t = k / 45.0;                                  // 45 Hz frame rate
  const sigma2_inv = 1.0 / (2.0 * FRESNEL_SIGMA_M * FRESNEL_SIGMA_M);
  const out = [];
  for (const rxStr of Object.keys(DEMO_RX)) {
    const rxId = +rxStr;
    const [rxX, rxY] = DEMO_RX[rxId];
    const px = truth.p0[0] + truth.v[0] * t;
    const py = truth.p0[1] + truth.v[1] * t;
    const segLen = Math.hypot(DEMO_TX[0] - rxX, DEMO_TX[1] - rxY);
    const detour = Math.hypot(px - DEMO_TX[0], py - DEMO_TX[1])
                 + Math.hypot(px - rxX,         py - rxY)
                 - segLen;
    const score = Math.max(
      0,
      Math.exp(-(detour * detour) * sigma2_inv) + (Math.random() - 0.5) * 0.05
    );
    out.push({
      type:  "csi",
      rx:    rxId,
      t:     performance.now() / 1000,
      score,
      rssi:  -40 - 0.1 * (rxX * rxX + rxY * rxY) + (Math.random() - 0.5),
    });
  }

  let fit = null;
  if (k === 9) {
    const speed   = Math.hypot(truth.v[0], truth.v[1]);
    const bearing = (Math.atan2(truth.v[1], truth.v[0]) * 180) / Math.PI;
    const dt      = (GATE_Y - truth.p0[1]) / truth.v[1];
    const gateX   = truth.p0[0] + truth.v[0] * dt;
    fit = {
      type:    "fit",
      t:       performance.now() / 1000,
      speed:   speed   * (1 + (Math.random() - 0.5) * 0.03),
      bearing: bearing + (Math.random() - 0.5) * 2,
      p0:      truth.p0,
      v:       truth.v,
      gate_cross_x: gateX,
      rmse:    0.015 + Math.random() * 0.020,
      n_frames: 27,
    };
  }
  return { frames: out, fit };
}
