# Hack-Latam · Orbital Debris Detection — Roadmap

A forward-looking plan from the current code state to a working stage demo
and pitch. Time budgets are guidance, not deadlines; the **go/no-go gates**
between phases are what matters. If a gate fails, fall back to the documented
backup before sinking more hours.

---

## 0. Where we are now

### Code that exists and works

| Area | State |
|---|---|
| Python fusion pipeline | `--simulate` end-to-end works. Bistatic-Fresnel solver hits 0.2–1.4% speed error and ≤1° bearing error on synthetic noisy data. |
| Python receiver (UDP) | Accepts `{rx, t, score?, amp, rssi}` JSON packets; computes motion score from rolling baseline if not pre-computed by ESP32. |
| Dashboard (Vite + React) | Builds clean. StatusBar / BigReadout / TopDownView (Tx + 3 Rx) / EventLog / GlobeView (constellation + linear-physics debris orbits + moving debris dots) all render. Demo mode auto-fires synthetic transits every ~3 s. |
| ESP-IDF firmware | `tx_node` (ESP-NOW broadcast at ~500 Hz) and `rx_node` (promiscuous CSI capture + UDP) source written. **Not yet built/flashed.** |

### Critical missing pieces

1. **FastAPI WebSocket bridge.** The dashboard expects `ws://127.0.0.1:8000/ws/events`. Nothing serves that today. Live data won't reach the dashboard until this is built. (~2–3h)
2. **Anything physical.** No firmware has been flashed; no hotspot is running; no Tx/Rx have been placed in a room.
3. **Calibration.** Sensor positions in [config.py](fusion/config.py) are placeholders. The real room geometry needs to be measured.

---

## Phase 1 — Firmware bring-up · 4–6 h · **CRITICAL PATH**

**Goal:** real CSI JSON packets flowing from 1 Tx + 3 Rx ESP32s into the laptop on UDP 5005.

### Steps

1. **Install ESP-IDF** (VS Code + Espressif extension on Windows is smoothest). Verify with `idf.py --version`.
2. **Start the laptop's WiFi hotspot** (Windows: Settings → Network → Mobile Hotspot). Note its IP (usually `192.168.137.1`). Use a memorable SSID/password — paste these into both firmware files.
3. **Flash the Tx:**
   ```
   cd firmware/tx_node
   idf.py set-target esp32
   idf.py build flash monitor
   ```
   Read its MAC from the serial output. Paste into `TX_MAC[]` in [firmware/rx_node/main/main.c](firmware/rx_node/main/main.c).
4. **Flash three Rx boards** with different `RX_ID`s:
   ```
   cd firmware/rx_node
   idf.py build -DRX_ID=1 flash    # repeat with =2 and =3 on the other two boards
   ```
5. **Verify packets arrive:**
   ```
   python -c "import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(('0.0.0.0', 5005)); [print(s.recvfrom(2048)[0][:140]) for _ in range(6)]"
   ```

### Go/no-go gate

You see one JSON line per Rx every few ms in that dump.

**If failed by hour 6:** see [Backup plan A](#backup-plan-a--csi-fails) below. Don't keep grinding — pivot the demo modality.

---

## Phase 2 — WebSocket bridge · 2–3 h

**Goal:** Python fusion pipeline emits live `frame` and `fit` events to the dashboard over WebSocket. Dashboard status badge turns **LIVE** instead of **DEMO MODE**.

### Steps

1. Add to [fusion/requirements.txt](fusion/requirements.txt):
   ```
   fastapi>=0.110
   uvicorn[standard]>=0.27
   websockets>=12
   ```
2. Create `fusion/server.py` with:
   - FastAPI app + `/ws/events` WebSocket endpoint
   - Background asyncio task: reads UDP from existing `open_udp()` socket, parses with `parse_line()`, runs `fit_trajectory()` on completed events
   - Broadcasts `{type: "csi", ...}` per frame and `{type: "fit", ...}` per fit to all connected clients
3. Run with `uvicorn fusion.server:app --host 127.0.0.1 --port 8000`
4. Open the dashboard at `http://127.0.0.1:5173` and confirm the StatusBar pulses green.

### Go/no-go gate

Status badge says **LINK · LIVE** and packet/fit counters increment when you wave near the sensors.

---

## Phase 3 — Physical setup + calibration · 4–6 h

**Goal:** Tx and three Rx mounted in the actual demo space with measured positions in [config.py](fusion/config.py), quiet baseline collected, target choice locked.

### Steps

1. **Mount the four ESP32s.** Tx on a tripod or tape on one wall; three Rx on the opposite wall in a triangle:
   - Rx-1: left, ~2.4 m from Tx
   - Rx-2: right, ~2.4 m from Tx
   - Rx-3: centre, ~3.3 m from Tx (further back for triangulation)
   - Heights: keep all four at the same height (waist level is fine).
2. **Measure positions with a tape** to ±5 cm. Update `TX_POSITION` and `RX_POSITIONS` in [config.py](fusion/config.py).
3. **Quiet-room baseline:** with no movement in the room, run `python main.py` for 30 s. Score histogram per Rx should be ≤0.05.
4. **Sanity test:** walk slowly across the gate — all three Rx scores should spike to 0.3+. If only one Rx fires, geometry is wrong; rearrange.
5. **Pick the target.** In order of preference:
   - **Foil-wrapped tennis ball** (3+ layers, tight) — the recommended target
   - Aluminium pan lid swung on a string — easy backup
   - Drone hovering through the gate — easy but slow
6. **Tune `FRESNEL_SIGMA_M`** in [config.py](fusion/config.py). Observe the half-width of motion-score peaks when a target crosses; that's roughly `sigma`. Real value will probably be 0.4–0.7 m at 2.4 GHz.

### Go/no-go gate

A foil-wrapped target thrown through the gate produces score peaks ≥ 0.18 on at least 2 of 3 Rx. If not, target is too small — see [Backup plan B](#backup-plan-b--target-too-small).

---

## Phase 4 — Trajectory accuracy · 3–4 h

**Goal:** fits land within ±20% of the eyeballed truth for speed and ±15° for bearing across 20+ test throws.

### Steps

1. Throw the target through the gate ~5 times at different angles and speeds. Record each:
   - Approximate truth: distance / stopwatch time, throw direction
   - Fit output from the dashboard
2. If `vx` is consistently flipped, the multi-start solver's mirror tiebreaker is failing — check that the Tx is centred and the three Rx form a non-degenerate triangle.
3. If RMSE is high (>0.08 in linear score units), revisit `FRESNEL_SIGMA_M` and `MOTION_SCORE_TRIGGER`.
4. If a single Rx is consistently late/early in its peak, its position measurement is probably off. Re-measure.
5. **Record a clean run** to MP4 with OBS or QuickTime once you have a good throw. **This is your backup demo if the live throw fails on stage.**

### Go/no-go gate

You have a clean recorded run AND can produce a live fit within ±20% in 3 out of 5 throws.

---

## Phase 5 — Pitch + rehearsal · 4–6 h

**Goal:** an 8–10 slide deck and a 5-minute demo rehearsed end-to-end.

### Pitch beats (the deff/acc story)

1. **Problem.** Kessler syndrome; the 1–10 cm orbital debris gap that ground radar can't see; ~1M objects, untracked, lethal at orbital velocity.
2. **Insight.** Sensors on the constellation itself can passively detect debris as it perturbs the inter-satellite RF links. Joint Communication and Sensing — every satellite is also a radar.
3. **What we built.** Live demo: 1 Tx + 3 Rx ESP32s; WiFi CSI as a stand-in for the on-orbit radio links; a foil-wrapped target stands in for ~3 cm metallic debris (same RCS class as real LEO fragments).
4. **The math.** Bistatic Fresnel model; full-curve nonlinear least squares; ~1% speed accuracy on synthetic, ~10–20% on hardware.
5. **The constellation view.** Each lab-frame trajectory projects linearly into an orbital great circle through the inter-satellite encounter point; debris orbit crossing the constellation is shown in real time.
6. **Honest limitations.** Lab-frame velocity is not absolute orbital energy; we show direction-of-orbit, not eccentricity. ECI as ECEF. No J2. Document this.
7. **What's real today vs simulated.** Real: CSI capture, motion fusion, trajectory math. Simulated: orbital projection geometry, constellation positions.
8. **Roadmap.** Hosted-payload pilot on a single Iridium NEXT slot; ground-truth correlation with LeoLabs; integration with existing conjunction-warning systems.
9. **Team / ask.**

### Rehearsals

- Run the full demo 5+ times.
- Time it. 5 minutes is short.
- Have **one person operate the dashboard**, one person throw, one person narrate.
- **Charge everything overnight.** Power banks, laptop, ESP32s.

---

## Phase 6 — Demo day · 2–4 h before stage

- Arrive early. Set up the hotspot. Place sensors. Run a calibration sweep in the actual demo venue (WiFi is louder there than where you tested).
- Have the **backup video** queued in a browser tab. If the live throw fails twice, switch.
- Bring spares: 1 extra ESP32, 5+ foil targets, USB-C cable, two power banks, tape, scissors, a measuring tape.
- Do **nothing new** in the last 6 hours. No code changes. No "small improvements." Stabilise.

---

## Risks and backup plans

### Backup plan A — CSI fails

If by **hour 6** ESP-IDF won't build, or CSI packets aren't arriving, or `wifi_csi_info_t->len == 0`:

- The repo still contains [firmware/sensor_node/](firmware/sensor_node/) — the Arduino HB100 doppler approach, fully working in earlier commits. Resurrect the doppler architecture.
- The Python solver is modality-agnostic enough that the rewrite cost back to doppler is small (≈3–4 h).
- The pitch shifts from "CSI / JCAS" to "doppler radar mesh" — still valid for the deff/acc track.

### Backup plan B — Target too small

If the foil-wrapped tennis ball doesn't trigger reliable peaks:

- Try a **larger metal lid swung on a string** through the gate. Still credible — just slow it down.
- Try a **toy drone** flying through. Bigger RCS, slower, almost-guaranteed detection. Pitch it as "we calibrated against a slower drone-class target to demonstrate the geometry; the system extends to faster, smaller debris."

### Backup plan C — Live demo fails on stage

- Have the recorded clean-run video ready in a tab.
- Narrate over the video: *"In rehearsal we measured 22.4 m/s, bearing 87°. Live, our hotspot has competition — this is the recorded ground truth."* Honest framing beats a broken live demo.

---

## What NOT to do in the remaining hours

- **No new features after hour 40.** Polish only.
- **No refactoring.** The code you have works; leave it.
- **No "small improvements" to the solver** unless they fix an observed failure.
- **No pivoting the architecture** once Phase 3 is done. You've already pivoted once (HB100 → CSI). Pivoting a second time spends more hours than you have left.
- **No staying up past hour 44 unless absolutely necessary.** A team that's slept 6 hours pitches better than a team that hasn't slept at all.

---

## Checklist before walking on stage

- [ ] All 4 ESP32s booted and connected to hotspot (LEDs / serial confirms)
- [ ] `uvicorn` running, dashboard shows **LINK · LIVE**
- [ ] One clean recorded video queued in browser tab
- [ ] Sensor positions match [config.py](fusion/config.py) (re-measure on site!)
- [ ] At least 3 backup foil targets
- [ ] Power banks at >50%
- [ ] One team member's phone hotspot as backup AP
- [ ] Demo script printed, not on the laptop you're showing
- [ ] Slack/Discord notifications off

Good luck.
