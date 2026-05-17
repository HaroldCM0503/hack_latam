# Hack-Latam · Orbital Debris Detection — Roadmap

A forward-looking plan from the current code state to a working stage demo
and pitch. Time budgets are guidance, not deadlines; the **go/no-go gates**
between phases are what matters.

**Modality is fixed: WiFi CSI interference, end of story.** This is the
*ESP32 + espressif/esp-csi + ESP-IDF Wi-Fi driver* path. We are not keeping
doppler, ultrasonic, camera, or any other modality in reserve. If CSI
struggles, we debug CSI — we don't pivot away from it.

---

## 0. Where we are now

### Code that exists and works

| Area | State |
|---|---|
| Python fusion pipeline | `--simulate` end-to-end works. Bistatic-Fresnel solver hits 0.2–1.4% speed error and ≤1° bearing error on synthetic noisy data. |
| Python receiver (UDP) | Accepts `{rx, t, score?, amp, rssi}` JSON packets; computes motion score from rolling baseline if not pre-computed by ESP32. |
| Dashboard (Vite + React) | Builds clean. StatusBar / BigReadout / TopDownView (Tx + 3 Rx) / EventLog / GlobeView (constellation + linear-physics debris orbits + moving debris dots) all render. Demo mode auto-fires synthetic transits every ~3 s. |
| ESP-IDF firmware | `tx_node` and `rx_node` skeletons exist with our own custom EWMA motion-score code. **Replace both with esp-csi upstream examples + esp-radar component** — see Phase 1. Not yet built/flashed. |

### Critical missing pieces

1. **Firmware rebuild on upstream.** Our `rx_node` currently reimplements amplitude scoring by hand. Swap it for esp-radar's on-device detector before flashing — it's the high-frame-rate path we want (~3–4 h, folded into Phase 1).
2. **FastAPI WebSocket bridge.** The dashboard expects `ws://127.0.0.1:8000/ws/events`. Nothing serves that today. Live data won't reach the dashboard until this is built. (~2–3 h)
3. **Anything physical.** No firmware has been flashed; no hotspot is running; no Tx/Rx have been placed in a room.
4. **Calibration.** Sensor positions in [config.py](fusion/config.py) are placeholders. The real room geometry needs to be measured.

### Upstream we are standing on — do not reimplement

We are **not writing a CSI stack, a Tx broadcaster, or a motion detector**. Espressif already ships all three. Our firmware should be a thin **UDP shipper** glued on top of their components. Anything more than that is wasted hours.

**[espressif/esp-csi](https://github.com/espressif/esp-csi)** — the reference repository. Four pieces from it that we adopt:

| Upstream piece | What it gives us | Our role |
|---|---|---|
| `examples/get-started/csi_send` | Tx broadcaster — ESP-NOW frames at a fixed, tunable rate, with rate-locking and channel-locking already correct | Use it as `tx_node` **unmodified** (or near-unmodified) — only edit the SSID/password |
| `examples/get-started/csi_recv_router` | Rx CSI capture that **associates to a router and pulls CSI from its own beacons/data** — proven to hit ~100–500 Hz with the default Tx | Template for our Rx; we add only the UDP shipping line |
| `components/esp-radar` | On-device motion / presence / breath detector consuming the raw CSI stream. **Replaces the rolling-EWMA scorer we wrote ourselves.** | Register `esp_radar_cb_t`; ship the `motion` field over UDP instead of our hand-rolled `score` |
| `examples/console_test` | Pre-built app that runs esp-radar end-to-end and prints motion/presence to a CLI | Smoke-test on one Rx before integrating — if its motion metric responds, ours will too |

**[ESP-IDF Wi-Fi driver guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi-driver/index.html)** — authoritative docs for everything we touch:

- `esp_wifi_set_csi_config` / `esp_wifi_set_csi_rx_cb` / `wifi_csi_info_t` — the CSI API
- `esp_wifi_set_promiscuous` / `wifi_promiscuous_filter_t` — promiscuous + frame filtering
- `esp_wifi_config_80211_tx_rate` — **fixed-MCS Tx, the main knob for stable high frame rate**
- ESP-NOW section — broadcast cadence, max payload, ack semantics
- "ESP32 Wi-Fi Channel State Information" — channel-locking gotchas (the #1 cause of "CSI works but rate is unstable")
- Power save — must be `WIFI_PS_NONE` on both ends

Rule of thumb: if a problem is **about CSI semantics, Tx rate, or Wi-Fi state machine behaviour**, the answer is in one of those two links. Don't patch our code blindly — check the reference first.

### High frame rate — what we tune, what we don't write

We want **≥200 Hz of CSI frames per Rx**, ideally 500 Hz. The path to that is *configuration*, not code:

1. **Tx side** — use esp-csi's `csi_send` as-is. The frame rate is controlled by the ESP-NOW send-loop delay (a `vTaskDelay(pdMS_TO_TICKS(N))` in their `wifi_csi_send_task`). Lower N → higher rate. Keep payload tiny (≤4 bytes; the radar metric is in the channel response, not the payload).
2. **Fix the Tx PHY rate** to a low MCS via `esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_MCS0_LGI)` — short frames, deterministic timing, much more uniform inter-arrival than the auto-rate default. This is the single biggest knob for "high *and stable* frame rate".
3. **Lock the channel.** Both nodes must associate to the same hotspot, and the hotspot must not channel-hop. Windows mobile hotspot fixes channel by default; just verify with `esp_wifi_get_channel` on both ends and confirm they match in the stats log.
4. **Rx side** — esp-radar already runs in its own task; the CSI callback only enqueues. Our UDP `sendto` is the one new failure mode we introduce — keep the JSON small (drop the `amp`/`real`/`imag` arrays once esp-radar's motion field is the only signal we need; one UDP packet should be ~80 bytes, not ~2 KB). At 500 Hz × 80 B that's 40 KB/s per Rx, well within margin.
5. **Don't** roll your own ring buffer, your own subcarrier amplitude estimator, your own baseline tracker, your own breath/presence detector, or your own batching layer. All exist in esp-radar already. Wire ours up only if a specific upstream behaviour is wrong for our use case — and document why before doing so.

---

## Phase 1 — Firmware bring-up · 4–6 h · **CRITICAL PATH**

**Goal:** ≥200 Hz of CSI-derived motion-score JSON packets per Rx flowing into the laptop on UDP 5005, built by **adopting esp-csi's `csi_send` / `csi_recv_router` examples + the `esp-radar` component** and bolting a UDP `sendto` on the receive side. We write the shipper, not the radar.

### Steps

1. **Install ESP-IDF v5.x** (VS Code + Espressif extension on Windows is smoothest). Verify with `idf.py --version`. The Wi-Fi driver docs assume v5.x APIs.
2. **Clone esp-csi as the upstream source of truth:**
   ```
   git clone https://github.com/espressif/esp-csi.git ../esp-csi
   ```
   Smoke-test `examples/console_test` on **one** Rx board for 5 minutes. Wave a hand in front of it; watch the CLI's `motion` metric react. If that fails, no further work helps — fix the toolchain / board / antenna before continuing.
3. **Start the laptop's WiFi hotspot** (Windows: Settings → Network → Mobile Hotspot). Note its IP (usually `192.168.137.1`). Pick a memorable SSID/password. Verify the hotspot is on a fixed 2.4 GHz channel (it is by default on Windows). All three Rx **and** the Tx will associate to this hotspot.
4. **Adopt `csi_send` as `tx_node`.** Copy `esp-csi/examples/get-started/csi_send/` over our [firmware/tx_node/](firmware/tx_node/) (or just flash it from in-place — there is nothing we need to add). Tune two things only:
   - `vTaskDelay` in the send loop → controls frame rate. Start at `pdMS_TO_TICKS(2)` (~500 Hz) and back off if you see drops.
   - Add `esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_MCS0_LGI)` after `esp_wifi_start()` for stable inter-arrival.
   Flash, monitor, copy the MAC into the Rx config.
5. **Build the Rx as `csi_recv_router` + `esp_radar` + UDP shipper.** Replace the bulk of [firmware/rx_node/main/main.c](firmware/rx_node/main/main.c) with the structure from `esp-csi/examples/get-started/csi_recv_router/main/app_main.c`, then:
   - Register `esp_radar_cb` (from the esp-radar component) instead of our hand-rolled motion-score block.
   - In the radar callback, format a single small JSON line `{"rx":N,"t":...,"rssi":...,"motion":...,"presence":...}` and `sendto()` it. **Drop the `amp`/`real`/`imag` arrays** — they were our fallback when we didn't have a real motion detector; we do now.
   - Add `esp-radar` to `main/CMakeLists.txt` `REQUIRES` and copy `components/esp-radar/` into our project (or use `EXTRA_COMPONENT_DIRS` to point at the esp-csi checkout).

   Flash three boards with `-DRX_ID=1/2/3`.
6. **Verify packets arrive at ≥200 Hz per Rx:**
   ```
   python -c "import socket,time; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(('0.0.0.0', 5005)); t=time.time(); n={1:0,2:0,3:0}; \
   exec(\"while time.time()-t<5:\\n d,_=s.recvfrom(2048)\\n import json; m=json.loads(d); n[m['rx']]+=1\"); print(n)"
   ```
   Expect each Rx to report ≥1000 frames over 5 s.
7. **Update [fusion/](fusion/) to consume `motion` directly.** The Python receiver currently recomputes the score from raw amplitudes; with esp-radar shipping a real `motion` field, switch `parse_line()` to prefer that field when present. The bistatic-Fresnel fitter does not care which detector produced the score curve — only that the curve has the right shape near gate crossing.

### Go/no-go gate

You see one JSON line per Rx every few ms in that dump.

**If failed by hour 6:** see [Backup plan A](#backup-plan-a--csi-bring-up-stalls) below. The fallback is *still CSI* — we drop in the upstream esp-csi example unmodified and ship its output. We do not change modality.

### Common Phase 1 failures — look here first

Each of these is documented in the references above; check there before "fixing" our code.

- **CSI rate is well under 200 Hz.** Tx send loop delay is too large, or PHY rate auto-negotiation is hopping. Lower the `vTaskDelay` in `csi_send`'s send task and add `esp_wifi_config_80211_tx_rate(...MCS0_LGI)`. The Wi-Fi driver guide's "Wi-Fi Throughput" and "Channel state information" sections cover the rate-locking knobs.
- **`csi_total` increments but no CSI from our Tx MAC.** Either the TX_MAC string is wrong (re-read from Tx serial) or the Tx and Rx associated to different hotspots / are on different channels. The Wi-Fi driver guide's "Promiscuous Mode" section explains how the station-mode channel is locked by the AP association.
- **`first_word_invalid` flag set on most frames.** Antenna / RF environment issue. esp-csi README has a troubleshooting matrix; the short version is: keep boards ≥30 cm apart, avoid USB hubs, prefer external-antenna ESP32 variants. Note: esp-radar already drops these frames internally — if we still see flat motion, the source frames are the problem, not the detector.
- **esp-radar's `motion` field is flat even though CSI flows.** Run `esp-csi/examples/console_test` on the same board with the same Tx. If its motion metric is also flat, the Tx is too close to the Rx (saturating LNA) or the room is RF-dead. Move sensors farther apart; check `rssi` is in -40 to -70 dBm range.
- **UDP packets drop above 300 Hz.** JSON payload too large. Confirm you stripped `amp`/`real`/`imag` arrays once esp-radar is wired in — payload should be ~80 B, not ~2 KB.

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

### Backup plan A — CSI bring-up stalls

If by **hour 6** our Rx isn't producing JSON over UDP, the fallback is **still CSI** — we are not changing modality. Steps in order, ≈1 h each:

1. **Run `esp-csi/examples/console_test` unmodified** on one Rx. If its `motion` metric reacts to movement, esp-radar and the radio path both work — the bug is in our UDP shipping or build wiring. Diff our `app_main.c` against the example's and converge.
2. **If `console_test` also doesn't react,** flash `esp-csi/examples/get-started/csi_recv_router` (raw CSI, no radar) and confirm CSI frames at least *arrive*. If they don't, the issue is environmental — toolchain version, ESP32 variant, channel mismatch, or RF setup. Walk the "Common Phase 1 failures" checklist above and re-read the [ESP-IDF Wi-Fi driver guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi-driver/index.html) sections on CSI and Promiscuous Mode.
3. **As a last resort, run upstream `csi_recv_router` everywhere and tail its serial.** We lose UDP and on-device motion scoring, but we keep raw CSI flowing to Python via `idf.py monitor` piped to a small tail script that re-emits UDP. The [fusion/](fusion/) pipeline can compute motion score from raw amp/real/imag, so this path still gets us a live demo.

CSI is the demo. We debug until it works.

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
- **No modality pivots.** CSI is locked in. If something breaks, debug CSI — don't go looking for a different sensing approach. Every hour spent considering alternatives is an hour not spent making CSI work.
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
