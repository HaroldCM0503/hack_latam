# CSI Multi-Object Anomaly Detector — Project Manifest

## Overview

This project is a browser-based simulator for **passive Wi-Fi CSI anomaly detection and localization**. It detects the **presence of unknown objects** and **estimates their 3D positions** by analyzing CSI interference patterns across 6 communication links between satellites (or ESP32s in the lab).

The core concept: LEO satellites (and ESP32 nodes in the lab) already communicate with each other. When unknown objects enter the propagation environment, the CSI changes. By capturing a **baseline** (empty-room fingerprint) and subtracting it from live measurements, we isolate reflections caused by new objects, then triangulate their positions from multiple links.

## Runtime

- Type: static web app
- Languages: HTML, CSS, JavaScript
- Build step: none
- Package manager: none
- External dependencies: none

Run locally:

```powershell
python -m http.server 5173
```

Open:

```text
http://127.0.0.1:5173/
```

Syntax check:

```powershell
node --check engine.js; node --check app.js
```

## File Inventory

| File | Purpose |
| --- | --- |
| `index.html` | UI layout: environment selector, multi-object controls, detection panel, 6-link viewer, charts |
| `styles.css` | Dark UI styling with detection-specific accents (purple for detections, green for safe) |
| `engine.js` | Core simulation: link graph, CSI synthesis, baseline subtraction, CIR, CFAR, association, position solver |
| `app.js` | UI state, drawing routines, controls, render loop |
| `PROJECT_MANIFEST.md` | This manifest |

## Link Architecture

The system operates on **6 simultaneous links** between 4 nodes (1 transmitter + 3 receivers):

| Link | Type | Endpoints | Purpose |
| --- | --- | --- | --- |
| Tx→R1 | Primary | GPS/AP → LEO 1/ESP32 1 | Baseline communication link |
| Tx→R2 | Primary | GPS/AP → LEO 2/ESP32 2 | Baseline communication link |
| Tx→R3 | Primary | GPS/AP → LEO 3/ESP32 3 | Baseline communication link |
| R1↔R2 | Inter-node | LEO 1 ↔ LEO 2 | Cross-link (ISL / ESP32 mesh) |
| R1↔R3 | Inter-node | LEO 1 ↔ LEO 3 | Cross-link (ISL / ESP32 mesh) |
| R2↔R3 | Inter-node | LEO 2 ↔ LEO 3 | Cross-link (ISL / ESP32 mesh) |

Inter-node links give stronger object reflections (objects are near receivers) and diverse geometry for better position discrimination. In Iridium, these correspond to real Ka-band inter-satellite links. In the lab, each ESP32 pair forms a natural inter-node link.

Links are defined in `engine.js` via `buildLinks(room)` and stored as `room.links`, each with `{tx, rx, label, id}`.

## Detection Pipeline

```
1. Capture baseline CSI on all 6 links (empty room → H_baseline[k] per link)
2. Generate live CSI with N objects (superpose reflections from all objects on each link)
3. Subtract per link: ΔH[k] = H_live[k] − H_baseline[k]
4. IFFT(ΔH) per link → ΔCIR (residual impulse response)
5. CFAR peak detection on |ΔCIR| per link → candidate peaks with bistatic range
6. Cross-link association: group peaks at similar delays across ≥ min_links → confirmed detections
7. Position estimation: solve ellipsoid intersection for each confirmed detection
```

## Position Estimation Strategy

### Physical Basis — Bistatic Ellipsoids

Each CFAR peak on a link gives a **bistatic range**: the total path length `Tx → Object → Rx`. This constrains the object to an **ellipsoid** in 3D space with the link's transmitter and receiver as foci:

```
dist(Tx_i, object) + dist(object, Rx_i) = baseline_dist_i + excess_range_i
```

Where `excess_range_i = delay_peak × speed_of_light` is the range excess measured by CFAR.

With N links confirming the same detection, we have N ellipsoid constraints. The object lies at their intersection.

| Confirming links | Geometric constraint | Position quality |
| --- | --- | --- |
| 1 | Ellipsoid surface | No localization (infinite solutions) |
| 2 | Curve (ellipsoid intersection) | Ambiguous |
| 3 | Discrete point(s) | Solvable, may have ghost solutions |
| 4–6 | Overdetermined | Robust least-squares solution |

### Solver — Gauss-Newton Least Squares

The position is estimated by iteratively minimizing the residual vector:

```
r_i = dist(est, Tx_i) + dist(est, Rx_i) − measured_bistatic_range_i
```

The Jacobian is analytic — each row is the sum of unit vectors from the estimate to the two foci:

```
J_i = unit(est − Tx_i) + unit(est − Rx_i)
```

The normal equations `(J^T J) δ = −J^T r` are solved via a 3×3 Gaussian elimination with partial pivoting, giving a position update `δ` per iteration. Convergence is typically reached in 4–8 iterations.

The initial guess is the centroid of all node positions involved in the detection.

Implementation: `engine.js`, `solvePositionFromDetection()` and `solve3x3()`.

### Why 6 Links Matters for Estimation

With only the 3 primary links (GPS → LEO), all ellipsoids share the same focus (GPS), limiting geometric diversity. The 3 inter-node links add ellipsoids with different foci (LEO pairs), providing:

- **Perpendicular baselines** in the receiver plane → resolves lateral ambiguity
- **Shorter path lengths** → higher SNR on CFAR peaks → more accurate delay measurement
- **Overdetermined system** (6 equations, 3 unknowns) → robust least-squares with redundancy

### Peak Association — The Ghost Target Problem

Before solving, peaks from different links must be matched to the same physical object. The current approach uses **delay-proximity clustering**: peaks within a tolerance window (~8 ns) across different links are grouped together.

This works well when objects are sparse (typical case: space debris detection, room presence sensing). For dense object fields, more sophisticated methods like multi-hypothesis tracking would be needed.

Implementation: `engine.js`, `associateDetections()`.

## Background / Room Reflection Subtraction

Static room reflections (walls, furniture, Earth surface in orbit) are modeled as deterministic multipath components in the CSI. The baseline captures these with zero objects present. Subtraction removes them entirely, isolating object-induced perturbations only.

The model includes:
- 1 direct-path component per link
- 1 primary room-multipath component with subcarrier-dependent delay
- 4 additional static reflectors for realism
- Gaussian noise per subcarrier

For slowly drifting environments, the baseline can be recaptured at any time.

## Noise Resilience

| Strategy | Description | Control |
| --- | --- | --- |
| CFAR threshold | Adaptive noise-floor threshold per delay bin | σ slider (1.5–8.0) |
| Guard cells | CFAR guard band to prevent peak self-contamination | Guard cells slider (1–6) |
| Spatial correlation | Require detection on multiple links simultaneously | Min links slider (1–6) |
| Subcarrier coherence | Real objects create phase-coherent ΔH; noise is incoherent | Implicit in CSI physics |
| Link diversity | 6 links from different geometries reduce single-link false alarms | Inherent in architecture |

## Key Code Locations

| Area | Location |
| --- | --- |
| Physical constants | `engine.js`, top |
| Link graph builder | `engine.js`, `buildLinks()` |
| Room and orbit presets | `engine.js`, `roomPresets` |
| CSI synthesis (per link) | `engine.js`, `generateCsiForLink()` |
| Baseline subtraction | `engine.js`, `subtractCsi()` |
| CIR computation | `engine.js`, `computeCir()` |
| CFAR peak detection | `engine.js`, `cfarDetect()` |
| Cross-link association | `engine.js`, `associateDetections()` |
| Position solver | `engine.js`, `solvePositionFromDetection()` |
| 3×3 linear solver | `engine.js`, `solve3x3()` |
| Anomaly strength | `engine.js`, `anomalyStrength()` |
| Scene drawing | `app.js`, `drawScene()` |
| CSI/CIR chart drawing | `app.js`, `drawCsi()`, `drawCir()` |
| Detection list UI | `app.js`, `updateDetections()` |
| Baseline capture | `app.js`, `captureBaseline()` |
| Main render loop | `app.js`, `render()` |

## Controls

- **Environment**: lab bench, classroom, warehouse, orbit-scale (Iridium-like LEO cluster)
- **Object count**: 0–8 simultaneous unknown objects
- **Reflectivity**: shared reflectivity for all objects
- **Ambient noise**: communication channel noise level
- **Animate objects**: toggle object motion
- **Capture Baseline**: record empty-room CSI fingerprint across all 6 links
- **CFAR σ**: detection threshold sensitivity (higher = fewer false alarms, lower = more sensitive)
- **Guard cells**: CFAR guard band size
- **Min links**: minimum confirming links for a detection (1–6)
- **Link view**: select which of the 6 links' CSI/CIR to display (3 primary + 3 inter-node)

## Orbit Preset — LEO Detection Focus

The orbit preset places objects within ~500 km of the LEO constellation (corridor `maxT = 0.10`), focusing on the primary use case: detecting unknown objects (debris, untracked satellites) in the vicinity of operational LEO satellites. The camera view still shows the full GPS-to-LEO corridor for context.

## Verification Notes

- `node --check engine.js` passes
- `node --check app.js` passes
- App loads at `http://127.0.0.1:5173/` with no console errors
- Baseline capture/clear works on all 6 links
- CFAR detection finds peaks when objects are present
- Cross-link association filters false alarms
- Position solver converges and renders estimated positions in the scene
- Inter-node link CSI/CIR can be viewed independently
- All room presets and orbit preset load correctly
- Object animation updates detection and position estimation in real-time

## Suggested Next Steps

- Add temporal averaging (rolling buffer over N frames) for noise reduction
- Add detection timeline / history chart showing anomaly evolution over time
- Add import/export for recorded ESP32 CSI vectors
- Add MUSIC/ESPRIT superresolution for closely-spaced object separation
- Add exponential moving average baseline adaptation for drifting environments
- Train ML models (MLP for presence detection, CNN for positioning) using simulated or real ESP32 data
- Add Doppler estimation from temporal CSI phase drift for moving object characterization
