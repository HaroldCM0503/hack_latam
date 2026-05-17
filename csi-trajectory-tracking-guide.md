# CSI Trajectory Tracking — Implementation Guide for AI Coding Agents

> **Audience:** Claude Code or another coding agent working on a WiFi-CSI project that tracks a moving object (here: a ball moving against a wall) in a cluttered indoor environment.
> **Assumed prior work:** Background subtraction against a recorded baseline is already implemented.
> **Goal of this doc:** Give the agent enough physical, algorithmic, and code-level context to build the rest of the pipeline correctly on the first pass, and to refuse to take shortcuts that look reasonable but fail silently.

---

## 0. Operating principles for the agent

Before writing any code, internalize these:

1. **CSI is not an image.** Do not treat 64 subcarriers × T samples as a 2D image and slap a vision CNN on it. The axes have different physical meaning (frequency vs. time). Mixing them with 2D convolutions discards structure.
2. **Phase is not amplitude.** They have different noise models, different distortions, and different correction procedures. Always process them on separate code paths until features are extracted.
3. **A "spike" in raw CSI is almost never the target.** It is almost always: a packet retransmission, an AGC step, a CFO jump, or a NIC bug. Treat spikes as outliers until proven otherwise.
4. **Generalization is the hard part, not accuracy on the training recording.** Any metric reported on data from the same recording session as the training set is meaningless. Validation must be across sessions, ideally across days.
5. **Trajectory tracking ≠ classification.** A ball's 2D position over time is a regression problem with temporal continuity. The pipeline must end in a filter (Kalman/particle), not just a softmax.

If you (the agent) are about to violate one of these, stop and surface the issue to the user.

---

## 1. Reality check on the problem

The user's setup: a ball moves across a blank wall; the rest of the room is cluttered and static. WiFi CSI is captured between a transmitter and receiver in the same room.

**What is genuinely tractable:**

- Detecting that *something* is moving in the dynamic component.
- Estimating coarse motion direction (toward/away from the link) via Doppler shift.
- Estimating speed (via Doppler magnitude).
- 1D trajectory along the dominant link axis, in favorable geometries.

**What is hard and needs to be flagged to the user, not silently attempted:**

- Sub-wavelength 2D position estimation from commodity WiFi (the wavelength at 5 GHz is ~6 cm; at 2.4 GHz, ~12 cm). A ball with a small radar cross-section produces a perturbation that may be at or below the noise floor of single-link CSI.
- Tracking when the ball is between the static clutter and the receiver (occlusion / shadowing dominates over direct reflection).
- Tracking when the ball's reflected path length change per packet is much less than 1 mm (slow motion is invisible at commodity packet rates ~100–1000 Hz unless integrated over long windows, which blurs trajectory).

**Required physical info before coding:**

- Carrier frequency (2.4 GHz / 5 GHz / 6 GHz). Determines wavelength λ.
- Bandwidth (20 / 40 / 80 / 160 MHz). Determines range resolution ≈ c/(2·BW).
- Number of TX/RX antennas. Determines whether you have any spatial diversity.
- Packet rate (Hz). Determines maximum unambiguous Doppler ≈ packet_rate/2.
- Ball: material (RF-reflective vs. absorptive), size relative to λ, expected speed.

If the user has not specified these, **ask before building feature extraction**, because the right feature set depends on them.

---

## 2. Pipeline architecture

```
                     ┌─────────────────────────────────────────────┐
                     │ Stage 0: Capture                            │
                     │   raw CSI tensor: [T, N_sub, N_tx, N_rx]    │
                     └─────────────────────────────────────────────┘
                                         │
                                         ▼
                     ┌─────────────────────────────────────────────┐
                     │ Stage 1: Sanitization                       │
                     │   - drop pilot / null subcarriers           │
                     │   - outlier removal (Hampel)                │
                     │   - amplitude smoothing                     │
                     └─────────────────────────────────────────────┘
                                         │
                                         ▼
                     ┌─────────────────────────────────────────────┐
                     │ Stage 2: Phase handling                     │
                     │   - linear-fit detrend OR                   │
                     │   - CSI ratio between two RX antennas       │
                     └─────────────────────────────────────────────┘
                                         │
                                         ▼
                     ┌─────────────────────────────────────────────┐
                     │ Stage 3: Background subtraction (DONE)      │
                     │   - extend with adaptive / recursive update │
                     └─────────────────────────────────────────────┘
                                         │
                                         ▼
                     ┌─────────────────────────────────────────────┐
                     │ Stage 4: Dynamic component extraction       │
                     │   - PCA / SVD across subcarriers            │
                     │   - or band-pass in the motion band         │
                     └─────────────────────────────────────────────┘
                                         │
                                         ▼
                     ┌─────────────────────────────────────────────┐
                     │ Stage 5: Feature engineering                │
                     │   - Doppler spectrograms (STFT)             │
                     │   - amplitude envelope                      │
                     │   - phase-difference time series            │
                     └─────────────────────────────────────────────┘
                                         │
                                         ▼
                     ┌─────────────────────────────────────────────┐
                     │ Stage 6: Model                              │
                     │   - 1D CNN over time, OR                    │
                     │   - 2D CNN over Doppler-time, OR            │
                     │   - small Transformer for long context      │
                     │   output: per-frame position estimate       │
                     └─────────────────────────────────────────────┘
                                         │
                                         ▼
                     ┌─────────────────────────────────────────────┐
                     │ Stage 7: Tracking filter                    │
                     │   - Kalman (constant-velocity model) or     │
                     │   - particle filter for non-linear motion   │
                     └─────────────────────────────────────────────┘
```

Implement and test stage by stage. Do not skip ahead to the model.

---

## 3. Stage 1 — Sanitization

### 3.1 Drop pilot and null subcarriers

The "64 subcarriers" of an 802.11n 20 MHz channel includes:

- Null subcarriers (DC and guard bands) → no information, often exactly zero or floating noise.
- Pilot subcarriers (4 of them at fixed indices) → used for channel estimation reference; their values are not meaningful as scene measurements.

Use only **data subcarriers** (typically 52 of 64 for 802.11n 20 MHz; 56 of 64 with pilots if you really want them).

```python
# 802.11n 20 MHz: data subcarrier indices (0-indexed, FFT-shifted)
DATA_SUBCARRIERS_20MHZ = [
    -28,-27,-26,-25,-24,-23,-22,-20,-19,-18,-17,-16,-15,-14,
    -13,-12,-11,-10,-9,-8,-6,-5,-4,-3,-2,-1,
     1, 2, 3, 4, 5, 6, 8, 9,10,11,12,13,
    14,15,16,17,18,19,20,22,23,24,25,26,27,28,
]
# NOTE: the exact set depends on your CSI tool. Verify against your tool's docs.
```

This replaces the "DC component removal" idea in ad-hoc form. The DC subcarrier is already nulled by the standard; what you actually want is to drop it from your array because it contains no signal.

### 3.2 Outlier removal: Hampel filter on amplitude

Don't use a plain moving average — it smears spikes into neighbors. Hampel replaces values >k·MAD from the local median.

```python
import numpy as np
from scipy.signal import medfilt

def hampel(x, window=7, n_sigmas=3.0):
    """Hampel filter along axis 0 (time). Works per subcarrier."""
    x = np.asarray(x, dtype=float)
    k = 1.4826  # MAD → sigma for Gaussian
    rolling_median = medfilt(x, kernel_size=(window, 1))
    diff = np.abs(x - rolling_median)
    mad = k * medfilt(diff, kernel_size=(window, 1))
    mask = diff > n_sigmas * mad
    out = x.copy()
    out[mask] = rolling_median[mask]
    return out
```

### 3.3 Amplitude smoothing: Savitzky-Golay

Savitzky-Golay preserves peaks better than a moving average and does not introduce the phase lag of a causal IIR filter. Apply along the time axis, per subcarrier.

```python
from scipy.signal import savgol_filter

def smooth_amplitude(amp, window=11, poly=3):
    # amp shape: [T, N_sub] (or higher dims, axis=0 is time)
    return savgol_filter(amp, window_length=window, polyorder=poly, axis=0)
```

Tune `window` so its duration is ~10–20 ms (depends on packet rate). Too long = blurs the motion you are trying to detect.

---

## 4. Stage 2 — Phase handling

Raw CSI phase on commodity NICs is contaminated by:

- **CFO** (carrier frequency offset): adds a time-linear term.
- **SFO** (sampling frequency offset): adds a subcarrier-linear term.
- **PDD** (packet detection delay): adds a subcarrier-linear term that *changes per packet*.

This is why raw phase looks like random sawtooth ramps. **There are two viable strategies. Pick one; do not mix.**

### 4.1 Strategy A — Linear-fit detrending (single antenna)

For each packet, fit a line across subcarriers to the unwrapped phase and subtract it:

```python
def sanitize_phase_linear(csi_packet):
    """csi_packet: complex 1D array of length N_sub for a single packet."""
    phase = np.unwrap(np.angle(csi_packet))
    k = np.arange(len(phase))
    # least-squares line: phase ≈ a*k + b
    a, b = np.polyfit(k, phase, 1)
    return phase - (a * k + b)
```

Limitations:
- Removes any signal that happens to look linear across subcarriers. Most real CSI has a linear-across-frequency component, so you do remove some real information.
- Per-packet noise still leaks through; PDD jitter is not perfectly cancelled.

Use this if you have **a single RX antenna**.

### 4.2 Strategy B — CSI ratio between two RX antennas (STRONGLY PREFERRED if available)

This is the technique that makes phase usable on commodity hardware. Both antennas in the same NIC share the same RF chain, so CFO and SFO are *identical*. Dividing one antenna's CSI by another's cancels these offsets exactly:

```
H_ratio(f,t) = H_rx1(f,t) / H_rx2(f,t)
```

What survives is the relative channel between the two antennas, which is dominated by the dynamic scatterer's geometry and is stable enough for ML.

```python
def csi_ratio(csi, ref_rx=0, tgt_rx=1, eps=1e-9):
    """csi shape: [T, N_sub, N_tx, N_rx], complex.
       Returns complex tensor [T, N_sub, N_tx]."""
    num = csi[..., tgt_rx]
    den = csi[..., ref_rx]
    return num / (den + eps)
```

Then operate on the amplitude and phase of `H_ratio` instead of raw CSI. Papers to cite if asked: Zeng et al. "FarSense" (UbiComp 2019), Li et al. "IndoTrack" (UbiComp 2017).

**Do not also run linear-fit detrending on the ratio's phase.** The ratio has already removed the offsets that detrending was trying to remove.

### 4.3 Verify before continuing

After phase handling, the phase time series of a single subcarrier in a *static* recording should look like low-amplitude noise around a slowly drifting mean. If it still looks like a sawtooth, the correction failed — stop and debug rather than feeding it to the model.

---

## 5. Stage 3 — Background subtraction (already implemented, enhance it)

The user already has a baseline-subtraction feature. Two enhancements to layer on top:

### 5.1 Exponential moving average for adaptive background

A fixed pre-recorded baseline drifts: temperature, humidity, slow movement of "static" objects (curtains, plants), and NIC clock drift all change `H_s` over minutes. Use a slow EMA in parallel:

```python
def adaptive_background(csi_seq, alpha=1e-3):
    """alpha small (1e-3 to 1e-4) so the background tracks slow drift
       but not the fast dynamic motion."""
    bg = csi_seq[0].copy()
    out = np.empty_like(csi_seq)
    for t, x in enumerate(csi_seq):
        bg = (1 - alpha) * bg + alpha * x
        out[t] = x - bg
    return out
```

Run this **in addition to** the user's recorded-baseline subtraction. Recorded baseline removes the bulk static profile; EMA tracks slow drift.

### 5.2 Per-subcarrier subtraction, not global

Subtract per-subcarrier means, not a single scalar. The static channel response is frequency-selective.

### 5.3 Honest framing

ChatGPT was right that the "needle on a blank table" analogy is too optimistic. Static subtraction removes the *time-invariant* part of clutter, not the *dynamic multipath* caused by the ball reflecting off the cluttered geometry. Expect SNR improvement of maybe 6–15 dB in typical indoor settings, not infinite. Continue building the rest of the pipeline as if SNR is still limited.

---

## 6. Stage 4 — Dynamic component extraction

After background subtraction, the residual still has noise across all 52 data subcarriers. The motion-relevant signal lives in a low-dimensional subspace. Two methods:

### 6.1 PCA / SVD across subcarriers

```python
def pca_dynamic(x, n_components=3):
    """x: real-valued [T, N_sub] (use abs(H_ratio) or unwrapped phase residuals).
       Returns [T, n_components] and the components matrix."""
    mu = x.mean(axis=0, keepdims=True)
    xc = x - mu
    U, S, Vt = np.linalg.svd(xc, full_matrices=False)
    return (U[:, :n_components] * S[:n_components]), Vt[:n_components]
```

**Heuristic:** the first PC usually captures the dominant motion. The second and third often capture secondary multipath paths. Throw away components beyond ~3–5; they are noise.

### 6.2 Band-pass filter in the motion frequency band

If the ball moves with characteristic Doppler frequencies in (say) 5–40 Hz given expected speeds, band-pass each subcarrier in that range before PCA:

```python
from scipy.signal import butter, filtfilt

def bandpass(x, fs, low, high, order=4):
    b, a = butter(order, [low, high], btype='band', fs=fs)
    return filtfilt(b, a, x, axis=0)
```

`fs` is the packet rate. Use `filtfilt` (zero-phase), not `lfilter`, to avoid time-shifting your features.

---

## 7. Stage 5 — Feature engineering

For trajectory tracking specifically, the most informative feature is **Doppler over time**, not raw amplitude.

### 7.1 Doppler spectrogram (STFT)

```python
from scipy.signal import stft

def doppler_spectrogram(x, fs, nperseg=128, noverlap=112):
    """x: real or complex 1D time series (e.g. first PCA component).
       Returns frequencies, times, |Zxx|."""
    f, t, Zxx = stft(x, fs=fs, nperseg=nperseg, noverlap=noverlap,
                     return_onesided=False)  # full spectrum, signed Doppler
    return f, t, np.abs(Zxx)
```

Use `return_onesided=False` so you keep the sign of the Doppler frequency, which encodes direction (toward vs. away from the link).

Map Doppler frequency `f_d` (Hz) to radial velocity:
```
v_r = (λ / 2) * f_d
```

### 7.2 Phase difference between antennas

If you have multiple RX antennas, the phase difference between them encodes angle-of-arrival information:

```python
def phase_diff(csi, rx_a=0, rx_b=1):
    return np.angle(csi[..., rx_a] * np.conj(csi[..., rx_b]))
```

With 3 antennas (Intel 5300), you have 3 pairs → enough for crude angular tracking along one axis.

### 7.3 What features to feed the model

For a 1D model: `[doppler_spectrogram_frames, |H_ratio|_frames, phase_diff_frames]` stacked as channels.

For a 2D model: the Doppler spectrogram itself as a single-channel image.

Do **not** feed all 52 raw subcarrier amplitudes plus all 52 raw phases plus the Doppler stuff. That's 100+ channels of mostly-redundant input; the model will overfit. Pick 5–10 well-chosen channels.

---

## 8. Stage 6 — Model

### 8.1 Architecture choice

| Task | Recommended start |
|---|---|
| Presence / motion detection | Small 1D CNN, 3–4 layers |
| Coarse trajectory (left/right, fast/slow) | 1D CNN + GRU |
| Continuous 2D position | 2D CNN over Doppler spectrogram + GRU + tracking filter |
| Long-horizon trajectory | Small Transformer (4–8 layers, d_model=128) with windowed attention |

Do not start with a Transformer. Start with the smallest model that could plausibly work and only scale up if validation loss is bottlenecked by model capacity (not by data).

### 8.2 Output head and loss

Predict position as continuous coordinates, not a softmax over a grid:

```python
# pseudocode
out = nn.Linear(d_model, 2)  # (x, y) per frame
loss = F.smooth_l1_loss(out, target_xy)
```

Huber / smooth-L1 is more robust to label noise than plain MSE.

### 8.3 Data augmentation (this is where generalization comes from)

Apply at training time:

- **Subcarrier dropout:** randomly zero 10–20% of subcarriers each batch. Forces the model not to memorize specific subcarriers.
- **Time warping:** resample windows by 0.9–1.1×. Forces speed-invariance.
- **Additive Gaussian noise** on the input features, std ~ 1–5% of feature std.
- **Background remix:** if you have multiple background recordings, swap them between training examples.
- **Antenna permutation:** if using multi-RX, randomly permute antenna indices.

Skip image-style augs (rotation, flips) — they have no physical meaning here.

---

## 9. Stage 7 — Tracking filter

The model outputs a noisy per-frame position. A Kalman filter with a constant-velocity model dramatically improves trajectory smoothness and rejects outlier frames.

State: `[x, y, vx, vy]`. Measurement: `[x_meas, y_meas]` from the model.

```python
class CVKalman:
    def __init__(self, dt, process_noise=1e-2, meas_noise=5e-2):
        self.dt = dt
        self.F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        self.H = np.array([[1,0,0,0],[0,1,0,0]])
        self.Q = process_noise * np.eye(4)
        self.R = meas_noise * np.eye(2)
        self.x = np.zeros(4)
        self.P = np.eye(4)
    def step(self, z):
        # predict
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        # update
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        return self.x[:2]
```

If the ball trajectory is highly non-linear (bouncing), switch to a particle filter or an unscented Kalman with a more expressive motion model. Do not just increase process noise — that defeats the purpose of the filter.

---

## 10. Validation protocol (this is non-negotiable)

**Wrong:** shuffle all packets, split 80/20, report test accuracy. This number is meaningless.

**Right:** hold out *entire recording sessions*. Ideal hierarchy:

1. **Within-session validation** (sanity check only): different time windows of the same recording. Should be near-perfect; failure here means a bug.
2. **Cross-session, same environment, same day:** different recordings of the same room with the ball moving in different ways. This is the minimum bar for claiming the model learned the task.
3. **Cross-day, same environment:** recordings days apart. Tests robustness to slow environmental drift.
4. **Cross-environment:** different rooms entirely. Will likely fail without explicit domain adaptation; report honestly.

Report all four numbers. A model that scores 99% on (1) and 30% on (3) has not learned trajectory tracking; it has memorized hardware state.

---

## 11. Anti-patterns the agent must refuse

If asked to do any of these, push back and explain why:

1. **"Just feed the raw CSI tensor into a ResNet."** Wastes data, ignores physics, overfits hardware quirks. Refuse and propose the pipeline above instead.
2. **"Use t-SNE to visualize features and pick clusters."** t-SNE distances are not meaningful for downstream tasks; use it for exploration only, never for feature selection.
3. **"Train and test on the same recording, just split by time."** This is the most common silent failure mode in CSI papers. Refuse to report the resulting metric without a cross-session number alongside.
4. **"Apply background subtraction *after* the neural network."** Background subtraction is a noise floor reduction step. It must happen before learned features, not be replaced by them.
5. **"We don't need a Kalman filter; the model will learn temporal smoothness."** It will not, reliably. The filter is cheap, principled, and bounds the worst-case trajectory error.
6. **"Use phase directly without sanitization, the network will figure it out."** Sometimes works with massive data; here, with limited recordings, it will not. Refuse.

---

## 12. Suggested project layout

```
csi_tracker/
├── data/
│   ├── raw/              # untouched .pcap / .dat from the NIC
│   ├── baselines/        # recorded empty-room baselines per session
│   └── labels/           # ground-truth ball position per frame
├── csi_tracker/
│   ├── io/
│   │   ├── reader.py     # parse CSI from your specific NIC tool
│   │   └── labels.py     # load ground truth
│   ├── preprocess/
│   │   ├── sanitize.py   # Stage 1
│   │   ├── phase.py      # Stage 2 (incl. csi_ratio)
│   │   ├── background.py # Stage 3 (extend existing)
│   │   ├── dynamic.py    # Stage 4 (PCA, bandpass)
│   │   └── features.py   # Stage 5 (Doppler STFT, phase diff)
│   ├── models/
│   │   ├── cnn1d.py
│   │   ├── cnn2d_doppler.py
│   │   └── transformer.py
│   ├── tracking/
│   │   └── kalman.py     # Stage 7
│   ├── train.py
│   └── eval.py
├── tests/
│   ├── test_sanitize.py        # synthetic spike → Hampel removes it
│   ├── test_phase.py           # known CFO → csi_ratio cancels it
│   ├── test_background.py      # zero motion → output ~ 0
│   ├── test_doppler.py         # tone at f_d → spectrogram peak at f_d
│   └── test_kalman.py          # noisy line → recovered slope within ε
└── notebooks/
    └── 01_sanity_check.ipynb   # visualize each stage on one recording
```

Every preprocessing module gets a unit test with a synthetic signal where ground truth is known. **Do not skip these.** Most CSI pipeline bugs are silent and only surface as "the model doesn't generalize."

---

## 13. Dependencies

Minimal, well-maintained:

```
numpy
scipy            # signal.savgol_filter, signal.stft, signal.butter
scikit-learn     # PCA, baseline classifiers for sanity checks
torch            # models
matplotlib       # debugging plots only; do not use in production loop
```

NIC-specific CSI parsers (pick one based on hardware):

- Intel 5300 → `csiread` (Python) or the Linux 802.11n CSI tool.
- Atheros (AR9580 etc.) → Atheros CSI tool.
- Broadcom (Nexmon) → `nexmon_csi` + `csiread`.
- ESP32 → ESP32-CSI-Tool.
- ASUS PCE-AC68 / various → Picoscenes.

Verify the parser's subcarrier indexing convention against its docs *before* applying the data-subcarrier mask in §3.1; conventions differ.

---

## 14. Order of operations for the agent

When picking up this project:

1. Confirm the hardware/bandwidth/antenna parameters in §1.
2. Run `notebooks/01_sanity_check.ipynb` on a known recording and visually verify each stage's output looks right.
3. Implement and unit-test each preprocessing module in isolation against synthetic signals before integrating.
4. Train the smallest viable model (1D CNN) end-to-end on one session. Confirm it overfits (training loss → ~0). If it does not, there is a bug upstream; fix it before scaling.
5. Add cross-session validation. Expect a large gap on first attempt; close it with augmentation (§8.3) and CSI ratio (§4.2).
6. Add Kalman filter on top of model output.
7. Only after all of the above: scale to a larger model or add domain adaptation.

If at any step the agent finds itself patching results downstream to compensate for an unfixed upstream bug, stop and fix the upstream bug.

---

## 15. What to escalate to the user

Surface these decisions explicitly rather than choosing silently:

- Carrier frequency and bandwidth (affects feature design).
- Whether multiple RX antennas are available (determines §4.1 vs §4.2).
- Expected ball speed range (sets band-pass cutoffs and STFT window).
- Whether ground-truth ball position is available per frame, or only sparse anchors.
- Acceptable latency budget (offline batch processing vs. real-time).

Each of these can change the pipeline materially. Ask, don't guess.
