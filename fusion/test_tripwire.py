"""Synthetic-signal test for fusion/tripwire.py.

Runs three scenarios on a 3-Rx detector with fully synthetic CSI streams:

  1. Quiet scene with only noise -> no tripwire events fire.
  2. A 'transit' modelled as a Gaussian amplitude bump injected on each Rx
     at known times -> exactly one TransitEvent fires, in the expected
     direction, with speed within tolerance of the ground truth.
  3. Reversed direction -> direction tuple is reversed.

Run as a script:
    python test_tripwire.py
"""

from __future__ import annotations

import numpy as np
from tripwire import TripwireDetector


SAMPLE_RATE_HZ = 100
N_SUBC         = 64
RNG            = np.random.default_rng(42)


def synth_quiet(t_sec: float, base_level: float = 30.0,
                noise_std: float = 0.5) -> np.ndarray:
    """One CSI vector at time `t_sec`. Flat baseline + small Gaussian noise."""
    return base_level + RNG.normal(0.0, noise_std, size=N_SUBC).astype(np.float32)


def synth_transit_bump(t_sec: float, t_center: float, sigma_sec: float,
                       amplitude: float, base_level: float = 30.0,
                       noise_std: float = 0.5) -> np.ndarray:
    """Baseline + Gaussian-time-windowed broadband perturbation at t_center."""
    gain = amplitude * float(np.exp(-((t_sec - t_center) ** 2) / (2 * sigma_sec ** 2)))
    perturb = gain * (1.0 + 0.3 * RNG.standard_normal(N_SUBC).astype(np.float32))
    return base_level + perturb + RNG.normal(0.0, noise_std, size=N_SUBC).astype(np.float32)


def _make_detector(link_spacings_m=None):
    return TripwireDetector(
        rx_ids        = [1, 2, 3],
        n_subc        = N_SUBC,
        ema_alpha     = 1e-3,                # fast for the short test
        threshold_high = 0.20,
        threshold_low  = 0.08,
        refractory_sec = 0.3,
        coincidence_sec = 2.0,
        min_rx_fires    = 2,
        link_spacings_m = link_spacings_m,
    )


def _capture_baseline(detector, t0=0.0, duration_sec=1.0):
    """Drive the detector with quiet samples while a baseline capture is running."""
    detector.start_capture_all(duration_sec)
    # We need each Rx's _RxState to see `duration_sec` worth of quiet samples
    # AND see a t_mono >= cap_start+duration_sec at the END so it finalises.
    dt = 1.0 / SAMPLE_RATE_HZ
    n  = int(duration_sec * SAMPLE_RATE_HZ) + 5
    # ALSO need to override their internal `cap_start` to use our fake clock.
    for state in detector.states.values():
        state._cap_start = t0
    for k in range(n):
        t = t0 + k * dt
        for rx in detector.rx_ids:
            detector.push(rx, t, synth_quiet(t))
    assert all(s.has_baseline for s in detector.states.values()), \
        "baseline capture failed to finalise"


def test_quiet_no_events():
    det = _make_detector()
    _capture_baseline(det, t0=0.0)
    dt = 1.0 / SAMPLE_RATE_HZ
    events = 0
    for k in range(SAMPLE_RATE_HZ * 5):           # 5 seconds of quiet
        t = 5.0 + k * dt
        for rx in det.rx_ids:
            if det.push(rx, t, synth_quiet(t)) is not None:
                events += 1
    assert events == 0, f"quiet scene fired {events} transit events"
    print("[ok] quiet scene -> no events")


def test_forward_transit():
    # 1.5 m spacing between consecutive Rx; ground-truth speed 5 m/s
    # => 0.3 s between consecutive peaks.
    link_spacings = {(1, 2): 1.5, (2, 3): 1.5}
    det = _make_detector(link_spacings_m=link_spacings)
    _capture_baseline(det, t0=0.0)

    truth_speed = 5.0
    t_peaks = {1: 10.0, 2: 10.0 + 1.5 / truth_speed, 3: 10.0 + 3.0 / truth_speed}
    sigma   = 0.10
    bump_amp = 12.0          # big enough to clearly cross threshold

    dt    = 1.0 / SAMPLE_RATE_HZ
    transits = []
    # The detector buffers events for the full coincidence window before
    # committing, so we need to keep pushing samples past the last peak +
    # coincidence_sec for the transit to actually emit.
    t_start = t_peaks[1] - 1.0
    t_end   = t_peaks[3] + det.coincidence_sec + 1.0
    n = int((t_end - t_start) / dt) + 1
    for k in range(n):
        t = t_start + k * dt
        for rx in det.rx_ids:
            amp = synth_transit_bump(t, t_peaks[rx], sigma, bump_amp)
            ev = det.push(rx, t, amp)
            if ev is not None:
                transits.append(ev)

    assert len(transits) == 1, f"expected 1 transit, got {len(transits)}"
    ev = transits[0]
    assert ev.direction == (1, 2, 3), f"expected dir (1,2,3), got {ev.direction}"
    assert ev.speed_mps is not None and abs(ev.speed_mps - truth_speed) / truth_speed < 0.1, \
        f"expected speed ~{truth_speed} m/s, got {ev.speed_mps}"
    print(f"[ok] forward transit -> dir={ev.direction}  speed={ev.speed_mps:.2f} m/s")


def test_reverse_transit():
    link_spacings = {(1, 2): 1.5, (2, 3): 1.5}
    det = _make_detector(link_spacings_m=link_spacings)
    _capture_baseline(det, t0=0.0)

    truth_speed = 7.5
    t_peaks = {3: 10.0, 2: 10.0 + 1.5 / truth_speed, 1: 10.0 + 3.0 / truth_speed}
    sigma, bump_amp = 0.10, 12.0

    dt = 1.0 / SAMPLE_RATE_HZ
    transits = []
    t_start = min(t_peaks.values()) - 1.0
    t_end   = max(t_peaks.values()) + det.coincidence_sec + 1.0
    n = int((t_end - t_start) / dt) + 1
    for k in range(n):
        t = t_start + k * dt
        for rx in det.rx_ids:
            amp = synth_transit_bump(t, t_peaks[rx], sigma, bump_amp)
            ev = det.push(rx, t, amp)
            if ev is not None:
                transits.append(ev)

    assert len(transits) == 1, f"expected 1 transit, got {len(transits)}"
    ev = transits[0]
    assert ev.direction == (3, 2, 1), f"expected dir (3,2,1), got {ev.direction}"
    assert ev.speed_mps is not None and abs(ev.speed_mps - truth_speed) / truth_speed < 0.1, \
        f"expected speed ~{truth_speed} m/s, got {ev.speed_mps}"
    print(f"[ok] reverse transit -> dir={ev.direction}  speed={ev.speed_mps:.2f} m/s")


if __name__ == "__main__":
    test_quiet_no_events()
    test_forward_transit()
    test_reverse_transit()
    print("\nall tripwire tests passed")
