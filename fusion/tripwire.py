"""Tripwire-style transit detector for a 3-Rx bistatic CSI link array.

The math half of the math-vs-ML choice for the car-transit task: per-Rx
adaptive baseline + threshold-crossing event detection + cross-Rx
coincidence + speed-from-transit-time. No training data, no GPU, no
generalisation problem — all knobs are interpretable.

Pipeline per Rx
---------------
1. RECORDED BASELINE  B_rx           captured during a quiet calibration
                                     window (3 s by default). Snapshot of
                                     the room's RF fingerprint with no
                                     moving objects.
2. ADAPTIVE EMA DRIFT E_rx           slow exponential moving average that
                                     tracks environmental drift (heating
                                     asphalt, parked cars leaving, etc.).
                                     Update is PAUSED during FIRING so a
                                     perturbation doesn't drag the
                                     baseline.
3. SCORE  s_rx(t) =                  normalised perturbation magnitude in
   ||amp(t) - (B_rx + E_rx)|| /      dimensionless units. ~0 in quiet
   ||B_rx||                          state, spikes above threshold on
                                     transit.
4. HYSTERESIS  T_high / T_low        score must cross T_high to enter
                                     FIRING; must fall below T_low to exit.
                                     Prevents chattering at the threshold.
5. REFRACTORY  refractory_sec        after FIRING ends, ignore for
                                     refractory_sec so one car doesn't
                                     fire twice.

Cross-Rx coincidence
--------------------
Recent TripwireEvents are kept in a short ring. When a new event fires we
look for events from OTHER Rx within ``coincidence_sec``. If
``>= min_rx_fires`` distinct Rx are present in the window, we emit a
TransitEvent stamped with all participating Rx events.

Speed and direction
-------------------
If you provide pairwise ``link_spacings_m`` (e.g. ``{(1,2): 1.5, (2,3): 1.5}``),
each consecutive pair in the transit produces a pairwise speed estimate
``v_{i,j} = d_{i,j} / (t_peak_j - t_peak_i)`` and the TransitEvent reports
the median across pairs.

Knobs you'll actually tune
--------------------------
- ema_alpha       1e-4 default; smaller = baseline tracks drift slower.
                  At 100 Hz, alpha=1e-4 -> half-life ~70 s.
- threshold_high  0.20 default; raise if you see false positives.
- threshold_low   0.08 default; usually ~0.4 x threshold_high.
- refractory_sec  0.2 default; one car = one event.
- coincidence_sec 2.0 default; a car traverses 3 tripwires in <2 s at
                  typical urban speeds.
- min_rx_fires    2 default; 2/3 Rx confirms a real transit. Set to 3 if
                  your link spacing is tight and a real car should always
                  hit all three.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


# ===========================================================================
# Events
# ===========================================================================
@dataclass
class TripwireEvent:
    """One Rx's threshold-crossing event."""
    rx_id:      int
    t_start:    float       # monotonic seconds when score first crossed T_high
    t_peak:     float       # when score reached its max
    t_end:      float       # when score dropped back below T_low
    peak_score: float

    @property
    def duration(self) -> float:
        return self.t_end - self.t_start


@dataclass
class TransitEvent:
    """A cross-Rx coincidence: at least ``min_rx_fires`` distinct Rx fired
    within the coincidence window. This is what your downstream pipeline
    will typically log as 'one car passed'."""
    t:          float                              # earliest tripwire peak time
    rx_events:  Dict[int, TripwireEvent]
    direction:  Tuple[int, ...]                    # rx ids in time order
    speed_mps:  Optional[float] = None             # if geometry was provided
    pair_speeds: List[Tuple[int, int, float]] = field(default_factory=list)

    def to_jsonable(self) -> dict:
        return {
            "t":          round(self.t, 6),
            "direction":  list(self.direction),
            "speed_mps":  None if self.speed_mps is None else round(self.speed_mps, 3),
            "pair_speeds": [(a, b, round(v, 3)) for a, b, v in self.pair_speeds],
            "rx_events": {
                str(rx): {
                    "t_start":    round(ev.t_start, 6),
                    "t_peak":     round(ev.t_peak, 6),
                    "t_end":      round(ev.t_end, 6),
                    "peak_score": round(ev.peak_score, 4),
                } for rx, ev in self.rx_events.items()
            },
        }


# ===========================================================================
# Per-Rx state machine
# ===========================================================================
class _RxState:
    """Per-Rx baseline + state machine. Internal — used by TripwireDetector."""

    IDLE     = 0
    FIRING   = 1
    COOLDOWN = 2

    def __init__(self, n_subc: int, ema_alpha: float,
                 t_high: float, t_low: float, refractory_sec: float):
        self.n_subc           = n_subc
        self.recorded_base: Optional[np.ndarray] = None
        self.ema_drift        = np.zeros(n_subc, dtype=np.float32)
        self.ema_alpha        = float(ema_alpha)
        # snapshot-capture state
        self._cap_active      = False
        self._cap_sum         = np.zeros(n_subc, dtype=np.float64)
        self._cap_count       = 0
        self._cap_start       = 0.0
        self._cap_dur         = 0.0
        # state machine
        self.t_high           = float(t_high)
        self.t_low            = float(t_low)
        self.refractory_sec   = float(refractory_sec)
        self.state            = _RxState.IDLE
        self._fire_t_start    = 0.0
        self._fire_t_peak     = 0.0
        self._fire_peak       = 0.0
        self._cooldown_until  = 0.0

    # ---- baseline control --------------------------------------------------
    def start_capture(self, duration_sec: float, now: float) -> None:
        self._cap_active = True
        self._cap_sum.fill(0)
        self._cap_count  = 0
        self._cap_start  = now
        self._cap_dur    = float(duration_sec)
        # While capturing we DON'T fire — the score depends on a baseline we
        # don't have yet. Reset state machine to IDLE just in case.
        self.state = _RxState.IDLE

    def clear_baseline(self) -> None:
        self.recorded_base = None
        self.ema_drift.fill(0)
        self._cap_active   = False
        self.state         = _RxState.IDLE

    @property
    def has_baseline(self) -> bool:
        return self.recorded_base is not None

    @property
    def capture_active(self) -> bool:
        return self._cap_active

    @property
    def capture_remaining_sec(self) -> float:
        if not self._cap_active:
            return 0.0
        return max(0.0, self._cap_dur - (time.monotonic() - self._cap_start))

    # ---- per-sample step ---------------------------------------------------
    def push(self, t_now: float, amp: np.ndarray) -> Tuple[float, Optional[TripwireEvent]]:
        """Process one CSI amplitude sample. Returns ``(score, event_or_None)``.
        The returned event (if any) has ``rx_id=-1`` and the caller is expected
        to fill it in."""
        n = min(len(amp), self.n_subc)
        amp32 = amp[:n].astype(np.float32, copy=False)

        # 1) baseline capture accumulator
        if self._cap_active:
            self._cap_sum[:n] += amp32.astype(np.float64)
            self._cap_count   += 1
            if (t_now - self._cap_start) >= self._cap_dur and self._cap_count > 0:
                self.recorded_base = (self._cap_sum / self._cap_count).astype(np.float32)
                self.ema_drift.fill(0)
                self._cap_active = False
            # don't compute a score until the baseline is final
            return 0.0, None

        if self.recorded_base is None:
            return 0.0, None

        # 2) score against recorded baseline + EMA drift
        base = self.recorded_base[:n] + self.ema_drift[:n]
        diff_norm = float(np.linalg.norm(amp32 - base))
        base_norm = float(np.linalg.norm(self.recorded_base[:n]))
        score = (diff_norm / base_norm) if base_norm > 1e-6 else 0.0

        # 3) state machine
        event = None
        if self.state == _RxState.IDLE:
            # EMA only updates while IDLE so a perturbation doesn't drag it.
            self.ema_drift[:n] += self.ema_alpha * (
                amp32 - self.recorded_base[:n] - self.ema_drift[:n]
            )
            if score > self.t_high:
                self.state          = _RxState.FIRING
                self._fire_t_start  = t_now
                self._fire_t_peak   = t_now
                self._fire_peak     = score
        elif self.state == _RxState.FIRING:
            if score > self._fire_peak:
                self._fire_peak   = score
                self._fire_t_peak = t_now
            if score < self.t_low:
                event = TripwireEvent(
                    rx_id      = -1,
                    t_start    = self._fire_t_start,
                    t_peak     = self._fire_t_peak,
                    t_end      = t_now,
                    peak_score = self._fire_peak,
                )
                self.state           = _RxState.COOLDOWN
                self._cooldown_until = t_now + self.refractory_sec
        elif self.state == _RxState.COOLDOWN:
            if t_now > self._cooldown_until:
                self.state = _RxState.IDLE

        return score, event

    # ---- pre-computed score path (no baseline needed) ----------------------
    def push_score(self, t_now: float, score: float) -> Tuple[float, Optional[TripwireEvent]]:
        """Run the state machine on a pre-computed score (e.g. esp-radar
        ``motion`` field).  Skips baseline capture, EMA drift, and norm
        computation — just hysteresis + event emission."""
        event = None
        if self.state == _RxState.IDLE:
            if score > self.t_high:
                self.state          = _RxState.FIRING
                self._fire_t_start  = t_now
                self._fire_t_peak   = t_now
                self._fire_peak     = score
        elif self.state == _RxState.FIRING:
            if score > self._fire_peak:
                self._fire_peak   = score
                self._fire_t_peak = t_now
            if score < self.t_low:
                event = TripwireEvent(
                    rx_id      = -1,
                    t_start    = self._fire_t_start,
                    t_peak     = self._fire_t_peak,
                    t_end      = t_now,
                    peak_score = self._fire_peak,
                )
                self.state           = _RxState.COOLDOWN
                self._cooldown_until = t_now + self.refractory_sec
        elif self.state == _RxState.COOLDOWN:
            if t_now > self._cooldown_until:
                self.state = _RxState.IDLE
        return score, event


# ===========================================================================
# Top-level multi-Rx detector
# ===========================================================================
class TripwireDetector:
    """Multi-Rx tripwire detector with cross-Rx coincidence + (optional)
    speed estimation."""

    STATE_NAMES = {_RxState.IDLE: "idle", _RxState.FIRING: "FIRE",
                   _RxState.COOLDOWN: "cool"}

    def __init__(self,
                 rx_ids: List[int],
                 n_subc: int = 64,
                 ema_alpha: float = 1e-4,
                 threshold_high: float = 0.20,
                 threshold_low: float = 0.08,
                 refractory_sec: float = 0.2,
                 coincidence_sec: float = 2.0,
                 min_rx_fires: int = 2,
                 link_spacings_m: Optional[Dict[Tuple[int, int], float]] = None):
        self.rx_ids          = list(rx_ids)
        self.states: Dict[int, _RxState] = {
            r: _RxState(n_subc, ema_alpha, threshold_high, threshold_low,
                        refractory_sec)
            for r in self.rx_ids
        }
        self._t_high         = float(threshold_high)
        self._t_low          = float(threshold_low)
        self.coincidence_sec = float(coincidence_sec)
        self.min_rx_fires    = int(min_rx_fires)
        self.link_spacings_m = link_spacings_m or {}
        # diagnostics, useful for the on-screen status line
        self.latest_score:  Dict[int, float] = {r: 0.0 for r in self.rx_ids}
        self.latest_state:  Dict[int, int]   = {r: _RxState.IDLE for r in self.rx_ids}
        # log of recent fired transits (kept for the UI; cross-Rx coincidence
        # logic doesn't use this — it uses _pending_events below).
        self.recent_events: deque = deque(maxlen=64)
        # Pending transit assembly: when the first Rx fires we open a window
        # `coincidence_sec` wide; all subsequent Rx fires that fall inside
        # join the same pending transit. We commit (emit) only AFTER the
        # window expires, so a 3-Rx transit emits ONCE, not three times.
        self._pending_events: Dict[int, TripwireEvent] = {}
        self._pending_t_open: Optional[float]          = None
        # Lock around all mutable state so push() (called from the UDP RX
        # thread) and the control methods + status readers (called from the
        # main thread) don't race.
        self._lock = threading.Lock()

    # ---- live-tunable thresholds (matplotlib/cv2 trackbars hit these) ------
    @property
    def threshold_high(self) -> float: return self._t_high
    @threshold_high.setter
    def threshold_high(self, v: float):
        self._t_high = float(v)
        for s in self.states.values(): s.t_high = self._t_high

    @property
    def threshold_low(self) -> float: return self._t_low
    @threshold_low.setter
    def threshold_low(self, v: float):
        self._t_low = float(v)
        for s in self.states.values(): s.t_low = self._t_low

    # ---- baseline control --------------------------------------------------
    def start_capture_all(self, duration_sec: float) -> None:
        with self._lock:
            now = time.monotonic()
            for state in self.states.values():
                state.start_capture(duration_sec, now)
            # also drop any in-flight pending transit — capture is a "reset"
            self._pending_events = {}
            self._pending_t_open = None

    def clear_baselines(self) -> None:
        with self._lock:
            for state in self.states.values():
                state.clear_baseline()
            self._pending_events = {}
            self._pending_t_open = None

    def baseline_status(self) -> Dict[int, str]:
        with self._lock:
            out = {}
            for rx, state in self.states.items():
                if state.capture_active:
                    out[rx] = f"cap {state.capture_remaining_sec:.1f}s"
                elif state.has_baseline:
                    out[rx] = "ready"
                else:
                    out[rx] = "none"
            return out

    def snapshot_scores(self) -> Dict[int, float]:
        """Thread-safe copy of the latest per-Rx score for UI display."""
        with self._lock:
            return dict(self.latest_score)

    # ---- main entry point --------------------------------------------------
    def push(self, rx_id: int, t_mono: float, amp_vec) -> Optional[TransitEvent]:
        """Process one CSI sample from one Rx. Returns a TransitEvent when
        a cross-Rx coincidence finishes assembling at-or-after this sample,
        else None."""
        with self._lock:
            if rx_id not in self.states:
                return self._maybe_commit_pending(t_mono)
            amp = np.asarray(amp_vec, dtype=np.float32)
            score, ev = self.states[rx_id].push(t_mono, amp)
            self.latest_score[rx_id] = score
            self.latest_state[rx_id] = self.states[rx_id].state
            if ev is not None:
                ev.rx_id = rx_id
                self.recent_events.append(ev)
                self._add_to_pending(ev)
            return self._maybe_commit_pending(t_mono)

    def push_score(self, rx_id: int, t_mono: float, score: float) -> Optional[TransitEvent]:
        """Process a pre-computed perturbation score from one Rx (e.g.
        esp-radar ``motion`` field).  No baseline capture needed."""
        with self._lock:
            if rx_id not in self.states:
                return self._maybe_commit_pending(t_mono)
            sc, ev = self.states[rx_id].push_score(t_mono, score)
            self.latest_score[rx_id] = sc
            self.latest_state[rx_id] = self.states[rx_id].state
            if ev is not None:
                ev.rx_id = rx_id
                self.recent_events.append(ev)
                self._add_to_pending(ev)
            return self._maybe_commit_pending(t_mono)

    # ---- pending transit assembly -----------------------------------------
    def _add_to_pending(self, ev: TripwireEvent) -> None:
        if self._pending_t_open is None:
            # New transit window opens with the first fired Rx.
            self._pending_t_open = ev.t_peak
            self._pending_events = {ev.rx_id: ev}
            return
        # Window already open: this Rx joins. If it already had an event in
        # the pending bag, keep whichever is closer in time to the window-open
        # event (covers the rare case of two close-in-time fires from one Rx).
        cur = self._pending_events.get(ev.rx_id)
        if cur is None or abs(ev.t_peak - self._pending_t_open) < \
                          abs(cur.t_peak - self._pending_t_open):
            self._pending_events[ev.rx_id] = ev

    def _maybe_commit_pending(self, t_now: float) -> Optional[TransitEvent]:
        """Emit the assembled transit IF the coincidence window has expired
        since the first Rx fired. Otherwise keep collecting."""
        if self._pending_t_open is None:
            return None
        if (t_now - self._pending_t_open) < self.coincidence_sec:
            return None
        events                = self._pending_events
        self._pending_events  = {}
        self._pending_t_open  = None
        if len(events) < self.min_rx_fires:
            return None
        return self._build_transit(events)

    def _build_transit(self, related: Dict[int, TripwireEvent]) -> TransitEvent:
        ordered   = sorted(related.items(), key=lambda kv: kv[1].t_peak)
        direction = tuple(rx for rx, _ in ordered)
        t0        = ordered[0][1].t_peak
        # pairwise speeds (only between consecutive Rx in time order)
        pair_speeds: List[Tuple[int, int, float]] = []
        for i in range(len(ordered) - 1):
            (rx_a, ev_a), (rx_b, ev_b) = ordered[i], ordered[i + 1]
            dt = ev_b.t_peak - ev_a.t_peak
            if dt <= 0:
                continue
            d = self.link_spacings_m.get((rx_a, rx_b),
                self.link_spacings_m.get((rx_b, rx_a)))
            if d is not None:
                pair_speeds.append((rx_a, rx_b, d / dt))
        speed_mps = float(np.median([s for _, _, s in pair_speeds])) \
                    if pair_speeds else None
        return TransitEvent(t=t0, rx_events=related, direction=direction,
                            speed_mps=speed_mps, pair_speeds=pair_speeds)
