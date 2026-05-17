"""UDP receiver for the WiFi-CSI passive sensing system.

Each of the 3 Rx ESP32s sends one JSON line over UDP per CSI frame.

Current schema (esp-radar based firmware, preferred):

    { "rx":       1,            // Rx node id 1/2/3
      "t":        1234567,      // ESP32-side ms timestamp
      "motion":   0.0123,       // esp-radar waveform_jitter  (fast-varying)
      "presence": 0.0008        // esp-radar waveform_wander  (slow-varying)
    }

The `motion` field is the bistatic-Fresnel score curve consumed downstream.

Legacy schema (pre-esp-radar firmware, still parsed for back-compat):

    { "rx": 1, "t": ..., "rssi": -42.0,
      "amp":   [a0, a1, ...],   // per-subcarrier amplitude
      "score": 0.42             // optional, computed on-device
    }

If neither "motion" nor "score" is present, we fall back to computing a
score on the laptop from "amp" via the rolling baseline tracker.
"""

from __future__ import annotations

import json
import math
import socket
import time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Generator, List, Optional

from config import (
    CSIFrame,
    EVENT_WINDOW_MS,
    MIN_FRAMES_PER_RX,
    MOTION_SCORE_TRIGGER,
    RX_POSITIONS,
    SUBCARRIERS,
    UDP_BIND_HOST,
    UDP_PORT,
)


# ---------------------------------------------------------------------------
# Event container
# ---------------------------------------------------------------------------
@dataclass
class Event:
    """A burst of CSI frames around one moving-object transit."""
    t_start_us:    int
    frames_by_rx:  dict = field(default_factory=lambda: defaultdict(list))

    def add(self, frame: CSIFrame) -> None:
        self.frames_by_rx[frame.rx_id].append(frame)

    def total_frames(self) -> int:
        return sum(len(v) for v in self.frames_by_rx.values())

    def is_complete(self, now_us: int) -> bool:
        return (now_us - self.t_start_us) > EVENT_WINDOW_MS * 1000

    def is_usable(self) -> bool:
        if len(self.frames_by_rx) < 2:               # need >=2 Rx for any geometry
            return False
        usable_rx = sum(1 for v in self.frames_by_rx.values() if len(v) >= MIN_FRAMES_PER_RX)
        return usable_rx >= 2


# ---------------------------------------------------------------------------
# Per-Rx rolling baseline + motion-score computation
# ---------------------------------------------------------------------------
class _BaselineTracker:
    """Maintains, per Rx, a rolling baseline of CSI amplitudes and computes
    a LINEAR motion score for each incoming frame:
        score = ||amp - mean(amp_baseline)|| / ||mean(amp_baseline)||
    ~0 in a quiet scene; spikes to 0.3-1.0+ when something moves near the
    bistatic Tx-Rx line.
    """

    BASELINE_LEN = 60                                # frames of history per Rx (~ 1-2 s at 30 Hz)

    def __init__(self) -> None:
        self._history: Dict[int, Deque[List[float]]] = defaultdict(
            lambda: deque(maxlen=self.BASELINE_LEN)
        )

    def score(self, rx_id: int, amp: List[float]) -> float:
        hist = self._history[rx_id]
        if len(hist) < 10:
            hist.append(list(amp))
            return 0.0                                # not yet calibrated
        # Mean amplitude per subcarrier
        n = len(amp)
        mean = [0.0] * n
        for snap in hist:
            for i in range(min(n, len(snap))):
                mean[i] += snap[i]
        inv = 1.0 / len(hist)
        for i in range(n):
            mean[i] *= inv
        diff_sq = 0.0
        base_sq = 0.0
        for i in range(n):
            d = amp[i] - mean[i]
            diff_sq += d * d
            base_sq += mean[i] * mean[i]
        hist.append(list(amp))
        if base_sq <= 1e-9:
            return 0.0
        return math.sqrt(diff_sq) / math.sqrt(base_sq)


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------
def parse_line(line: str, t_arrival_us: int, baseline: _BaselineTracker) -> Optional[CSIFrame]:
    line = line.strip()
    if not line or not line.startswith("{"):
        return None
    try:
        obj  = json.loads(line)
        rx   = int(obj["rx"])
        rssi = float(obj.get("rssi", -90.0))
        amp  = [float(x) for x in obj.get("amp", [])]
        real = [int(x) for x in obj.get("real", [])]
        imag = [int(x) for x in obj.get("imag", [])]

        # Score priority:
        #   1. "motion"  — esp-radar waveform_jitter (current firmware)
        #   2. "score"   — legacy on-device EWMA score
        #   3. computed locally from "amp"
        if "motion" in obj:
            score = float(obj["motion"])
        elif "score" in obj:
            score = float(obj["score"])
        else:
            score = baseline.score(rx, amp) if amp else 0.0

        return CSIFrame(
            rx_id = rx,
            t_us  = t_arrival_us,
            rssi  = rssi,
            score = score,
            amp   = amp,
            real  = real,
            imag  = imag,
        )
    except (ValueError, KeyError):
        return None


# ---------------------------------------------------------------------------
# Event windowing
# ---------------------------------------------------------------------------
class _EventBuilder:
    def __init__(self) -> None:
        self.current: Optional[Event] = None

    def push(self, frame: CSIFrame) -> Optional[Event]:
        if self.current is None:
            if frame.score >= MOTION_SCORE_TRIGGER:
                self.current = Event(t_start_us=frame.t_us)
                self.current.add(frame)
            return None

        if frame.t_us - self.current.t_start_us <= EVENT_WINDOW_MS * 1000:
            self.current.add(frame)
            return None

        finished = self.current
        self.current = Event(t_start_us=frame.t_us) if frame.score >= MOTION_SCORE_TRIGGER else None
        if self.current is not None:
            self.current.add(frame)
        return finished if finished.is_usable() else None

    def flush_if_expired(self, now_us: int) -> Optional[Event]:
        if self.current is None or not self.current.is_complete(now_us):
            return None
        finished = self.current
        self.current = None
        return finished if finished.is_usable() else None


# ---------------------------------------------------------------------------
# UDP transport
# ---------------------------------------------------------------------------
def open_udp(host: str = UDP_BIND_HOST, port: int = UDP_PORT) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.settimeout(0.05)
    return sock


def stream_events_udp(sock: socket.socket) -> Generator[Event, None, None]:
    """Block-yield Events from CSI packets sent by the three Rx."""
    baseline = _BaselineTracker()
    builder  = _EventBuilder()
    while True:
        try:
            data, _ = sock.recvfrom(8192)             # CSI payload can be ~1 kB
        except socket.timeout:
            finished = builder.flush_if_expired(int(time.monotonic() * 1e6))
            if finished is not None:
                yield finished
            continue
        except OSError:
            continue

        t_arrival_us = int(time.monotonic() * 1e6)
        try:
            line = data.decode("utf-8", errors="ignore")
            print(f"[DEBUG] RAW UDP: {line[:100]}...") # Print first 100 chars to check data flow
        except Exception:
            continue
        frame = parse_line(line, t_arrival_us, baseline)
        if frame is None:
            continue
            
        # Print every incoming packet continuously
        print(f"[{frame.t_seconds():.3f}] Rx{frame.rx_id} | RSSI: {frame.rssi} dBm | Score: {frame.score:.4f} | Subcarriers: {len(frame.amp)}")
        
        finished = builder.push(frame)
        if finished is not None:
            yield finished
