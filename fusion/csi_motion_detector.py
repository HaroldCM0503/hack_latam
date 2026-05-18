"""CSI motion detector — camera overlay + amplitude-based movement detection.

Replicates the camera-view + TX→RX link-line visualisation from
``webcam_yolo.py`` and adds a simple motion-detection algorithm that
operates on the CSI amplitude stream from each Rx:

Algorithm
---------
1. Keep a rolling window of the last ``WINDOW_SEC`` seconds of amplitude
   vectors for each Rx.
2. Compute a moving average (mean) over that window.
3. For every new measurement, compute the RMS distance between the new
   amplitude and the current moving average.
4. If that RMS exceeds ``RMS_THRESHOLD`` for ``CONSECUTIVE_HITS``
   consecutive measurements, print **"ok!"** to the console (once per
   burst — it resets after the burst breaks).

Usage
-----
    python csi_motion_detector.py
    python csi_motion_detector.py --cam 1 --rx-ids 1,2
    python csi_motion_detector.py --rms-threshold 5.0 --consecutive 15

Keys (window focused)
---------------------
    Q - quit
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import threading
import time
from collections import deque
from datetime import datetime
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

from config import UDP_BIND_HOST, UDP_PORT


# ---- Defaults --------------------------------------------------------------
WINDOW_SEC        = 10.0     # rolling window for the moving average (seconds)
RMS_THRESHOLD     = 3.0      # RMS above moving average to count as "hit"
CONSECUTIVE_HITS  = 10       # how many consecutive hits before printing "ok!"
TIMESERIES_LEN    = 600      # number of samples kept for the chart
MAX_SUBC          = 64       # 802.11n 20 MHz CSI buffer width
AMP_VMAX          = 80.0     # initial Y-axis ceiling
CHART_HEIGHT_PER_RX = 160    # pixel height of each Rx subplot

# ---- Car / signal-drop detection defaults ----------------------------------
DROP_THRESHOLD    = -5.0     # mean relative deviation (%) below which = drop
COHERENCE_MIN     = 0.60     # fraction of subcarriers that must drop together
DROP_CONSECUTIVE  = 1        # consecutive drop-hits needed to confirm
DROP_HOLD_SEC     = 1.5      # seconds the drop flag stays True after last hit


# ===========================================================================
# Per-Rx motion analyser
# ===========================================================================
class RxMotionAnalyser:
    """Keeps a time-windowed ring of amplitude vectors for one Rx and
    computes the rolling-average + per-sample RMS deviation described in
    the module docstring."""

    def __init__(self, rx_id: int, window_sec: float, rms_threshold: float,
                 consecutive_hits: int, ts_len: int = TIMESERIES_LEN,
                 drop_threshold: float = DROP_THRESHOLD,
                 coherence_min: float = COHERENCE_MIN,
                 drop_consecutive: int = DROP_CONSECUTIVE,
                 drop_hold_sec: float = DROP_HOLD_SEC):
        self.rx_id           = rx_id
        self.window_sec      = window_sec
        self.rms_threshold   = rms_threshold
        self.consecutive_hits = consecutive_hits

        # Ring of (t_mono, amp_array) tuples
        self._ring: deque = deque()

        # Running sum for the moving average (avoids re-summing every time)
        self._running_sum: Optional[np.ndarray] = None
        self._running_count: int = 0

        # Consecutive-hit counter and "already fired" flag
        self._hit_count = 0
        self._fired     = False       # True once "ok!" printed; resets on miss

        # Diagnostics exposed for overlay
        self.last_rms: float = 0.0
        self.last_avg: Optional[np.ndarray] = None

        # RMS history for the timeseries chart (fixed-length ring)
        self.rms_history = np.zeros(ts_len, dtype=np.float32)

        # Per-subcarrier amplitude history for the CSI timeseries chart
        # Shape: (ts_len, MAX_SUBC) — each row is one CSI snapshot
        self.amp_history = np.zeros((ts_len, MAX_SUBC), dtype=np.float32)
        self.pkt_count   = 0

        # Static baseline parameters (same as webcam_yolo)
        self.static_base: Optional[np.ndarray] = None
        self.capturing_base: bool = False
        self.capture_buffer: list = []
        self.capture_end_time: float = 0.0

        # ---- Car / signal-drop detection state -----------------------------
        self._drop_threshold   = drop_threshold    # signed % below baseline
        self._coherence_min    = coherence_min      # fraction of subcarriers
        self._drop_consecutive = drop_consecutive
        self._drop_hold_sec    = drop_hold_sec
        self._drop_hit_count   = 0
        self._drop_last_hit_t  = 0.0
        self.last_mean_rel: float  = 0.0   # signed mean relative deviation %
        self.last_coherence: float = 0.0   # fraction of subcarriers that dropped
        self.drop_detected: bool   = False # True while a confirmed car drop is active
        self._drop_printed: bool   = False # prevents repeated console spam

    def _time_str(self) -> str:
        return time.strftime("%H:%M:%S") + f".{int((time.time() % 1) * 1000):03d}"

    # ---- helpers -----------------------------------------------------------
    def _evict_old(self, now: float) -> None:
        """Drop samples older than ``window_sec`` and update running sum."""
        cutoff = now - self.window_sec
        while self._ring and self._ring[0][0] < cutoff:
            _, old_amp = self._ring.popleft()
            self._running_sum -= old_amp
            self._running_count -= 1

    def start_capture(self, duration_sec: float) -> None:
        self.capture_buffer = []
        self.capturing_base = True
        self.capture_end_time = time.monotonic() + duration_sec

    def clear_baseline(self) -> None:
        self.static_base = None

    # ---- main entry --------------------------------------------------------
    def push(self, t_mono: float, amp_list) -> None:
        """Feed one CSI amplitude measurement. Prints 'ok!' when sustained
        motion is detected."""
        amp = np.asarray(amp_list, dtype=np.float64)

        if self.capturing_base:
            if t_mono < self.capture_end_time:
                self.capture_buffer.append(amp)
                return
            else:
                self.capturing_base = False
                if self.capture_buffer:
                    self.static_base = np.mean(self.capture_buffer, axis=0)
                    print(f"[{self._time_str()}] [SYS/CALIB] [RX-{self.rx_id}] Static baseline locked from {len(self.capture_buffer)} packets.")
                self.capture_buffer = []

        # Push raw amplitudes into the per-subcarrier history
        n_sub = min(len(amp), MAX_SUBC)
        self.amp_history[:-1] = self.amp_history[1:]
        self.amp_history[-1]  = 0.0
        self.amp_history[-1, :n_sub] = amp[:n_sub]
        self.pkt_count += 1

        # Initialise running sum on first sample
        if self._running_sum is None:
            self._running_sum = np.zeros_like(amp, dtype=np.float64)

        # Grow sum if a wider amp vector arrives (shouldn't happen, but safe)
        if len(amp) > len(self._running_sum):
            new = np.zeros(len(amp), dtype=np.float64)
            new[:len(self._running_sum)] = self._running_sum
            self._running_sum = new

        # Evict old entries
        self._evict_old(t_mono)

        # Update running sum
        self._running_sum[:len(amp)] += amp
        self._running_count += 1
        self._ring.append((t_mono, amp.copy()))

        # Compute baseline average (static baseline if captured, otherwise rolling average)
        if self.static_base is not None:
            avg = self.static_base[:len(amp)]
        else:
            if self._running_count < 2:
                self.last_rms = 0.0
                return
            avg = self._running_sum[:len(amp)] / self._running_count
            
        self.last_avg = avg

        # Relative deviation per-subcarrier
        # Capped denominator prevents null subcarriers from blowing up with noise
        noise_floor = max(1e-6, float(np.max(avg)) * 0.05)
        safe_avg = np.maximum(avg, noise_floor)
        
        # Signed relative difference — uniform shifts accumulate while random noise
        # cancels out in the signed mean, giving coherent drops more weight.
        rel_diff = (amp - avg) / safe_avg
        
        # Multiply by 100 to convert to a percentage so the trackbar (0-20) 
        # still feels intuitive and the default 3.0 threshold corresponds to 3%.
        rms = float(np.abs(np.mean(rel_diff))) * 100.0
        self.last_rms = rms

        # Push into the timeseries ring
        self.rms_history[:-1] = self.rms_history[1:]
        self.rms_history[-1]  = rms

        # Threshold check (general motion)
        if rms > self.rms_threshold:
            self._hit_count += 1
            if self._hit_count >= self.consecutive_hits and not self._fired:
                self._fired = True
        else:
            self._hit_count = 0
            self._fired = False

        # ---- Car / signal-drop detection ----------------------------------
        # Signed mean relative deviation (negative = amplitude dropped)
        mean_rel = float(np.mean(rel_diff)) * 100.0
        # Fraction of subcarriers whose individual amplitude dropped > 5 %
        coherence = float(np.mean(rel_diff < -0.05))
        self.last_mean_rel  = mean_rel
        self.last_coherence = coherence

        is_drop_hit = (mean_rel < self._drop_threshold) and (coherence >= self._coherence_min)

        if is_drop_hit:
            self._drop_hit_count += 1
            self._drop_last_hit_t = t_mono
            if self._drop_hit_count >= self._drop_consecutive and not self.drop_detected:
                self.drop_detected = True
                if not self._drop_printed:
                    self._drop_printed = True
                    print(f"[{self._time_str()}] [CAR-DROP]  [RX-{self.rx_id}] "
                          f"DROP detected  mean={mean_rel:+.1f}%  coh={coherence:.2f}")
        else:
            self._drop_hit_count = 0
            # Hold the flag for _drop_hold_sec after the last hit
            if self.drop_detected and (t_mono - self._drop_last_hit_t) > self._drop_hold_sec:
                self.drop_detected = False
                self._drop_printed = False
                print(f"[{self._time_str()}] [CAR-RECV] [RX-{self.rx_id}] "
                      f"Signal recovered")


# ===========================================================================
# CSI UDP listener (background thread)
# ===========================================================================
class CSIMotionListener:
    """Receives CSI JSON on UDP, feeds amplitude arrays into per-Rx
    ``RxMotionAnalyser`` instances."""

    def __init__(self, rx_ids: list[int], window_sec: float,
                 rms_threshold: float, consecutive_hits: int,
                 drop_threshold: float = DROP_THRESHOLD,
                 coherence_min: float = COHERENCE_MIN,
                 drop_consecutive: int = DROP_CONSECUTIVE,
                 drop_hold_sec: float = DROP_HOLD_SEC):
        self.analysers: Dict[int, RxMotionAnalyser] = {
            rx: RxMotionAnalyser(rx, window_sec, rms_threshold, consecutive_hits,
                                 drop_threshold=drop_threshold,
                                 coherence_min=coherence_min,
                                 drop_consecutive=drop_consecutive,
                                 drop_hold_sec=drop_hold_sec)
            for rx in rx_ids
        }
        self.stop_flag = threading.Event()
        self.thread    = threading.Thread(target=self._run, daemon=True)
        self.lock      = threading.Lock()
        # For status display
        self.rx_arrivals: Dict[int, deque] = {}
        self.last_rssi:   Dict[int, int]   = {}

    def start(self):
        self.thread.start()

    def shutdown(self):
        self.stop_flag.set()
        self.thread.join(timeout=1.0)

    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)  # 256 KB
        try:
            sock.bind((UDP_BIND_HOST, UDP_PORT))
        except OSError as e:
            print(f"[CSI] could not bind UDP {UDP_BIND_HOST}:{UDP_PORT}: {e}")
            print( "      is another listener already running?")
            return
        sock.settimeout(0.2)
        print(f"[CSI] listening on UDP {UDP_BIND_HOST}:{UDP_PORT}")

        while not self.stop_flag.is_set():
            try:
                data, _ = sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            t_mono = time.monotonic()
            try:
                obj = json.loads(data.decode("utf-8", errors="ignore"))
                rx  = int(obj["rx"])
            except (ValueError, KeyError, TypeError):
                continue

            # Rate counter
            with self.lock:
                buf = self.rx_arrivals.setdefault(rx, deque())
                buf.append(t_mono)
                cutoff = t_mono - 1.0
                while buf and buf[0] < cutoff:
                    buf.popleft()
                if "rssi" in obj:
                    try:
                        self.last_rssi[rx] = int(obj["rssi"])
                    except (TypeError, ValueError):
                        pass

            # Feed analyser
            amp = obj.get("amp")
            if amp and rx in self.analysers:
                self.analysers[rx].push(t_mono, amp)

        sock.close()

    def rx_rates(self) -> Dict[int, int]:
        """Return a dict of {rx_id: packets_per_second}."""
        with self.lock:
            return {rx: len(dq) for rx, dq in self.rx_arrivals.items()}

    def status_string(self) -> str:
        with self.lock:
            if not self.rx_arrivals:
                return "CSI -none-"
            parts = []
            for rx in sorted(self.rx_arrivals):
                hz = len(self.rx_arrivals[rx])
                parts.append(f"rx{rx}={hz}/s")
            return "CSI " + " ".join(parts)


# ===========================================================================
# Camera
# ===========================================================================
def open_camera(cam_index: int, width: int, height: int):
    backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
    cap = cv2.VideoCapture(cam_index, backend)
    # Set MJPG compression to allow high framerates at high resolutions
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    if width:  cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    if height: cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # Force fallback to maximum hardware framerate
    cap.set(cv2.CAP_PROP_FPS, 1000)
    if not cap.isOpened():
        raise RuntimeError(f"could not open camera {cam_index}")
    aw  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah  = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    print(f"[cam] index {cam_index}: {aw}x{ah} @ {fps:.1f} fps")
    return cap, aw, ah, fps


# ===========================================================================
# Visual design system
# ===========================================================================
# Modern accent palette (BGR)
_ACCENT = {
    1: (235, 180, 52),    # cyan  #34B4EB
    2: (120, 80, 230),    # magenta #E650B8 -> warm pink
    3: (60, 210, 255),    # amber #FFD23C
}

def _rx_color(rx_id: int) -> tuple:
    if rx_id in _ACCENT:
        return _ACCENT[rx_id]
    hue = int((rx_id * 47 + 120) % 180)
    bgr = cv2.cvtColor(np.uint8([[[hue, 180, 230]]]), cv2.COLOR_HSV2BGR)[0][0]
    return tuple(int(c) for c in bgr)


def _glass_panel(frame, x, y, w, h, alpha=0.55):
    """Draw a frosted-glass rectangle onto frame (in-place)."""
    x1, y1 = max(0, x), max(0, y)
    x2, y2 = min(frame.shape[1], x + w), min(frame.shape[0], y + h)
    if x2 <= x1 or y2 <= y1:
        return
    roi = frame[y1:y2, x1:x2]
    dark = (roi * 0.25).astype(np.uint8)
    # Tint with a subtle blue
    tint = np.full_like(dark, (40, 28, 18), dtype=np.uint8)
    blended = cv2.addWeighted(dark, 0.7, tint, 0.3, 0)
    frame[y1:y2, x1:x2] = cv2.addWeighted(roi, 1.0 - alpha, blended, alpha, 0)
    # Subtle border
    cv2.rectangle(frame, (x1, y1), (x2 - 1, y2 - 1), (80, 75, 65), 1)


def _glow_line(frame, p1, p2, color, intensity=1.0):
    """Draw a line with an outer glow effect. Uses ROI blending for speed."""
    glow_a = min(1.0, intensity * 0.4)
    margin = max(14, int(14 * intensity))
    h, w = frame.shape[:2]
    # Compute tight bounding box around the line + margin
    x1 = max(0, min(p1[0], p2[0]) - margin)
    y1 = max(0, min(p1[1], p2[1]) - margin)
    x2 = min(w, max(p1[0], p2[0]) + margin)
    y2 = min(h, max(p1[1], p2[1]) + margin)
    if x2 <= x1 or y2 <= y1:
        return
    # Shifted coords within the ROI
    sp1 = (p1[0] - x1, p1[1] - y1)
    sp2 = (p2[0] - x1, p2[1] - y1)
    roi = frame[y1:y2, x1:x2]
    # Outer glow
    ov = roi.copy()
    cv2.line(ov, sp1, sp2, color, max(8, int(12 * intensity)), cv2.LINE_AA)
    cv2.addWeighted(ov, glow_a * 0.3, roi, 1.0 - glow_a * 0.3, 0, roi)
    # Mid glow
    ov2 = roi.copy()
    cv2.line(ov2, sp1, sp2, color, max(3, int(6 * intensity)), cv2.LINE_AA)
    cv2.addWeighted(ov2, glow_a * 0.5, roi, 1.0 - glow_a * 0.5, 0, roi)
    # Core line
    cv2.line(roi, sp1, sp2,
             tuple(min(255, int(c * 1.3)) for c in color),
             max(1, int(2 * intensity)), cv2.LINE_AA)
    frame[y1:y2, x1:x2] = roi


def _glow_circle(frame, center, radius, color, intensity=1.0):
    """Draw a circle with outer glow rings. ROI-based for speed."""
    margin = radius + int(8 * intensity)
    h, w = frame.shape[:2]
    x1 = max(0, center[0] - margin)
    y1 = max(0, center[1] - margin)
    x2 = min(w, center[0] + margin)
    y2 = min(h, center[1] + margin)
    if x2 <= x1 or y2 <= y1:
        return
    sc = (center[0] - x1, center[1] - y1)
    roi = frame[y1:y2, x1:x2]
    for r_off, a in [(6, 0.10), (3, 0.20)]:
        ov = roi.copy()
        cv2.circle(ov, sc, radius + int(r_off * intensity),
                   color, -1, cv2.LINE_AA)
        cv2.addWeighted(ov, a, roi, 1.0 - a, 0, roi)
    cv2.circle(roi, sc, radius, color, -1, cv2.LINE_AA)
    cv2.circle(roi, sc, radius,
               tuple(min(255, c + 60) for c in color), 1, cv2.LINE_AA)
    frame[y1:y2, x1:x2] = roi


def draw_overlay(frame, rx_positions_px: dict,
                 analysers: Dict[int, RxMotionAnalyser],
                 info_top: str, info_bot: str):
    """Modern overlay with glassmorphic panels, glowing lines, pulsing nodes."""
    h, w = frame.shape[:2]
    tx_pos = (w // 2, h - 1)

    # ---- link lines TX → each Rx (drawn first = underneath) ---------------
    teal = (235, 180, 52)   # consistent teal for all laser lines (BGR)
    if rx_positions_px:
        for rx, (rx_x, rx_y) in rx_positions_px.items():
            a = analysers.get(rx)
            rms = a.last_rms if a else 0.0
            intensity = max(0.25, min(2.0, 0.25 + rms * 0.6))
            _glow_line(frame, tx_pos, (rx_x, rx_y), teal, intensity)

        # TX node
        _glow_circle(frame, tx_pos, 5, (220, 220, 220), 0.5)

        # RX nodes
        for rx, (rx_x, rx_y) in rx_positions_px.items():
            a = analysers.get(rx)
            rms = a.last_rms if a else 0.0
            intensity = max(0.5, min(2.0, 0.5 + rms * 0.8))
            _glow_circle(frame, (rx_x, rx_y), 8, teal, intensity)
            lbl = f"RX{rx}"
            if rms > 0.01:
                lbl += f"  {rms:.2f}"
            cv2.putText(frame, lbl, (rx_x + 14, rx_y + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        tuple(min(255, c + 40) for c in teal), 1,
                        cv2.LINE_AA)

    # ---- glassmorphic top status panel ------------------------------------
    _glass_panel(frame, 6, 6, min(w - 12, 440), 52)
    cv2.putText(frame, info_top, (14, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (230, 230, 240), 1, cv2.LINE_AA)
    cv2.putText(frame, info_bot, (14, 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (160, 165, 175), 1, cv2.LINE_AA)

    # ---- bottom key hint --------------------------------------------------
    cv2.putText(frame, "Q quit", (10, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 110), 1, cv2.LINE_AA)


def render_timeseries(width: int, rx_ids: list,
                      analysers: Dict[int, RxMotionAnalyser],
                      rx_rates: Dict[int, int] = None,
                      height_per_rx: int = CHART_HEIGHT_PER_RX) -> np.ndarray:
    """Render a spectrogram-style heatmap per Rx: X = time (newest right),
    Y = subcarrier index (0 top, 63 bottom), colour = |CSI| amplitude.

    This is much faster than drawing 64 polylines and looks dramatically
    better — a waterfall / spectrogram view that immediately reveals
    perturbation patterns across the full subcarrier set."""
    n_rx    = len(rx_ids)
    total_h = height_per_rx * n_rx
    canvas  = np.zeros((total_h, width, 3), dtype=np.uint8)
    canvas[:] = (14, 14, 18)

    pad_l, pad_r = 44, 40   # left for Y labels, right for colorbar
    plot_w = width - pad_l - pad_r
    if plot_w < 20:
        return canvas

    for idx, rx in enumerate(rx_ids):
        a = analysers.get(rx)
        if a is None:
            continue

        region_top = idx * height_per_rx
        y_top  = region_top + 20      # title space
        y_bot  = (idx + 1) * height_per_rx - 6
        plot_h = y_bot - y_top
        if plot_h < 10:
            continue

        # Transpose: amp_history is (time, subc) → we want (subc, time)
        # so subc is the Y-axis (rows) and time is the X-axis (columns).
        data = a.amp_history.T            # (MAX_SUBC, TIMESERIES_LEN)

        # Normalise to 0..255 for the colormap
        d_max = float(data.max()) if data.size else 1.0
        d_max = max(d_max, 1.0)
        norm  = (data / d_max * 255.0).astype(np.uint8)

        # Apply a beautiful colormap (MAGMA / INFERNO look great as
        # spectrograms — hot=high amplitude, dark=quiet)
        heatmap = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)

        # Resize to fill the plot region exactly
        heatmap_resized = cv2.resize(heatmap, (plot_w, plot_h),
                                     interpolation=cv2.INTER_LINEAR)

        # Place into canvas
        canvas[y_top:y_bot, pad_l:pad_l + plot_w] = heatmap_resized

        # Subtle border
        cv2.rectangle(canvas, (pad_l - 1, y_top - 1),
                      (pad_l + plot_w, y_bot), (60, 55, 50), 1)

        # Y-axis labels (subcarrier indices)
        for sc_label in (0, 16, 32, 48, 63):
            frac = sc_label / max(1, MAX_SUBC - 1)
            py = y_top + int(frac * plot_h)
            cv2.putText(canvas, str(sc_label), (4, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28,
                        (130, 130, 140), 1, cv2.LINE_AA)
            # Tiny tick mark
            cv2.line(canvas, (pad_l - 4, py), (pad_l - 1, py),
                     (80, 80, 90), 1)

        # Y-axis title
        cv2.putText(canvas, "subc", (2, y_top + plot_h // 2 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, (100, 100, 115), 1,
                    cv2.LINE_AA)

        # Colorbar on the right side
        cbar_x = pad_l + plot_w + 6
        cbar_w = 14
        cbar_strip = np.linspace(255, 0, plot_h).astype(np.uint8).reshape(-1, 1)
        cbar_strip = np.repeat(cbar_strip, cbar_w, axis=1)
        cbar_color = cv2.applyColorMap(cbar_strip, cv2.COLORMAP_INFERNO)
        canvas[y_top:y_bot, cbar_x:cbar_x + cbar_w] = cbar_color
        cv2.rectangle(canvas, (cbar_x - 1, y_top - 1),
                      (cbar_x + cbar_w, y_bot), (60, 55, 50), 1)
        # Colorbar labels
        cv2.putText(canvas, f"{d_max:.0f}",
                    (cbar_x + cbar_w + 3, y_top + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.25, (140, 140, 150), 1,
                    cv2.LINE_AA)
        cv2.putText(canvas, "0",
                    (cbar_x + cbar_w + 3, y_bot - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.25, (140, 140, 150), 1,
                    cv2.LINE_AA)

        # Title for this Rx subplot
        if getattr(a, "drop_detected", False):
            state_lbl = "CAR-DROP"
            color = (0, 180, 255)
        elif a.pkt_count > 0:
            state_lbl = "LIVE"
            color = (0, 255, 0) if getattr(a, "_fired", False) else (0, 0, 255)
        else:
            state_lbl = "----"
            color = (0, 0, 255)
        # Colored dot + text
        cv2.circle(canvas, (pad_l + 4, region_top + 11), 4, color, -1,
                   cv2.LINE_AA)
        hz = (rx_rates or {}).get(rx, 0)
        cv2.putText(canvas,
                    f"RX{rx}  |CSI| spectrogram   {hz} Hz   n={a.pkt_count}  {state_lbl}",
                    (pad_l + 14, region_top + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (210, 215, 225), 1,
                    cv2.LINE_AA)

    return canvas


# ===========================================================================
# Entry point
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    # Camera
    ap.add_argument("--cam",    type=int, default=0, help="OpenCV camera index")
    ap.add_argument("--width",  type=int, default=10000)
    ap.add_argument("--height", type=int, default=10000)
    # Rx layout
    ap.add_argument("--rx-ids", default="1,2",
                    help="comma-separated Rx ids (default 1,2)")
    ap.add_argument("--rx-positions-px", default="300,40;400,300",
                    help="pixel position of each Rx in the camera frame, "
                         "semicolon-separated. Format: 'x,y;x,y;...'.")
    # Motion algorithm tuning
    ap.add_argument("--window-sec", type=float, default=WINDOW_SEC,
                    help=f"moving average window in seconds (default {WINDOW_SEC})")
    ap.add_argument("--rms-threshold", type=float, default=RMS_THRESHOLD,
                    help=f"RMS threshold for a 'hit' (default {RMS_THRESHOLD})")
    ap.add_argument("--consecutive", type=int, default=CONSECUTIVE_HITS,
                    help=f"consecutive hits before printing 'ok!' (default {CONSECUTIVE_HITS})")
    ap.add_argument("--baseline-capture-sec", type=float, default=3.0,
                    help="how many seconds to average over when you press B (default 3.0 s)")
    ap.add_argument("--display-width", type=int, default=900,
                    help="width of the output display in pixels, independent "
                         "of the camera resolution (default 900)")
    ap.add_argument("--video-delay", type=float, default=0.0,
                    help="delay the video display in seconds to match CSI signal latency")
    # Car / signal-drop detection
    ap.add_argument("--drop-threshold", type=float, default=DROP_THRESHOLD,
                    help=f"mean relative deviation (%%) below which = car drop "
                         f"(default {DROP_THRESHOLD})")
    ap.add_argument("--coherence-min", type=float, default=COHERENCE_MIN,
                    help=f"fraction of subcarriers that must drop together "
                         f"(default {COHERENCE_MIN})")
    ap.add_argument("--drop-consecutive", type=int, default=DROP_CONSECUTIVE,
                    help=f"consecutive drop-hits to confirm (default {DROP_CONSECUTIVE})")
    ap.add_argument("--drop-hold", type=float, default=DROP_HOLD_SEC,
                    help=f"seconds the drop flag stays active after last hit "
                         f"(default {DROP_HOLD_SEC})")
    args = ap.parse_args()

    rx_ids = [int(s) for s in args.rx_ids.split(",") if s.strip()]

    # Open camera first so we know the native resolution
    cap, cam_w, cam_h, cam_fps = open_camera(args.cam, args.width, args.height)

    # Display resolution (independent of camera)
    display_w = args.display_width
    display_h = int(cam_h * display_w / max(cam_w, 1))

    # Parse Rx pixel positions (in DISPLAY coordinates, not camera coords)
    rx_positions_px: Dict[int, Tuple[int, int]] = {}
    if args.rx_positions_px:
        segs = [s for s in args.rx_positions_px.split(";") if s.strip()]
        for rx, seg in zip(rx_ids, segs):
            vals = [int(v) for v in seg.split(",")]
            if len(vals) != 2:
                raise SystemExit(f"rx position for rx{rx} must be 'x,y'")
            rx_positions_px[rx] = tuple(vals)
    else:
        # Auto-place Rx nodes across the top of the display frame
        for i, rx in enumerate(rx_ids):
            x = int((i + 1) * display_w / (len(rx_ids) + 1))
            y = 40
            rx_positions_px[rx] = (x, y)

    # Start CSI listener + per-Rx motion analysers
    csi = CSIMotionListener(
        rx_ids, args.window_sec, args.rms_threshold, args.consecutive,
        drop_threshold=args.drop_threshold,
        coherence_min=args.coherence_min,
        drop_consecutive=args.drop_consecutive,
        drop_hold_sec=args.drop_hold,
    )
    csi.start()

    chart_h = CHART_HEIGHT_PER_RX * len(rx_ids)

    win = "CSI Motion Detector"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, display_w, display_h + chart_h)


    fps_dts   = deque(maxlen=30)
    t_prev    = time.monotonic()
    frame_cnt = 0

    frame_buffer = deque()

    # ---- video recorder (lazy init on first combined frame) ----------------
    rec_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
    os.makedirs(rec_dir, exist_ok=True)
    rec_ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    rec_path = os.path.join(rec_dir, f"csi_{rec_ts}.mp4")
    writer: Optional[cv2.VideoWriter] = None

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[cam] read failed; stopping")
                break
            t_mono = time.monotonic()
            frame_cnt += 1

            # Buffer frame for delay alignment
            frame_buffer.append((t_mono, frame))
            max_delay = max(0.0, args.video_delay)
            while len(frame_buffer) > 1 and (t_mono - frame_buffer[1][0]) > max_delay:
                frame_buffer.popleft()
            
            # Find the delayed frame
            target_t = t_mono - max_delay
            display_frame = frame_buffer[0][1]
            for t_f, frm in frame_buffer:
                if t_f >= target_t:
                    display_frame = frm
                    break

            # FPS
            dt = t_mono - t_prev
            t_prev = t_mono
            if dt > 1e-4:
                fps_dts.append(dt)
            fps = (len(fps_dts) / sum(fps_dts)) if fps_dts else 0.0

            # Build status lines
            rms_parts = []
            for rx in rx_ids:
                a = csi.analysers[rx]
                rms_parts.append(f"rx{rx}={a.last_rms:.2f}")
            info_top = f"fps={fps:.1f}  f={frame_cnt}  " + "  ".join(rms_parts)
            info_bot = csi.status_string()

            # Resize delayed camera frame to display resolution before overlay
            display_frame_resized = cv2.resize(display_frame, (display_w, display_h),
                                               interpolation=cv2.INTER_LINEAR)

            draw_overlay(display_frame_resized, rx_positions_px, csi.analysers,
                         info_top, info_bot)

            # Render chart at display width (independent of camera)
            chart_img = render_timeseries(display_w, rx_ids, csi.analysers,
                                           rx_rates=csi.rx_rates())
            combined = np.vstack([display_frame_resized, chart_img])

            # Write frame to recording
            if writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                writer = cv2.VideoWriter(rec_path, fourcc, 20.0,
                                         (combined.shape[1], combined.shape[0]))
                print(f"[REC] recording to {rec_path}")
            writer.write(combined)

            cv2.imshow(win, combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('b'):
                print(f"[SYS] Capturing static baselines for {args.baseline_capture_sec:.1f}s — keep still")
                for a in csi.analysers.values():
                    a.start_capture(args.baseline_capture_sec)
            elif key == ord('c'):
                print("[SYS] Static baselines cleared (reverted to rolling average)")
                for a in csi.analysers.values():
                    a.clear_baseline()
    finally:
        if writer is not None:
            writer.release()
            print(f"[REC] saved {rec_path}")
        csi.shutdown()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
