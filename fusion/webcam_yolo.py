"""Webcam labels + CSI recorder, combined.

This is Phase A1 + A2 of the roadmap: produce time-aligned
(CSI vector, object position) training pairs in ONE process so both streams
share the same clock and the same start/stop event.

Detection modes
---------------
    --mode yolo  : Ultralytics YOLO over COCO. Good for sports balls (class 32)
                   and other COCO objects. Not reliable on rubber ducks etc.
    --mode color : OpenCV HSV thresholding for a known-colour blob on a plain
                   background. Default for the rubber-duck-on-a-white-wall case.

Sync clock
----------
``time.monotonic()`` for everything — webcam frames, YOLO detections, CSI
UDP packets. The dataset builder joins on this clock.

Output (per run directory)
--------------------------
    meta.json          run metadata
    positions.jsonl    one line per webcam frame (always written if recording)
    csi.jsonl          one line per CSI UDP packet (always written if recording)

Status line
-----------
The window's top-left text always shows what each Rx is doing live — even
if you aren't recording — so you can verify the firmware is alive before
hitting R. Looks like:

    REC f=512 fps=29.8 det=480/512   CSI rx1=99/s rx2=101/s rx3=0/s

Keys (window focused)
---------------------
    R - toggle recording on/off
    Q - quit

Usage
-----
    pip install ultralytics opencv-python

    # rubber duck (yellow) on white wall — colour tracking
    python webcam_yolo.py --mode color --color yellow

    # sports ball (foil-wrapped tennis ball, etc) — YOLO
    python webcam_yolo.py --mode yolo --class 32

    # GPU
    python webcam_yolo.py --device cuda
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
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from config import UDP_BIND_HOST, UDP_PORT
from tripwire import TransitEvent, TripwireDetector


# ---- COCO class index used as default for --mode yolo --------------------
# Variable name kept for git-diff sanity; the user pivoted from ducks to cars
# so the value is now 2 (car), not 32 (sports ball).
COCO_SPORTS_BALL = 2

# Vehicle COCO classes used as default for --mode cars.
COCO_VEHICLE_CLASSES = (2, 3, 5, 7)     # car, motorcycle, bus, truck

# ---- Colour presets in HSV (OpenCV ranges: H 0..179, S/V 0..255) ----------
# Tweak with --hsv-lo / --hsv-hi if these don't lock onto your specific duck.
COLOR_PRESETS = {
    "yellow": ((15,  80,  80), (35, 255, 255)),
    # Tennis balls are yellow-green, slightly greener than the duck-yellow
    # preset above. Lower S/V floor too, because they tend to be matte and
    # sometimes dim in indoor light. If false positives appear (e.g. on a
    # lit wall), raise S to ~120.
    "tennis": ((25,  60,  60), (55, 255, 255)),
    "orange": (( 5, 100, 100), (18, 255, 255)),
    "green":  ((35,  60,  60), (85, 255, 255)),
    "blue":   ((95,  80,  60), (130, 255, 255)),
    "red":    (( 0, 100, 100), (10, 255, 255)),   # also matches the 170-180 wraparound, see hsv_inrange_red
    "magenta":((140, 80, 80),  (170, 255, 255)),
}


# ===========================================================================
# CSI UDP listener (background thread)
# ===========================================================================
class CSIListener:
    """Receives CSI JSON on UDP 5005, keeps a per-Rx live-rate counter for
    the status bar, feeds amp arrays into the (optional) TripwireDetector,
    and (when ``recorder`` is active) appends every packet to ``csi.jsonl``
    and every TransitEvent to ``transits.jsonl``."""

    def __init__(self, recorder_ref, tripwire: Optional[TripwireDetector] = None):
        self.recorder_ref = recorder_ref     # callable -> Optional[Recorder]
        self.tripwire     = tripwire
        self.stop_flag    = threading.Event()
        self.thread       = threading.Thread(target=self._run, daemon=True)
        self.lock         = threading.Lock()
        # rolling 1-sec rate per rx
        self.rx_arrivals: dict[int, deque] = {}
        self.last_rssi: dict[int, int]     = {}
        # ring of recent transits, for the on-screen overlay
        self.recent_transits: deque = deque(maxlen=8)

    def start(self):
        self.thread.start()

    def shutdown(self):
        self.stop_flag.set()
        self.thread.join(timeout=1.0)

    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((UDP_BIND_HOST, UDP_PORT))
        except OSError as e:
            print(f"[CSI] could not bind UDP {UDP_BIND_HOST}:{UDP_PORT}: {e}")
            print( "      is csi_timeseries.py or another listener already running?")
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

            rec = self.recorder_ref()
            if rec is not None and rec.active:
                rec.write_csi({"t_mono": round(t_mono, 6), **obj})

            # Feed the tripwire detector. Prefer raw amp (baseline-based
            # scoring); fall back to the firmware's pre-computed "motion"
            # score (esp-radar waveform_jitter) via push_score().
            if self.tripwire is not None:
                amp = obj.get("amp")
                motion = obj.get("motion")
                transit = None
                if amp:
                    transit = self.tripwire.push(rx, t_mono, amp)
                    # Debug: print once every ~200 packets per Rx
                    with self.lock:
                        _dbg_cnt = getattr(self, '_dbg_cnt', {})
                        _dbg_cnt[rx] = _dbg_cnt.get(rx, 0) + 1
                        self._dbg_cnt = _dbg_cnt
                        if _dbg_cnt[rx] % 200 == 1:
                            sc = self.tripwire.latest_score.get(rx, -1)
                            bs = self.tripwire.baseline_status().get(rx, '?')
                            print(f"[DBG] rx{rx} amp len={len(amp)} "
                                  f"score={sc:.4f} baseline={bs} "
                                  f"pkt#{_dbg_cnt[rx]}")
                elif motion is not None:
                    try:
                        transit = self.tripwire.push_score(rx, t_mono, float(motion))
                    except (TypeError, ValueError):
                        pass
                if transit is not None:
                    with self.lock:
                        self.recent_transits.append((t_mono, transit))
                    print(f"[TRIPWIRE] transit  dir={transit.direction}  "
                          f"speed={transit.speed_mps}  "
                          f"peaks={[round(e.t_peak, 3) for e in transit.rx_events.values()]}")
                    if rec is not None and rec.active:
                        rec.write_transit({"t_mono": round(t_mono, 6),
                                           **transit.to_jsonable()})
        sock.close()

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
# Detection backends
# ===========================================================================
class YoloBackend:
    def __init__(self, model_name: str, device: str, class_id: int, conf: float):
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise SystemExit(
                "Missing dependency. Install with:\n"
                "    pip install ultralytics opencv-python"
            ) from e
        print(f"[YOLO] loading {model_name} on device={device}")
        self.model    = YOLO(model_name)
        self.device   = device
        self.class_id = class_id
        self.conf     = conf
        self.label    = (self.model.names.get(class_id, f"class_{class_id}")
                         if class_id >= 0 else "ANY")
        print(f"[YOLO] target class: {self.label}")

    def detect(self, frame):
        """Return a list of detection dicts. YOLO mode returns at most one
        (the highest-confidence single match), so the list is length 0 or 1."""
        classes = None if self.class_id < 0 else [self.class_id]
        results = self.model.predict(frame, conf=self.conf, classes=classes,
                                     verbose=False, device=self.device)
        r = results[0]
        if r.boxes is None or len(r.boxes) == 0:
            return []
        confs = r.boxes.conf.cpu().numpy()
        best  = int(confs.argmax())
        x1, y1, x2, y2 = r.boxes.xyxy[best].cpu().numpy().tolist()
        cls   = int(r.boxes.cls[best].cpu().numpy())
        return [{
            "bbox_xyxy": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
            "center_px": [round((x1 + x2) / 2, 1), round((y1 + y2) / 2, 1)],
            "conf":      round(float(confs[best]), 3),
            "class_id":  cls,
            "label":     self.model.names.get(cls, str(cls)),
        }]


class ColorBackend:
    """HSV-based blob detector. Picks the largest contour matching the
    colour range. Fast and reliable for known-colour objects on plain
    backgrounds — the rubber-duck-on-a-white-wall case."""

    def __init__(self, hsv_lo, hsv_hi, min_area: int, label: str):
        self.lo       = np.array(hsv_lo, dtype=np.uint8)
        self.hi       = np.array(hsv_hi, dtype=np.uint8)
        self.min_area = min_area
        self.label    = label
        self._kernel  = np.ones((5, 5), np.uint8)
        print(f"[COLOR] target='{label}' HSV {tuple(hsv_lo)}..{tuple(hsv_hi)} "
              f"min_area={min_area}")

    def detect(self, frame):
        """Return a list of detection dicts (length 0 or 1)."""
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lo, self.hi)
        # Cleanup speckles + close small gaps so one duck becomes one blob.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []
        c    = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < self.min_area:
            return []
        x, y, w, h = cv2.boundingRect(c)
        x1, y1, x2, y2 = float(x), float(y), float(x + w), float(y + h)
        # "confidence" here = area fraction of the frame, clipped to 1.0.
        conf = min(1.0, area / (frame.shape[0] * frame.shape[1] * 0.25))
        return [{
            "bbox_xyxy": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
            "center_px": [round((x1 + x2) / 2, 1), round((y1 + y2) / 2, 1)],
            "conf":      round(conf, 3),
            "class_id":  -1,
            "label":     self.label,
        }]


class MovingCarBackend:
    """YOLO with Ultralytics' built-in BoT-SORT tracker + a velocity filter
    that rejects parked / stationary cars.

    Output is a list of tracked vehicles, each with a persistent integer
    ``id`` (stable across frames) and a ``speed_pxs`` in pixels per second
    computed over the most-recent ~0.5 s of that track's history. Tracks
    whose recent speed is below ``min_speed_pxs`` are dropped — that's what
    filters out static parked cars in the scene.

    Speed is in pixel space because we have no camera-to-world homography
    yet. Calibrating that comes later; for now we just need to distinguish
    "moving" from "parked". A car at ~30 km/h with a typical roadside
    framing maps to several hundred pixels/sec; static jitter from YOLO
    bbox flicker is typically <30 px/s.
    """

    def __init__(self, model_name, device, classes, conf,
                 min_speed_pxs, speed_window_sec, track_timeout_sec,
                 tracker_cfg):
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise SystemExit(
                "Missing dependency. Install with:\n"
                "    pip install ultralytics opencv-python"
            ) from e
        print(f"[CARS] loading {model_name} on device={device}")
        self.model              = YOLO(model_name)
        self.device             = device
        self.classes            = list(classes)
        self.conf               = conf
        self.min_speed_pxs      = float(min_speed_pxs)
        self.speed_window_sec   = float(speed_window_sec)
        self.track_timeout_sec  = float(track_timeout_sec)
        self.tracker_cfg        = tracker_cfg
        # Per-track (cx, cy) history with timestamps for velocity calc.
        self.history: dict[int, deque] = {}
        self._last_prune        = 0.0
        labels = ", ".join(self.model.names.get(c, str(c)) for c in self.classes)
        print(f"[CARS] vehicle classes: {labels}")
        print(f"[CARS] min_speed_pxs={min_speed_pxs}  "
              f"speed_window={speed_window_sec:.2f}s  "
              f"tracker={tracker_cfg}")

    @staticmethod
    def _speed_over_window(history: deque, window_sec: float) -> float:
        """Most-recent-window pixel speed. Falls back to full history if
        the window doesn't contain at least 2 points."""
        if len(history) < 2:
            return 0.0
        t_latest = history[-1][0]
        cutoff   = t_latest - window_sec
        pts = [p for p in history if p[0] >= cutoff]
        if len(pts) < 2:
            pts = list(history)
        t0, x0, y0 = pts[0]
        t1, x1, y1 = pts[-1]
        dt = t1 - t0
        if dt < 1e-3:
            return 0.0
        dx = x1 - x0
        dy = y1 - y0
        return (dx * dx + dy * dy) ** 0.5 / dt

    def _prune(self, t_now: float) -> None:
        if t_now - self._last_prune < 1.0:
            return
        stale = [tid for tid, h in self.history.items()
                 if t_now - h[-1][0] > self.track_timeout_sec]
        for tid in stale:
            del self.history[tid]
        self._last_prune = t_now

    def detect(self, frame):
        t_now   = time.monotonic()
        # persist=True keeps the tracker state between calls; required for IDs
        # to remain stable across frames.
        results = self.model.track(
            frame,
            classes=self.classes,
            conf=self.conf,
            persist=True,
            verbose=False,
            device=self.device,
            tracker=self.tracker_cfg,
        )
        r = results[0]
        detections = []

        if r.boxes is not None and r.boxes.id is not None and len(r.boxes) > 0:
            ids   = r.boxes.id.cpu().numpy().astype(int)
            confs = r.boxes.conf.cpu().numpy()
            clses = r.boxes.cls.cpu().numpy().astype(int)
            xyxys = r.boxes.xyxy.cpu().numpy()

            for i, tid in enumerate(ids):
                x1, y1, x2, y2 = xyxys[i].tolist()
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

                # Track history (bounded length so it doesn't grow forever)
                hist = self.history.setdefault(int(tid), deque(maxlen=120))
                hist.append((t_now, cx, cy))

                speed = self._speed_over_window(hist, self.speed_window_sec)
                if speed < self.min_speed_pxs:
                    # Parked / stationary — keep tracking it (so the ID stays
                    # the same when it does start moving) but don't emit it.
                    continue

                cls = int(clses[i])
                detections.append({
                    "bbox_xyxy": [round(x1, 1), round(y1, 1),
                                  round(x2, 1), round(y2, 1)],
                    "center_px": [round(cx, 1), round(cy, 1)],
                    "conf":      round(float(confs[i]), 3),
                    "class_id":  cls,
                    "label":     self.model.names.get(cls, str(cls)),
                    "id":        int(tid),
                    "speed_pxs": round(float(speed), 1),
                })

        self._prune(t_now)
        return detections


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
        raise RuntimeError(f"could not open camera {cam_index} — try a different --cam")
    aw   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps  = cap.get(cv2.CAP_PROP_FPS) or 30.0
    print(f"[cam] index {cam_index}: {aw}x{ah} @ {fps:.1f} fps requested")
    return cap, aw, ah, fps


# ===========================================================================
# Recorder: writes positions.jsonl + csi.jsonl + meta.json to one run dir.
# ===========================================================================
class Recorder:
    def __init__(self, cam_w, cam_h, cam_fps, meta_extra: dict):
        self.cam_w      = cam_w
        self.cam_h      = cam_h
        self.cam_fps    = cam_fps
        self.meta_extra = meta_extra
        self.path: Optional[Path] = None
        self.pos_fh     = None
        self.csi_fh     = None
        self.transit_fh = None
        # Locked because the UDP thread writes csi.jsonl + transits.jsonl
        # concurrently with the main thread writing positions.jsonl +
        # opening/closing files.
        self.lock   = threading.Lock()
        self.n_pos      = 0
        self.n_pos_det  = 0
        self.n_csi      = 0
        self.n_transit  = 0

    @property
    def active(self) -> bool:
        return self.pos_fh is not None

    def start(self, path: Optional[str]) -> None:
        with self.lock:
            if self.active:
                return
            if path is None:
                path = f"data/run-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
            p = Path(path)
            p.mkdir(parents=True, exist_ok=True)
            meta = {
                "started_iso":   datetime.now().isoformat(timespec="seconds"),
                "started_mono":  time.monotonic(),
                "camera_w":      self.cam_w,
                "camera_h":      self.cam_h,
                "camera_fps":    self.cam_fps,
                "clock":         "time.monotonic() seconds",
                "coord_system":  "pixels, origin top-left, +x right, +y down",
                **self.meta_extra,
            }
            (p / "meta.json").write_text(json.dumps(meta, indent=2))
            self.pos_fh     = open(p / "positions.jsonl", "w", encoding="utf-8")
            self.csi_fh     = open(p / "csi.jsonl",       "w", encoding="utf-8")
            self.transit_fh = open(p / "transits.jsonl",  "w", encoding="utf-8")
            self.path       = p
            self.n_pos     = 0
            self.n_pos_det = 0
            self.n_csi     = 0
            self.n_transit = 0
            print(f"[REC] -> {p}/")

    def write_pos(self, entry: dict) -> None:
        with self.lock:
            if self.pos_fh is None:
                return
            self.pos_fh.write(json.dumps(entry) + "\n")
            self.n_pos += 1
            if entry.get("detected"):
                self.n_pos_det += 1

    def write_csi(self, entry: dict) -> None:
        with self.lock:
            if self.csi_fh is None:
                return
            self.csi_fh.write(json.dumps(entry) + "\n")
            self.n_csi += 1

    def write_transit(self, entry: dict) -> None:
        with self.lock:
            if self.transit_fh is None:
                return
            self.transit_fh.write(json.dumps(entry) + "\n")
            self.n_transit += 1

    def stop(self) -> None:
        with self.lock:
            if not self.active:
                return
            self.pos_fh.close();     self.pos_fh     = None
            self.csi_fh.close();     self.csi_fh     = None
            self.transit_fh.close(); self.transit_fh = None
            print(f"[REC] stopped:  positions {self.n_pos_det}/{self.n_pos} detected,  "
                  f"csi {self.n_csi} packets,  transits {self.n_transit}  "
                  f"->  {self.path}/")
            self.path = None


# ===========================================================================
# Overlay drawing
# ===========================================================================
# BGR colours indexed by Rx id (1..).  First three are hand-picked for
# visual distinctness on a typical road-grey background; higher ids get
# auto-generated colours from the HSV wheel.
_RX_COLORS_PRESET = {
    1: (96, 96, 255),     # red-ish
    2: (96, 255, 96),     # green
    3: (255, 192, 64),    # blue/cyan
}


def _rx_color(rx_id: int) -> tuple:
    """Return a BGR colour for *rx_id*, generating dynamically for ids > 3."""
    if rx_id in _RX_COLORS_PRESET:
        return _RX_COLORS_PRESET[rx_id]
    hue = int((rx_id * 67) % 180)
    bgr = cv2.cvtColor(np.uint8([[[hue, 200, 220]]]), cv2.COLOR_HSV2BGR)[0][0]
    return tuple(int(c) for c in bgr)


def draw_overlay(frame, detections, rec_active, info_top, info_bot,
                 tripwire_lines=None, recent_transit_text=None,
                 tripwire_states=None, tuning_text=None,
                 rx_positions_px=None, csi_scores=None, mouse_pos=None):
    h, w = frame.shape[:2]
    tx_pos = (w // 2, h)   # camera / TX = bottom-centre of frame

    # ---- bistatic link lines (TX -> each Rx) ----------------------------
    # Drawn first (underneath everything) so detections render ON TOP.
    if rx_positions_px:
        cv2.circle(frame, tx_pos, 6, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, "TX/cam", (tx_pos[0] + 8, tx_pos[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        for rx, (rx_x, rx_y) in rx_positions_px.items():
            color = _rx_color(rx)
            score = (csi_scores or {}).get(rx, 0.0)
            state_str = (tripwire_states or {}).get(rx, "")
            # Thickness: 1 at score=0, up to 4 near T_high, 6 when FIRING
            if state_str == "FIRE":
                thickness = 6
                draw_color = tuple(min(255, int(c * 1.4)) for c in color)
            else:
                thickness = max(1, min(4, int(1 + score * 15)))
                dim = max(0.3, min(1.0, 0.3 + score * 3.5))
                draw_color = tuple(int(c * dim) for c in color)
            cv2.line(frame, tx_pos, (rx_x, rx_y),
                     draw_color, thickness, lineType=cv2.LINE_AA)
            # Rx node marker
            cv2.circle(frame, (rx_x, rx_y), 7, color, -1, cv2.LINE_AA)
            cv2.circle(frame, (rx_x, rx_y), 7, (255, 255, 255), 1, cv2.LINE_AA)
            lbl = f"rx{rx}"
            if score > 0.01:
                lbl += f" {score:.2f}"
            cv2.putText(frame, lbl, (rx_x + 10, rx_y + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    # ---- tripwire lines ------------------------------------------------
    if tripwire_lines:
        for rx, (x1, y1, x2, y2) in tripwire_lines.items():
            color     = _rx_color(rx)
            thickness = 3 if (tripwire_states and tripwire_states.get(rx) == "FIRE") else 1
            cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                     color, thickness, lineType=cv2.LINE_AA)
            cv2.putText(frame, f"rx{rx}", (int(x1) + 4, int(y1) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    # ---- detections (cars/balls/ducks) --------------------------------
    for d in detections:
        x1, y1, x2, y2 = (int(v) for v in d["bbox_xyxy"])
        cx, cy         = (int(v) for v in d["center_px"])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
        parts = []
        if "id" in d:
            parts.append(f"#{d['id']}")
        parts.append(d.get("label", "?"))
        parts.append(f"{d.get('conf', 0):.2f}")
        if "speed_pxs" in d:
            parts.append(f"{d['speed_pxs']:.0f}px/s")
        cv2.putText(frame, " ".join(parts), (x1, max(y1 - 6, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    # crosshair at image centre
    cv2.line(frame, (w // 2 - 8, h // 2), (w // 2 + 8, h // 2), (60, 60, 60), 1)
    cv2.line(frame, (w // 2, h // 2 - 8), (w // 2, h // 2 + 8), (60, 60, 60), 1)

    # status bars (top-left)
    cv2.putText(frame, info_top, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 0, 255) if rec_active else (200, 200, 200), 2)
    cv2.putText(frame, info_bot, (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (200, 200, 200), 1)
    if tuning_text:
        cv2.putText(frame, tuning_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (180, 220, 180), 1)

    # most-recent transit (bottom-left, above the key hint)
    if recent_transit_text:
        cv2.putText(frame, recent_transit_text, (10, h - 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2)

    cv2.putText(frame, "R: rec   B: capture baselines   C: clear   Q: quit",
                (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (180, 180, 180), 1)

    # ---- mouse position overlay (drawn last = always on top) -----------
    if mouse_pos is not None:
        mx, my = mouse_pos
        cv2.line(frame, (mx - 12, my), (mx + 12, my),
                 (0, 255, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (mx, my - 12), (mx, my + 12),
                 (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"({mx}, {my})", (mx + 14, my - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)


# ===========================================================================
# Entry point
# ===========================================================================
def parse_hsv(s: str):
    parts = [int(p.strip()) for p in s.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected H,S,V")
    return tuple(parts)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--mode", choices=["yolo", "color", "cars"], default="cars",
                    help="detection backend: 'cars' (moving-car tracker, default), "
                         "'yolo' (single COCO class), 'color' (HSV blob)")
    # YOLO + cars opts (shared)
    ap.add_argument("--model", default="yolo11n.pt", help="(yolo/cars) weights file")
    ap.add_argument("--class", dest="cls", type=int, default=COCO_SPORTS_BALL,
                    help=f"(yolo mode) COCO class id, default {COCO_SPORTS_BALL}. "
                         "Pass -1 for any class.")
    ap.add_argument("--conf", type=float, default=0.25,
                    help="(yolo/cars) confidence threshold")
    ap.add_argument("--device", default="cpu", help="(yolo/cars) 'cpu' or 'cuda'")
    # Cars-only opts
    ap.add_argument("--vehicle-classes", type=str, default=None,
                    help="(cars mode) comma-separated COCO ids; "
                         f"default {','.join(str(c) for c in COCO_VEHICLE_CLASSES)} "
                         "= car,motorcycle,bus,truck")
    ap.add_argument("--min-speed-pxs", type=float, default=30.0,
                    help="(cars mode) pixel-speed threshold above which a track "
                         "is considered 'moving' (default 30 px/s)")
    ap.add_argument("--speed-window", type=float, default=0.5,
                    help="(cars mode) seconds of recent history used to compute "
                         "track speed (default 0.5 s)")
    ap.add_argument("--track-timeout", type=float, default=2.0,
                    help="(cars mode) drop a track if not updated for this many "
                         "seconds (default 2.0 s)")
    ap.add_argument("--tracker", default="botsort.yaml",
                    help="(cars mode) Ultralytics tracker cfg "
                         "('botsort.yaml' or 'bytetrack.yaml')")
    # Colour opts
    ap.add_argument("--color", choices=list(COLOR_PRESETS), default="yellow",
                    help=f"(color mode) preset (default: yellow). Presets: "
                         f"{', '.join(COLOR_PRESETS)}.")
    ap.add_argument("--hsv-lo", type=parse_hsv, default=None,
                    help="(color mode) override low HSV bound as H,S,V")
    ap.add_argument("--hsv-hi", type=parse_hsv, default=None,
                    help="(color mode) override high HSV bound as H,S,V")
    ap.add_argument("--min-area", type=int, default=400,
                    help="(color mode) minimum blob pixel area (default 400)")
    # Camera
    ap.add_argument("--cam",    type=int, default=0,
                    help="OpenCV camera index. 0=built-in laptop cam, "
                         "1+=external USB. Run with --list-cams to probe.")
    ap.add_argument("--list-cams", action="store_true",
                    help="Probe camera indices 0..4, print their resolution, exit.")
    ap.add_argument("--width",  type=int, default=10000)
    ap.add_argument("--height", type=int, default=10000)
    # Recording
    ap.add_argument("--record", default=None,
                    help="run dir to start recording into (omit = idle, R toggles)")
    # Tripwire detector
    ap.add_argument("--rx-ids", default="1,2,3",
                    help="comma-separated Rx ids the detector listens for (default 1,2,3)")
    ap.add_argument("--tripwire-thigh", type=float, default=0.20,
                    help="score threshold to FIRE a tripwire (default 0.20)")
    ap.add_argument("--tripwire-tlow", type=float, default=0.08,
                    help="score threshold to release a tripwire / hysteresis (default 0.08)")
    ap.add_argument("--ema-alpha", type=float, default=1e-4,
                    help="adaptive baseline EMA alpha; smaller = baseline tracks "
                         "drift slower (default 1e-4)")
    ap.add_argument("--refractory-sec", type=float, default=0.2)
    ap.add_argument("--coincidence-sec", type=float, default=2.0,
                    help="how long after the first Rx fires we wait for the "
                         "others to also fire before committing a transit (default 2.0 s)")
    ap.add_argument("--min-rx-fires", type=int, default=None,
                    help="min Rx that must fire in the coincidence window to count "
                         "as a real transit (default: min(2, number of --rx-ids))")
    ap.add_argument("--link-spacings", default=None,
                    help="metres between consecutive Rx in road-direction order, "
                         "e.g. '1.5,1.5' for 3 Rx spaced 1.5 m apart. Enables "
                         "speed estimation. If omitted, transits report direction "
                         "only.")
    ap.add_argument("--tripwires-px", default=None,
                    help="visualisation: line endpoints per Rx in pixel coords, "
                         "semicolon-separated. Format: 'x1,y1,x2,y2;x1,y1,x2,y2;...'. "
                         "One segment per Rx, in --rx-ids order. Drawn on the camera "
                         "frame; flashes thick when that Rx is FIRING.")
    ap.add_argument("--rx-positions-px", default=None,
                    help="pixel position of each Rx in the camera frame, "
                         "semicolon-separated. Format: 'x,y;x,y;...'. "
                         "One position per Rx, in --rx-ids order. Enables "
                         "drawing of TX-Rx bistatic link lines on the video.")
    ap.add_argument("--baseline-capture-sec", type=float, default=3.0,
                    help="how many seconds to average over when you press B "
                         "(default 3.0 s)")
    args = ap.parse_args()

    # --- helper: probe webcams and exit if requested ---
    if args.list_cams:
        backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
        for idx in range(5):
            cap_test = cv2.VideoCapture(idx, backend)
            ok = cap_test.isOpened()
            if ok:
                w = int(cap_test.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap_test.get(cv2.CAP_PROP_FRAME_HEIGHT))
                print(f"  cam {idx}: OPEN  {w}x{h}")
                cap_test.release()
            else:
                print(f"  cam {idx}: not available")
        return

    # Camera first so we know the resolution before opening detection.
    cap, cam_w, cam_h, cam_fps = open_camera(args.cam, args.width, args.height)

    # Detection backend
    if args.mode == "yolo":
        backend = YoloBackend(args.model, args.device, args.cls, args.conf)
        meta_extra = {"detection": "yolo", "model": args.model,
                      "target_class": args.cls, "target_label": backend.label}
    elif args.mode == "cars":
        if args.vehicle_classes is not None:
            vehicle_cls = tuple(int(c) for c in args.vehicle_classes.split(",") if c.strip())
        else:
            vehicle_cls = COCO_VEHICLE_CLASSES
        backend = MovingCarBackend(
            args.model, args.device, vehicle_cls, args.conf,
            args.min_speed_pxs, args.speed_window, args.track_timeout,
            args.tracker,
        )
        meta_extra = {"detection":     "cars",
                      "model":         args.model,
                      "vehicle_class_ids": list(vehicle_cls),
                      "min_speed_pxs": args.min_speed_pxs,
                      "speed_window":  args.speed_window,
                      "tracker_cfg":   args.tracker}
    else:
        if args.hsv_lo is not None and args.hsv_hi is not None:
            lo, hi = args.hsv_lo, args.hsv_hi
            preset_name = "custom"
        else:
            lo, hi = COLOR_PRESETS[args.color]
            preset_name = args.color
        backend = ColorBackend(lo, hi, args.min_area, preset_name)
        meta_extra = {"detection": "color", "color_preset": preset_name,
                      "hsv_lo": list(lo), "hsv_hi": list(hi),
                      "min_area": args.min_area}

    # --- Tripwire detector ----------------------------------------------
    rx_ids = [int(s) for s in args.rx_ids.split(",") if s.strip()]

    # Parse Rx pixel positions early — if fewer positions than rx_ids are
    # provided, auto-trim rx_ids so single-node (or 2-node) operation works
    # without needing an explicit --rx-ids override.
    rx_positions_px: dict[int, tuple] = {}
    if args.rx_positions_px:
        segs = [s for s in args.rx_positions_px.split(";") if s.strip()]
        if len(segs) > len(rx_ids):
            raise SystemExit(
                f"--rx-positions-px has {len(segs)} positions but only "
                f"{len(rx_ids)} --rx-ids configured.")
        if len(segs) < len(rx_ids):
            rx_ids = rx_ids[:len(segs)]
            print(f"[config] --rx-positions-px has {len(segs)} position(s); "
                  f"auto-trimming --rx-ids to {rx_ids}")
        for rx, seg in zip(rx_ids, segs):
            vals = [int(v) for v in seg.split(",")]
            if len(vals) != 2:
                raise SystemExit(f"rx position for rx{rx} must be 'x,y'")
            rx_positions_px[rx] = tuple(vals)

    if args.min_rx_fires is None:
        args.min_rx_fires = min(2, len(rx_ids))
    link_spacings_m = None
    if args.link_spacings:
        gaps = [float(s) for s in args.link_spacings.split(",")]
        if len(gaps) != len(rx_ids) - 1:
            raise SystemExit(
                f"--link-spacings has {len(gaps)} gaps but expected "
                f"{len(rx_ids) - 1} (one fewer than --rx-ids count).")
        link_spacings_m = {}
        for i, gap in enumerate(gaps):
            link_spacings_m[(rx_ids[i], rx_ids[i + 1])] = gap

    tripwire = TripwireDetector(
        rx_ids          = rx_ids,
        ema_alpha       = args.ema_alpha,
        threshold_high  = args.tripwire_thigh,
        threshold_low   = args.tripwire_tlow,
        refractory_sec  = args.refractory_sec,
        coincidence_sec = args.coincidence_sec,
        min_rx_fires    = args.min_rx_fires,
        link_spacings_m = link_spacings_m,
    )

    # Parse tripwire-line endpoints for the on-camera overlay.
    tripwire_lines: dict[int, tuple] = {}
    if args.tripwires_px:
        segs = [s for s in args.tripwires_px.split(";") if s.strip()]
        if len(segs) != len(rx_ids):
            raise SystemExit(
                f"--tripwires-px has {len(segs)} segments but expected "
                f"{len(rx_ids)} (one per --rx-ids).")
        for rx, seg in zip(rx_ids, segs):
            vals = [int(v) for v in seg.split(",")]
            if len(vals) != 4:
                raise SystemExit(f"tripwire segment for rx{rx} must be 'x1,y1,x2,y2'")
            tripwire_lines[rx] = tuple(vals)

    # Recorder — created idle, the CSI thread holds a callable to fetch the
    # current Recorder so its file handle changes are visible immediately
    # when we hit R.
    rec_holder = {"rec": Recorder(cam_w, cam_h, cam_fps, dict(
        meta_extra,
        tripwire_thigh   = args.tripwire_thigh,
        tripwire_tlow    = args.tripwire_tlow,
        ema_alpha        = args.ema_alpha,
        rx_ids           = rx_ids,
        link_spacings_m  = [list(k) + [v] for k, v in (link_spacings_m or {}).items()],
        tripwires_px     = {str(k): list(v) for k, v in tripwire_lines.items()},
    ))}
    def get_recorder() -> Recorder:
        return rec_holder["rec"]

    csi = CSIListener(get_recorder, tripwire=tripwire)
    csi.start()

    if args.record is not None:
        rec_holder["rec"].start(args.record)

    win = "Webcam + CSI recorder"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, cam_w, cam_h)

    # ---- mouse position tracker ----------------------------------------
    _mouse_pos = [None]   # mutable holder: (x, y) or None
    def _on_mouse(event, x, y, flags, param):
        _mouse_pos[0] = (x, y)
    cv2.setMouseCallback(win, _on_mouse)

    # ---- live-tunable trackbars ----------------------------------------
    # OpenCV trackbars only handle ints, so we use 0..N ranges and divide.
    def _on_thigh(v):  tripwire.threshold_high = v / 100.0
    def _on_tlow(v):   tripwire.threshold_low  = v / 100.0
    cv2.createTrackbar("T_high x100", win, int(args.tripwire_thigh * 100), 100, _on_thigh)
    cv2.createTrackbar("T_low  x100", win, int(args.tripwire_tlow  * 100), 100, _on_tlow)
    if args.mode == "cars":
        def _on_min_speed(v):
            if hasattr(backend, "min_speed_pxs"):
                backend.min_speed_pxs = float(v)
        cv2.createTrackbar("min speed px/s", win, int(args.min_speed_pxs), 800, _on_min_speed)
    if args.mode in ("yolo", "cars"):
        def _on_conf(v):
            if hasattr(backend, "conf"):
                backend.conf = max(0.01, v / 100.0)
        cv2.createTrackbar("conf x100", win, int(args.conf * 100), 100, _on_conf)

    frame_count  = 0
    detect_count = 0
    fps_dts      = deque(maxlen=30)
    t_prev       = time.monotonic()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[cam] read failed; stopping")
                break
            t_mono = time.monotonic()
            frame_count += 1

            detections = backend.detect(frame)
            entry = {
                "t_mono":     round(t_mono, 6),
                "frame":      frame_count,
                "detected":   bool(detections),
                "img_w":      cam_w,
                "img_h":      cam_h,
                "detections": detections,
            }
            if detections:
                detect_count += 1

            rec_holder["rec"].write_pos(entry)

            # FPS
            dt = t_mono - t_prev
            t_prev = t_mono
            if dt > 1e-4:
                fps_dts.append(dt)
            fps = (len(fps_dts) / sum(fps_dts)) if fps_dts else 0.0

            rec_active = rec_holder["rec"].active
            rec_str    = "REC" if rec_active else "off"
            n_moving   = len(detections)
            info_top = (f"{rec_str}  f={frame_count}  fps={fps:4.1f}  "
                        f"det={detect_count}/{frame_count}  moving={n_moving}")
            # Bottom info bar: CSI rate + per-Rx tripwire state + scores
            base_status   = tripwire.baseline_status()
            scores        = tripwire.snapshot_scores()
            tw_state_str  = "  ".join(
                f"rx{r}={base_status.get(r,'?')[:5]}/{scores.get(r,0):.2f}"
                for r in rx_ids
            )
            info_bot = csi.status_string() + "    " + tw_state_str

            # Recent transit pulled from the listener's ring; fade after 5 s.
            with csi.lock:
                latest_transit = csi.recent_transits[-1] if csi.recent_transits else None
            recent_transit_text = None
            if latest_transit is not None:
                tt, transit = latest_transit
                age = time.monotonic() - tt
                if age < 5.0:
                    spd = (f"{transit.speed_mps:.2f} m/s"
                           if transit.speed_mps is not None else "(no spacings)")
                    recent_transit_text = (
                        f"TRANSIT  dir={'->'.join(str(r) for r in transit.direction)}  "
                        f"speed={spd}  ({age:.1f}s ago)"
                    )

            # Translate per-Rx state ints to short strings for line flashing.
            tripwire_states_str = {
                r: TripwireDetector.STATE_NAMES.get(tripwire.latest_state.get(r), "?")
                for r in rx_ids
            }

            # Live values of every slider, so you can see what each one
            # is currently set to without leaning over the slider bar.
            tuning_parts = []
            if hasattr(backend, "conf"):
                tuning_parts.append(f"conf={backend.conf:.2f}")
            if hasattr(backend, "min_speed_pxs"):
                tuning_parts.append(f"min_spd={backend.min_speed_pxs:.0f}px/s")
            tuning_parts.append(f"csi_Thi={tripwire.threshold_high:.2f}")
            tuning_parts.append(f"csi_Tlo={tripwire.threshold_low:.2f}")
            tuning_text = "tune: " + "  ".join(tuning_parts)

            draw_overlay(frame, detections, rec_active, info_top, info_bot,
                         tripwire_lines=tripwire_lines,
                         recent_transit_text=recent_transit_text,
                         tripwire_states=tripwire_states_str,
                         tuning_text=tuning_text,
                         rx_positions_px=rx_positions_px or None,
                         csi_scores=scores,
                         mouse_pos=_mouse_pos[0])
            cv2.imshow(win, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('r'):
                if rec_holder["rec"].active:
                    rec_holder["rec"].stop()
                else:
                    rec_holder["rec"].start(args.record)
            if key == ord('b'):
                tripwire.start_capture_all(args.baseline_capture_sec)
                print(f"[TRIPWIRE] capturing baselines for "
                      f"{args.baseline_capture_sec:.1f} s on every Rx — keep still")
            if key == ord('c'):
                tripwire.clear_baselines()
                print("[TRIPWIRE] baselines cleared on every Rx")
    finally:
        rec_holder["rec"].stop()
        csi.shutdown()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
