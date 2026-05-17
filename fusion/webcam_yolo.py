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
    the status bar, and (when ``recorder`` is active) appends every packet
    to ``csi.jsonl`` with a ``t_mono`` arrival timestamp."""

    def __init__(self, recorder_ref):
        self.recorder_ref = recorder_ref     # callable -> Optional[Recorder]
        self.stop_flag    = threading.Event()
        self.thread       = threading.Thread(target=self._run, daemon=True)
        self.lock         = threading.Lock()
        # rolling 1-sec rate per rx
        self.rx_arrivals: dict[int, deque] = {}
        self.last_rssi: dict[int, int]     = {}

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
    if width:  cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    if height: cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
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
        self.pos_fh = None
        self.csi_fh = None
        # Locked because the UDP thread writes csi.jsonl concurrently with
        # the main thread writing positions.jsonl + opening/closing files.
        self.lock   = threading.Lock()
        self.n_pos      = 0
        self.n_pos_det  = 0
        self.n_csi      = 0

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
            self.pos_fh    = open(p / "positions.jsonl", "w", encoding="utf-8")
            self.csi_fh    = open(p / "csi.jsonl",       "w", encoding="utf-8")
            self.path      = p
            self.n_pos     = 0
            self.n_pos_det = 0
            self.n_csi     = 0
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

    def stop(self) -> None:
        with self.lock:
            if not self.active:
                return
            self.pos_fh.close(); self.pos_fh = None
            self.csi_fh.close(); self.csi_fh = None
            print(f"[REC] stopped:  positions {self.n_pos_det}/{self.n_pos} detected,  "
                  f"csi {self.n_csi} packets  ->  {self.path}/")
            self.path = None


# ===========================================================================
# Overlay drawing
# ===========================================================================
def draw_overlay(frame, detections, rec_active, info_top, info_bot):
    h, w = frame.shape[:2]
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
    cv2.putText(frame, info_top, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 0, 255) if rec_active else (200, 200, 200), 2)
    cv2.putText(frame, info_bot, (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (200, 200, 200), 1)
    cv2.putText(frame, "R: rec    Q: quit", (10, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)


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
    ap.add_argument("--cam",    type=int, default=0)
    ap.add_argument("--width",  type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    # Recording
    ap.add_argument("--record", default=None,
                    help="run dir to start recording into (omit = idle, R toggles)")
    args = ap.parse_args()

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

    # Recorder — created idle, the CSI thread holds a callable to fetch the
    # current Recorder so its file handle changes are visible immediately
    # when we hit R.
    rec_holder = {"rec": Recorder(cam_w, cam_h, cam_fps, meta_extra)}
    def get_recorder() -> Recorder:
        return rec_holder["rec"]

    csi = CSIListener(get_recorder)
    csi.start()

    if args.record is not None:
        rec_holder["rec"].start(args.record)

    win = "Webcam + CSI recorder"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, cam_w, cam_h)

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
            n_moving = len(detections)
            info_top = (f"{rec_str}  f={frame_count}  fps={fps:4.1f}  "
                        f"det={detect_count}/{frame_count}  moving={n_moving}")
            info_bot = csi.status_string()
            draw_overlay(frame, detections, rec_active, info_top, info_bot)
            cv2.imshow(win, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('r'):
                if rec_holder["rec"].active:
                    rec_holder["rec"].stop()
                else:
                    rec_holder["rec"].start(args.record)
    finally:
        rec_holder["rec"].stop()
        csi.shutdown()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
