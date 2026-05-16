"""Real-time CSI subcarrier amplitude visualiser.

Parallel to the official Espressif tool at
  https://github.com/espressif/esp-csi/blob/master/tools/csi_data_read_parse.py
which reads CSI from one ESP32 over USB serial. We instead consume the
JSON-over-UDP stream produced by our rx_node firmware - so all Rx render
in parallel side-by-side with no firmware changes.

Per Rx the plot shows:
    top      -- current amplitude across the 64 subcarriers (line)
    bottom   -- waterfall of the last N frames (subcarrier vs time, colour=amp)
    title    -- live motion score and RSSI for that Rx

Usage
-----
    python csi_viewer.py

NOTE: this binds UDP_PORT (5005). Stop `main.py` first - they can't share
the port. Use the viewer for sanity-checking the firmware/path; when it
looks healthy, kill it and run main.py for the trajectory pipeline.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Dict

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from config import RX_POSITIONS, UDP_BIND_HOST, UDP_PORT


# ---- Display parameters ---------------------------------------------------
WATERFALL_ROWS  = 200          # rows of history shown in the waterfall
MAX_SUBC        = 64           # 802.11n 20 MHz CSI buffer width
AMP_VMAX        = 80.0         # colour-scale ceiling (int8 magnitudes ~ <= 100)
PLOT_FPS        = 15           # animation refresh rate
WINDOW_TITLE    = "ESP32 CSI · live subcarrier amplitudes"


# ===========================================================================
# Threaded UDP receiver + ring buffer
# ===========================================================================
class CSIBuffer:
    """Thread-safe per-Rx rolling buffer of subcarrier amplitudes."""
    def __init__(self, rx_ids):
        self.lock = threading.Lock()
        self.waterfall: Dict[int, np.ndarray] = {
            r: np.zeros((WATERFALL_ROWS, MAX_SUBC), dtype=np.float32)
            for r in rx_ids
        }
        self.latest_amp: Dict[int, np.ndarray] = {
            r: np.zeros(MAX_SUBC, dtype=np.float32) for r in rx_ids
        }
        self.score:  Dict[int, float] = {r: 0.0   for r in rx_ids}
        self.rssi:   Dict[int, float] = {r: -90.0 for r in rx_ids}
        self.count:  Dict[int, int]   = {r: 0     for r in rx_ids}
        self.last_seen: Dict[int, float] = {r: 0.0 for r in rx_ids}

    def push(self, rx_id: int, amp_list, score: float, rssi: float) -> None:
        if rx_id not in self.waterfall:
            return                                # ignore unexpected Rx
        n = min(len(amp_list), MAX_SUBC)
        vec = np.zeros(MAX_SUBC, dtype=np.float32)
        vec[:n] = np.asarray(amp_list[:n], dtype=np.float32)
        with self.lock:
            self.latest_amp[rx_id] = vec
            buf = self.waterfall[rx_id]
            buf[:-1] = buf[1:]                    # scroll up
            buf[-1]  = vec                         # newest row at the bottom
            self.score[rx_id]    = score
            self.rssi[rx_id]     = rssi
            self.count[rx_id]   += 1
            self.last_seen[rx_id] = time.monotonic()


def udp_thread(buffer: CSIBuffer, stop_flag: threading.Event) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_BIND_HOST, UDP_PORT))
    sock.settimeout(0.2)
    while not stop_flag.is_set():
        try:
            data, _ = sock.recvfrom(8192)
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            obj = json.loads(data.decode("utf-8", errors="ignore"))
            rx    = int(obj["rx"])
            amp   = obj.get("amp", [])
            score = float(obj.get("score", 0.0))
            rssi  = float(obj.get("rssi", -90.0))
        except (ValueError, KeyError):
            continue
        if amp:
            buffer.push(rx, amp, score, rssi)
    sock.close()


# ===========================================================================
# Plot
# ===========================================================================
def build_figure(rx_ids):
    n_rx = len(rx_ids)
    fig, axes = plt.subplots(2, n_rx, figsize=(5.2 * n_rx, 7.4))
    if n_rx == 1:
        axes = np.array([[axes[0]], [axes[1]]])
    fig.canvas.manager.set_window_title(WINDOW_TITLE)
    fig.patch.set_facecolor("#0b0b0e")

    lines, images, titles = {}, {}, {}
    x = np.arange(MAX_SUBC)
    for col, rx in enumerate(rx_ids):
        # ---- amplitude line plot ----
        ax_a = axes[0, col]
        ax_a.set_facecolor("#0b0b0e")
        (line,) = ax_a.plot(x, np.zeros(MAX_SUBC), color="#22d3ee", linewidth=1.4)
        ax_a.set_xlim(0, MAX_SUBC)
        ax_a.set_ylim(0, AMP_VMAX * 1.1)
        ax_a.set_xlabel("subcarrier index", color="#a1a1aa", fontsize=9)
        ax_a.set_ylabel("|CSI|", color="#a1a1aa", fontsize=9)
        ax_a.tick_params(colors="#71717a", labelsize=8)
        for spine in ax_a.spines.values():
            spine.set_color("#3f3f46")
        ax_a.grid(True, color="#1f1f24", linewidth=0.5)
        title = ax_a.set_title(
            f"Rx {rx} · score 0.00 · rssi --",
            color="#e4e4e7", fontsize=10, loc="left"
        )
        lines[rx]  = line
        titles[rx] = title

        # ---- waterfall ----
        ax_w = axes[1, col]
        ax_w.set_facecolor("#0b0b0e")
        img = ax_w.imshow(
            np.zeros((WATERFALL_ROWS, MAX_SUBC), dtype=np.float32),
            aspect="auto", cmap="viridis",
            vmin=0.0, vmax=AMP_VMAX,
            origin="upper", interpolation="nearest",
        )
        ax_w.set_xlabel("subcarrier index", color="#a1a1aa", fontsize=9)
        ax_w.set_ylabel("frames ago", color="#a1a1aa", fontsize=9)
        ax_w.tick_params(colors="#71717a", labelsize=8)
        for spine in ax_w.spines.values():
            spine.set_color("#3f3f46")
        images[rx] = img

    fig.suptitle("CSI · per-Rx subcarrier amplitudes",
                 color="#e4e4e7", fontsize=12)
    plt.tight_layout(rect=(0, 0, 1, 0.97))
    return fig, lines, images, titles


def make_update_fn(buffer, lines, images, titles, rx_ids):
    def update(_frame):
        now = time.monotonic()
        with buffer.lock:
            for rx in rx_ids:
                lines[rx].set_ydata(buffer.latest_amp[rx])
                images[rx].set_data(buffer.waterfall[rx])
                age = now - buffer.last_seen[rx]
                state = "LIVE" if age < 1.5 else "stale" if buffer.count[rx] else "----"
                titles[rx].set_text(
                    f"Rx {rx} · {state} · score {buffer.score[rx]:.2f} · "
                    f"rssi {int(buffer.rssi[rx])} dBm · n={buffer.count[rx]}"
                )
        # Return the modified artists for blitting (we disable blit anyway).
        return list(lines.values()) + list(images.values())
    return update


# ===========================================================================
# Entry point
# ===========================================================================
def main():
    rx_ids = sorted(RX_POSITIONS.keys())
    print(f"CSI viewer: listening on UDP {UDP_BIND_HOST}:{UDP_PORT}")
    print(f"            expecting Rx: {rx_ids}")
    print( "            (stop main.py first - it binds the same port)\n")

    buffer    = CSIBuffer(rx_ids)
    stop_flag = threading.Event()
    t = threading.Thread(target=udp_thread, args=(buffer, stop_flag), daemon=True)
    t.start()

    fig, lines, images, titles = build_figure(rx_ids)
    update = make_update_fn(buffer, lines, images, titles, rx_ids)
    ani = animation.FuncAnimation(
        fig, update, interval=int(1000 / PLOT_FPS), blit=False, cache_frame_data=False
    )
    # Keep `ani` referenced so it isn't garbage-collected.
    fig._csi_ani = ani

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_flag.set()


if __name__ == "__main__":
    main()
