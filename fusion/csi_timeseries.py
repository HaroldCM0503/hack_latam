"""Real-time CSI subcarrier-amplitude time series.

Listens to the UDP stream produced by firmware/rx_node and plots all
subcarriers of each Rx as overlaid coloured lines. The X axis is time
(seconds ago → now); new samples arrive at the right edge and everything
scrolls left. The Y axis is per-subcarrier amplitude |CSI|. Each line's
colour is taken from a viridis colormap indexed by subcarrier number, so
adjacent subcarriers have visually close colours.

By default the viewer shows Rx 1, 2 and 3 stacked vertically (shared X-axis).
Pass `--rx N` to focus on a single Rx instead.

Baseline subtraction
--------------------
The figure has two buttons at the bottom:

    [ Capture Baselines ]   Starts a time-averaged baseline capture
                            (default 3 s window) on every visible Rx
                            INDEPENDENTLY. The room's "fingerprint" — i.e.
                            the steady multipath structure with no moving
                            object — gets averaged into one vector per Rx.
                            Sudden spikes get smoothed out by the average.
    [ Clear ]               Drops every baseline, viewer reverts to raw |CSI|.

After capture, each Rx's plot shows |amp - baseline| per subcarrier — the
fingerprint cancels out and only disruptions are visible. The capture state
('idle' / 'capturing X.Xs/3.0s' / 'baseline subtracted') is shown in each
Rx's title independently.

Usage
-----
    python csi_timeseries.py                    # all three Rx stacked
    python csi_timeseries.py --rx 2             # only Rx 2
    python csi_timeseries.py --seconds 10
    python csi_timeseries.py --autoscale
    python csi_timeseries.py --baseline-seconds 5

NOTE: this binds UDP_PORT (5005). Stop `main.py` first - they can't share
the port.
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from typing import Dict, List, Optional

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button

from config import UDP_BIND_HOST, UDP_PORT


# ---- Display parameters ---------------------------------------------------
MAX_SUBC               = 64           # 802.11n 20 MHz CSI buffer width
HISTORY_SAMPLES        = 600           # ring buffer length (newest at index -1)
AMP_VMAX               = 80.0          # initial Y-axis ceiling (raw mode)
DEFAULT_FPS            = 30
DEFAULT_SECONDS        = 5.0
DEFAULT_BASELINE_SEC   = 3.0           # length of the baseline-averaging window
DEFAULT_RX_IDS         = (1, 2, 3)     # shown stacked when --rx is not given


# ===========================================================================
# Threaded UDP receiver + per-Rx ring buffer
# ===========================================================================
class CSIBuffer:
    """Thread-safe rolling buffer of per-subcarrier amplitudes for ONE Rx.

    `amp` is (HISTORY_SAMPLES, MAX_SUBC). Newest sample is at row -1
    (the right edge of the plot). On every new packet we shift the entire
    array one row left (older) and write the new sample to row -1.

    Baseline capture is also done in `push()` so it follows the actual
    incoming sample stream (not the plot refresh tick): while
    `capture_active` is set, every arriving row is accumulated; when the
    capture window elapses the running mean is frozen into `baseline`.
    """

    def __init__(self, n_samples: int, n_subc: int):
        self.lock      = threading.Lock()
        self.amp       = np.zeros((n_samples, n_subc), dtype=np.float32)
        self.count     = 0
        self.last_seen = 0.0
        self.last_rssi = -90

        # Baseline state
        self.baseline:       Optional[np.ndarray] = None   # shape (n_subc,)
        self.capture_active: bool                 = False
        self.capture_start:  float                = 0.0
        self.capture_window: float                = DEFAULT_BASELINE_SEC
        self._capture_sum:   np.ndarray           = np.zeros(n_subc, dtype=np.float64)
        self._capture_count: int                  = 0

    def push(self, amp_list, rssi: int) -> None:
        n = min(len(amp_list), self.amp.shape[1])
        new_row = np.zeros(self.amp.shape[1], dtype=np.float32)
        new_row[:n] = np.asarray(amp_list[:n], dtype=np.float32)
        now = time.monotonic()
        with self.lock:
            # Roll the visible buffer.
            self.amp[:-1] = self.amp[1:]
            self.amp[-1]  = new_row
            self.count   += 1
            self.last_seen = now
            self.last_rssi = rssi

            # Accumulate / finalise the baseline capture.
            if self.capture_active:
                if now - self.capture_start <= self.capture_window:
                    self._capture_sum   += new_row.astype(np.float64)
                    self._capture_count += 1
                else:
                    if self._capture_count > 0:
                        self.baseline = (
                            self._capture_sum / self._capture_count
                        ).astype(np.float32)
                    self.capture_active = False

    # ---- Baseline control (called from button callbacks) ------------------
    def start_capture(self, window_sec: float) -> None:
        with self.lock:
            self.capture_active  = True
            self.capture_window  = window_sec
            self.capture_start   = time.monotonic()
            self._capture_sum.fill(0.0)
            self._capture_count  = 0
            # Note: we deliberately keep the old baseline visible during
            # capture, so the plot doesn't suddenly flicker between modes.
            # It is replaced atomically when the window completes.

    def clear_baseline(self) -> None:
        with self.lock:
            self.baseline       = None
            self.capture_active = False
            self._capture_count = 0
            self._capture_sum.fill(0.0)

    # ---- Snapshot for the plot --------------------------------------------
    def snapshot(self):
        with self.lock:
            return (
                self.amp.copy(),
                self.count,
                self.last_seen,
                self.last_rssi,
                None if self.baseline is None else self.baseline.copy(),
                self.capture_active,
                self.capture_start,
                self.capture_window,
            )


def udp_thread(buffers: Dict[int, CSIBuffer], stop_flag: threading.Event) -> None:
    """One UDP socket, dispatch by `rx` field in the JSON to the matching buffer."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_BIND_HOST, UDP_PORT))
    sock.settimeout(0.2)
    print(f"CSI viewer: listening on UDP {UDP_BIND_HOST}:{UDP_PORT}, "
          f"showing Rx {sorted(buffers.keys())}")
    while not stop_flag.is_set():
        try:
            data, _ = sock.recvfrom(8192)
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            obj = json.loads(data.decode("utf-8", errors="ignore"))
            rx_id = int(obj["rx"])
            if rx_id not in buffers:
                continue
            amp = obj.get("amp", [])
            if not amp:
                continue
            rssi = int(obj.get("rssi", -90))
        except (ValueError, KeyError, TypeError):
            continue
        buffers[rx_id].push(amp, rssi)
    sock.close()


# ===========================================================================
# Plot
# ===========================================================================
def build_figure(rx_ids: List[int], seconds: float):
    """One row per Rx, all sharing the same X axis (time). Single shared
    colour-bar on the right that maps subcarrier index → line colour."""
    n_rx = len(rx_ids)
    fig, axes = plt.subplots(
        n_rx, 1,
        figsize=(12, 2.4 * n_rx + 1.6),
        sharex=True,
    )
    if n_rx == 1:
        axes = [axes]

    # Reserve a strip at the bottom for the control buttons.
    fig.subplots_adjust(bottom=0.13, top=0.95, left=0.07, right=0.91,
                        hspace=0.35)

    fig.patch.set_facecolor("#0b0b0e")
    title_rx = "/".join(str(r) for r in rx_ids)
    fig.canvas.manager.set_window_title(
        f"ESP32 CSI · Rx {title_rx} · subcarrier amplitude time series"
    )

    x = np.linspace(-seconds, 0.0, HISTORY_SAMPLES)
    cmap = plt.get_cmap("viridis")

    lines_by_rx: Dict[int, list] = {}
    titles_by_rx: Dict[int, plt.Text] = {}

    for ax, rx in zip(axes, rx_ids):
        ax.set_facecolor("#0b0b0e")
        rx_lines = []
        for i in range(MAX_SUBC):
            color = cmap(i / max(1, MAX_SUBC - 1))
            (ln,) = ax.plot(x, np.zeros(HISTORY_SAMPLES),
                            color=color, linewidth=0.7, alpha=0.85)
            rx_lines.append(ln)
        lines_by_rx[rx] = rx_lines

        ax.set_xlim(-seconds, 0)
        ax.set_ylim(0, AMP_VMAX * 1.1)
        ax.set_ylabel("|CSI|", color="#a1a1aa", fontsize=9)
        ax.tick_params(colors="#71717a", labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#3f3f46")
        ax.grid(True, color="#1f1f24", linewidth=0.5)

        titles_by_rx[rx] = ax.set_title(
            f"Rx {rx} · waiting for packets…",
            color="#e4e4e7", fontsize=10, loc="left", pad=4,
        )

    axes[-1].set_xlabel("time ago (s)        →   now",
                       color="#a1a1aa", fontsize=10)

    # One shared subcarrier-index colour bar on the right of the whole figure.
    sm = plt.cm.ScalarMappable(
        cmap=cmap, norm=plt.Normalize(vmin=0, vmax=MAX_SUBC - 1))
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=axes, pad=0.015, aspect=40,
                        fraction=0.025)
    cbar.set_label("subcarrier index", color="#a1a1aa")
    cbar.ax.tick_params(colors="#71717a")
    cbar.outline.set_edgecolor("#3f3f46")

    return fig, axes, lines_by_rx, titles_by_rx


def _style_button(btn: Button, label_color: str) -> None:
    btn.label.set_color(label_color)
    btn.label.set_fontweight("bold")
    btn.ax.set_facecolor("#1e293b")
    for spine in btn.ax.spines.values():
        spine.set_color("#475569")


def add_buttons(fig, buffers: Dict[int, CSIBuffer], window_sec: float):
    """Add the Capture / Clear control buttons to the bottom strip and
    wire them up to every Rx's baseline machinery."""
    # Place buttons by (left, bottom, width, height) in figure coords.
    cap_ax   = fig.add_axes([0.30, 0.025, 0.22, 0.055])
    clear_ax = fig.add_axes([0.54, 0.025, 0.14, 0.055])

    cap_btn   = Button(cap_ax,   f"Capture Baselines ({window_sec:.0f} s)",
                       color="#1e293b", hovercolor="#334155")
    clear_btn = Button(clear_ax, "Clear", color="#1e293b", hovercolor="#3f1d1d")
    _style_button(cap_btn,   "#22d3ee")
    _style_button(clear_btn, "#f87171")

    def on_capture(_event):
        for buf in buffers.values():
            buf.start_capture(window_sec)
        print(f"[baseline] capture started for Rx {sorted(buffers.keys())} "
              f"({window_sec:.1f} s window)")

    def on_clear(_event):
        for buf in buffers.values():
            buf.clear_baseline()
        print("[baseline] cleared on every Rx")

    cap_btn.on_clicked(on_capture)
    clear_btn.on_clicked(on_clear)
    return cap_btn, clear_btn


def make_update_fn(
    buffers: Dict[int, CSIBuffer],
    axes,
    lines_by_rx: Dict[int, list],
    titles_by_rx: Dict[int, plt.Text],
    rx_ids: List[int],
    autoscale: bool,
):
    def update(_frame):
        now = time.monotonic()
        all_artists = []
        for ax, rx in zip(axes, rx_ids):
            (data, count, last_seen, last_rssi,
             baseline, capture_active, cap_start, cap_window) = buffers[rx].snapshot()

            # If a baseline is captured, plot |amp - baseline| per subcarrier
            # — the room's flat fingerprint cancels out, leaving only the
            # disruptions. Autoscale Y in this mode regardless of the CLI
            # flag because subtracted magnitudes are typically much smaller.
            mode_subtracted = baseline is not None
            if mode_subtracted:
                display = np.abs(data - baseline[np.newaxis, :])
            else:
                display = data

            lines = lines_by_rx[rx]
            for i, ln in enumerate(lines):
                ln.set_ydata(display[:, i])

            if mode_subtracted or autoscale:
                dmax = float(display.max()) if display.size else 1.0
                ax.set_ylim(0, max(dmax * 1.1, 1.0))
            else:
                ax.set_ylim(0, AMP_VMAX * 1.1)

            # Title: live/stale/----, packet count, RSSI, baseline state.
            age   = now - last_seen
            state = "LIVE" if age < 1.0 else "stale" if count else "----"
            if capture_active:
                elapsed = now - cap_start
                bstate = f"[capturing {min(elapsed, cap_window):.1f}/{cap_window:.0f} s]"
                bcolor = "#fbbf24"
            elif mode_subtracted:
                bstate = "[baseline subtracted]"
                bcolor = "#22d3ee"
            else:
                bstate = "[no baseline]"
                bcolor = "#71717a"
            titles_by_rx[rx].set_text(
                f"Rx {rx} · {state} · n={count} · rssi {last_rssi} dBm  {bstate}"
            )
            titles_by_rx[rx].set_color(bcolor if state == "LIVE" else "#e4e4e7")

            all_artists.extend(lines)
            all_artists.append(titles_by_rx[rx])
        return all_artists
    return update


# ===========================================================================
# Entry point
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--rx", type=int, default=None,
                    help="Rx node id to display. Default: 1, 2, 3 stacked.")
    ap.add_argument("--seconds", type=float, default=DEFAULT_SECONDS,
                    help=f"time window on the X axis (default: {DEFAULT_SECONDS} s)")
    ap.add_argument("--fps", type=int, default=DEFAULT_FPS,
                    help=f"plot refresh rate (default: {DEFAULT_FPS} Hz)")
    ap.add_argument("--autoscale", action="store_true",
                    help="autoscale Y per Rx to the data (default: fixed 0..88)")
    ap.add_argument("--baseline-seconds", type=float,
                    default=DEFAULT_BASELINE_SEC,
                    help=f"baseline averaging window length "
                         f"(default: {DEFAULT_BASELINE_SEC} s)")
    args = ap.parse_args()

    rx_ids = [args.rx] if args.rx is not None else list(DEFAULT_RX_IDS)

    buffers   = {rx: CSIBuffer(HISTORY_SAMPLES, MAX_SUBC) for rx in rx_ids}
    stop_flag = threading.Event()
    t = threading.Thread(
        target=udp_thread, args=(buffers, stop_flag), daemon=True,
    )
    t.start()

    fig, axes, lines_by_rx, titles_by_rx = build_figure(rx_ids, args.seconds)
    buttons = add_buttons(fig, buffers, args.baseline_seconds)

    update = make_update_fn(buffers, axes, lines_by_rx, titles_by_rx,
                            rx_ids, args.autoscale)
    ani = animation.FuncAnimation(
        fig, update,
        interval=int(1000 / args.fps),
        blit=False,
        cache_frame_data=False,
    )
    # Keep references so the GC doesn't drop the animation or button widgets.
    fig._csi_ani     = ani
    fig._csi_buttons = buttons

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_flag.set()


if __name__ == "__main__":
    main()
