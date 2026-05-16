"""End-to-end CSI demo runner.

Usage
-----
    pip install -r requirements.txt
    python main.py                      # live: bind UDP 5005, await CSI from Rx ESP32s
    python main.py --simulate           # synthetic transit, no hardware

Live setup: flash ONE ESP32 with firmware/tx_node, THREE with firmware/rx_node,
edit fusion/config.py with the actual node positions, then run this.
"""

from __future__ import annotations

import argparse
import math
import random
import time
from collections import defaultdict
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np

from config import (
    CSIFrame,
    EXPECTED_GATE_Y_M,
    FRESNEL_SIGMA_M,
    GATE_AREA_Y_M,
    RX_POSITIONS,
    SUBCARRIERS,
    TX_POSITION,
)
from receiver import Event, open_udp, stream_events_udp
from trajectory_solver import TrajectoryFit, fit_trajectory


# ---------------------------------------------------------------------------
# Pretty printing
# ---------------------------------------------------------------------------
def print_fit(fit: TrajectoryFit) -> None:
    crossing = fit.gate_crossing(EXPECTED_GATE_Y_M)
    print("-" * 60)
    print(f"  TRAJECTORY FIT  ({fit.n_frames} frames, fit RMSE = {fit.rmse:.2f} m)")
    print(f"    speed      : {fit.speed:6.2f} m/s")
    print(f"    bearing    : {fit.bearing_deg:6.1f} deg  (90 = away from Tx)")
    print(f"    p0         : ({fit.p0[0]:+.2f}, {fit.p0[1]:+.2f}) m")
    print(f"    v          : ({fit.v[0]:+.2f}, {fit.v[1]:+.2f}) m/s")
    if crossing is not None:
        x_cross, _ = crossing
        print(f"    gate-cross : x = {x_cross:+.2f} m  at y = {EXPECTED_GATE_Y_M:.2f} m")
    print("-" * 60)


# ---------------------------------------------------------------------------
# Plot helper
# ---------------------------------------------------------------------------
def plot_event(event: Event, fit: TrajectoryFit | None) -> None:
    fig, (ax_xy, ax_v) = plt.subplots(1, 2, figsize=(12, 5))

    # Left panel: top-down geometry + trajectory
    tx = TX_POSITION
    ax_xy.plot(tx[0], tx[1], "s", color="#06b6d4", markersize=12, label="Tx")
    ax_xy.annotate("Tx", (tx[0], tx[1]), textcoords="offset points", xytext=(8, 8), fontsize=10)
    for rid, (rx, ry) in RX_POSITIONS.items():
        ax_xy.plot(rx, ry, "o", color="#22c55e", markersize=10)
        ax_xy.annotate(f"Rx{rid}", (rx, ry), textcoords="offset points", xytext=(8, 4), fontsize=10)
        # Bistatic Tx-Rx line
        ax_xy.plot([tx[0], rx], [tx[1], ry], color="#22c55e", linestyle=":", alpha=0.35, linewidth=1)
    ax_xy.axhline(EXPECTED_GATE_Y_M, color="gray", linestyle="--", linewidth=1,
                  label=f"gate y={EXPECTED_GATE_Y_M:.2f} m")

    if fit is not None:
        ts = np.linspace(-0.05, 0.30, 80)
        xs = fit.p0[0] + fit.v[0] * ts
        ys = fit.p0[1] + fit.v[1] * ts
        ax_xy.plot(xs, ys, color="#ef4444", linewidth=2.2, label="fit trajectory")
        crossing = fit.gate_crossing(EXPECTED_GATE_Y_M)
        if crossing is not None:
            ax_xy.plot(crossing[0], EXPECTED_GATE_Y_M, "*", color="#ef4444", markersize=14)

    ax_xy.set_aspect("equal")
    ax_xy.set_xlim(-3, 3)
    ax_xy.set_ylim(-1, 5)
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Top-down · Tx + 3 Rx")
    ax_xy.grid(alpha=0.3)
    ax_xy.legend(loc="upper right", fontsize=8)

    # Right panel: motion score per Rx
    colors = {1: "#22c55e", 2: "#3b82f6", 3: "#f59e0b"}
    for rid, frames in event.frames_by_rx.items():
        ts = np.array([f.t_us for f in frames]) - event.t_start_us
        ss = np.array([f.score for f in frames])
        ax_v.plot(ts / 1000.0, ss, "o-", color=colors.get(rid, "k"), label=f"Rx{rid}")
    ax_v.set_xlabel("time since event start [ms]")
    ax_v.set_ylabel("motion score (linear, 1.0 = on bistatic line)")
    ax_v.set_title("CSI motion · per Rx")
    ax_v.grid(alpha=0.3)
    ax_v.legend()

    if fit is not None:
        fig.suptitle(f"speed {fit.speed:.1f} m/s   bearing {fit.bearing_deg:.0f} deg   "
                     f"RMSE {fit.rmse:.2f} m",
                     fontsize=11)
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Live mode (UDP from 3 Rx ESP32s)
# ---------------------------------------------------------------------------
def run_live() -> None:
    from config import UDP_BIND_HOST, UDP_PORT
    print(f"Binding UDP {UDP_BIND_HOST}:{UDP_PORT} ...")
    sock = open_udp()
    print("Listening for CSI from 3 Rx nodes. Throw a rock through the gate!\n")
    try:
        for event in stream_events_udp(sock):
            print(f"\n>> EVENT: {event.total_frames()} frames from {len(event.frames_by_rx)} Rx")
            
            if event.frames_by_rx:
                first_rx_id = list(event.frames_by_rx.keys())[0]
                if event.frames_by_rx[first_rx_id]:
                    first_frame = event.frames_by_rx[first_rx_id][0]
                    print(f"   [Raw I/Q] Rx{first_rx_id} Real: {first_frame.real}")
                    print(f"   [Raw I/Q] Rx{first_rx_id} Imag: {first_frame.imag}")

            fit = fit_trajectory(event.frames_by_rx)
            if fit is None:
                print("   (fit failed)")
                continue
            print_fit(fit)
            plot_event(event, fit)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        sock.close()


# ---------------------------------------------------------------------------
# Simulation (no hardware)
# ---------------------------------------------------------------------------
def _bistatic_distance(p, T, R) -> float:
    """Bistatic range = d(T,p) + d(p,R). Equals |T-R| when p is on the segment."""
    d1 = math.hypot(p[0] - T[0], p[1] - T[1])
    d2 = math.hypot(p[0] - R[0], p[1] - R[1])
    return d1 + d2


def simulate_event(
    p0=(-0.6, -0.3),
    v =(2.5, 22.0),
    duration_s=0.35,
    fps=45.0,
    noise_score=0.025,                # additive noise on the linear score
    noise_rssi=0.5,
    seed=0,
) -> Event:
    """Synthesise a realistic CSI burst from a known trajectory.

    Motion score (LINEAR, same units the solver fits) follows the bistatic-
    Fresnel model: score = exp(-detour^2 / (2*sigma^2)) where detour is the
    extra path the wave travels via the scatterer vs. the direct Tx-Rx path.
    """
    rng = random.Random(seed)
    event = Event(t_start_us=int(time.monotonic() * 1e6))
    frames_by_rx: Dict[int, List[CSIFrame]] = defaultdict(list)

    n_frames = int(duration_s * fps)
    tx = TX_POSITION
    sigma2_inv = 1.0 / (2.0 * FRESNEL_SIGMA_M ** 2)
    for k in range(n_frames):
        t = k / fps
        p = (p0[0] + v[0] * t, p0[1] + v[1] * t)
        for rx_id, R in RX_POSITIONS.items():
            base_dist = math.hypot(tx[0] - R[0], tx[1] - R[1])
            detour = _bistatic_distance(p, tx, R) - base_dist
            score = math.exp(-(detour ** 2) * sigma2_inv)
            score = max(0.0, score + rng.gauss(0.0, noise_score))
            rssi  = -40.0 - 0.1 * (R[0] ** 2 + R[1] ** 2) + rng.gauss(0.0, noise_rssi)
            # Synthetic subcarrier amplitudes - quiet baseline + perturbation
            amp = [1.0 + 0.04 * rng.gauss(0, 1) + 0.35 * score * rng.gauss(0, 1)
                   for _ in range(SUBCARRIERS)]
            f = CSIFrame(
                rx_id = rx_id,
                t_us  = event.t_start_us + int(t * 1e6),
                rssi  = rssi,
                score = score,
                amp   = amp,
            )
            frames_by_rx[rx_id].append(f)

    event.frames_by_rx = frames_by_rx
    return event


def run_simulation() -> None:
    print("Running synthetic CSI transit (no hardware)...\n")
    truth_p0 = (-0.6, -0.3)
    truth_v  = (2.5, 22.0)
    truth_speed = math.hypot(*truth_v)
    truth_bear  = math.degrees(math.atan2(truth_v[1], truth_v[0]))
    print(f"  GROUND TRUTH:")
    print(f"    p0    = {truth_p0}")
    print(f"    v     = {truth_v}  ->  speed {truth_speed:.2f} m/s,"
          f" bearing {truth_bear:.1f} deg")
    print()

    event = simulate_event(p0=truth_p0, v=truth_v)
    print(f"Simulated {event.total_frames()} frames across "
          f"{len(event.frames_by_rx)} Rx.")
    fit = fit_trajectory(event.frames_by_rx)
    if fit is None:
        print("Fit failed!")
        return
    print_fit(fit)
    plot_event(event, fit)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulate", action="store_true",
                        help="Run a synthetic CSI transit (no hardware needed)")
    args = parser.parse_args()
    if args.simulate:
        run_simulation()
    else:
        run_live()


if __name__ == "__main__":
    main()
