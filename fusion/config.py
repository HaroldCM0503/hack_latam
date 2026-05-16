"""Geometry, calibration and event-windowing constants for the WiFi-CSI
passive sensing system.

Topology
--------
   1x Tx (continuously injecting WiFi packets, channel 6)
   3x Rx (in promiscuous mode, capturing CSI for each packet from Tx)
   PC   (UDP listener, motion fusion, trajectory solver)

Each moving object perturbs the channel between Tx and each Rx. The
strength of the perturbation peaks when the object is near the bistatic
line (Tx <-> Rx). Three Rx with known positions give us three "object
was near this line at this time" constraints, which together pin down a
2D trajectory.

Coordinate frame (2D, top-down view of the demo area, metres):
    x = left/right
    y = away from Tx, into the gate volume
    origin = Tx
EDIT THESE FOR YOUR ACTUAL ROOM LAYOUT.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

# ----- Hardware constants (CSI) -----
WIFI_CHANNEL  = 6                     # 802.11 channel both Tx and Rx live on
SUBCARRIERS   = 52                    # 802.11n 20 MHz HT data subcarriers
                                      # (raw CSI buffer carries 64; ~12 are pilot/null)

# ----- Node positions (metres) -----
TX_POSITION: Tuple[float, float] = (0.0, 0.0)
RX_POSITIONS: Dict[int, Tuple[float, float]] = {
    1: (-1.20, 2.40),
    2: (+1.20, 2.40),
    # 3: ( 0.00, 3.30),
}
GATE_AREA_Y_M = 2.0                    # nominal y of the rock-crossing line for display

# ----- Expected demo trajectory (priors for the solver's initial guess) -----
EXPECTED_GATE_Y_M      = 2.0
EXPECTED_SPEED_MPS     = 18.0
EXPECTED_DIRECTION_DEG = 90.0          # 90 deg = +y axis (away from Tx)

# ----- Network -----
UDP_BIND_HOST = "0.0.0.0"
UDP_PORT      = 5005

# ----- Motion-score thresholding / event windowing -----
# A frame's "motion score" is the normalised perturbation of its CSI amplitude
# vector relative to a rolling baseline (LINEAR, dimensionless):
#     score = ||amp - mean(amp_baseline)|| / ||mean(amp_baseline)||
# ~0 in a quiet scene, ~0.3-1.0 when something moves near a bistatic line.
MOTION_SCORE_TRIGGER = 0.18            # open an event when any Rx exceeds this
EVENT_WINDOW_MS      = 500             # collect this much data after first trigger
MIN_FRAMES_PER_RX    = 4               # minimum frames each Rx must contribute to fit

# Fresnel-zone effective width (metres). Tunes the bistatic perturbation model:
#   score_pred(p) = exp( - detour(p)^2 / (2 * sigma^2) )
# where detour(p) = (|p-Tx| + |p-Rx|) - |Tx-Rx|.
# At 2.4 GHz / 20 MHz with ~2.5 m Tx-Rx separation, the first Fresnel zone
# radius at midpoint is ~0.4-0.5 m.
FRESNEL_SIGMA_M      = 0.55

# ----- Solver bounds (constant-velocity 2D trajectory p(t) = p0 + v*t) -----
# Order: (x0_min, y0_min, vx_min, vy_min) / (x0_max, y0_max, vx_max, vy_max)
SOLVER_BOUNDS_LO = (-3.0, -1.0, -30.0,  1.0)
SOLVER_BOUNDS_HI = ( 3.0,  5.0,  30.0, 50.0)


@dataclass
class CSIFrame:
    """One CSI snapshot received at one Rx from one Tx packet."""
    rx_id:   int                       # 1, 2, or 3
    t_us:    int                       # laptop-side arrival time (microseconds)
    rssi:    float                     # dBm
    score:   float                     # motion-perturbation score (dB above baseline)
    amp:     List[float] = field(default_factory=list)  # optional per-subcarrier amps
    real:    List[int] = field(default_factory=list)    # raw real
    imag:    List[int] = field(default_factory=list)    # raw imag

    def t_seconds(self) -> float:
        return self.t_us * 1e-6


# (Legacy) serial port - kept for reference if you ever go back to USB tether
SERIAL_PORT = "COM5"
SERIAL_BAUD = 115200
