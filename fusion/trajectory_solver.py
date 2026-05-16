"""Trajectory fit from CSI motion-score time series at 3 Rx.

Bistatic-Fresnel model
----------------------
For each Rx_i at position R_i with the common Tx at T, a moving scatterer
at position p(t) perturbs the channel by an amount that depends on the
*detour*:

    detour_i(t) = |p(t) - T| + |p(t) - R_i| - |T - R_i|

This is the extra distance the scattered wave travels vs. the direct
Tx -> Rx path. detour = 0 when p is on the segment T-R_i; it grows as p
moves off the segment. The motion score has its peak when detour is
smallest and decays with a Fresnel-zone-width sigma:

    score_pred_i(t) = exp( - detour_i(t)^2 / (2 * sigma^2) )

We model the rock as a 2D constant-velocity trajectory p(t) = p0 + v*t
and fit (p0, v) by minimising

    sum over (Rx i, frame k) of  ( score_pred_i(t_k) - score_obs_i(t_k) )^2

This is a 4-unknown problem fit against ~30-80 observations - heavily
overdetermined and robust to per-frame noise.

We multi-start across a small grid of initial guesses (rock can come from
left, centre, or right) and keep the lowest-residual fit, which kills
local-minimum traps.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
from scipy.optimize import least_squares

from config import (
    CSIFrame,
    EXPECTED_DIRECTION_DEG,
    EXPECTED_GATE_Y_M,
    EXPECTED_SPEED_MPS,
    FRESNEL_SIGMA_M,
    RX_POSITIONS,
    SOLVER_BOUNDS_HI,
    SOLVER_BOUNDS_LO,
    TX_POSITION,
)


@dataclass
class TrajectoryFit:
    p0:           Tuple[float, float]
    v:            Tuple[float, float]
    t_ref_us:     int
    speed:        float
    bearing_deg:  float
    rmse:         float                  # residual RMS in score units
    n_frames:     int

    def position_at(self, t_us: int) -> Tuple[float, float]:
        dt = (t_us - self.t_ref_us) * 1e-6
        return (self.p0[0] + self.v[0] * dt, self.p0[1] + self.v[1] * dt)

    def gate_crossing(self, gate_y: float = EXPECTED_GATE_Y_M) -> Tuple[float, int] | None:
        if abs(self.v[1]) < 1e-6:
            return None
        dt = (gate_y - self.p0[1]) / self.v[1]
        x_cross = self.p0[0] + self.v[0] * dt
        return (x_cross, self.t_ref_us + int(dt * 1e6))


# ---------------------------------------------------------------------------
# Observation extraction
# ---------------------------------------------------------------------------
def _gather_observations(
    frames_by_rx: Dict[int, List[CSIFrame]], t_ref_us: int
) -> List[Tuple[float, Tuple[float, float], Tuple[float, float], float, float]]:
    """For each frame from each Rx, return:
        (dt_seconds, Tx_pos, Rx_pos, |Tx-Rx|, score_observed)
    """
    out = []
    tx = TX_POSITION
    for rx_id, frames in frames_by_rx.items():
        if rx_id not in RX_POSITIONS:
            continue
        rx = RX_POSITIONS[rx_id]
        seg_len = math.hypot(tx[0] - rx[0], tx[1] - rx[1])
        for f in frames:
            dt = (f.t_us - t_ref_us) * 1e-6
            out.append((dt, tx, rx, seg_len, float(f.score)))
    return out


# ---------------------------------------------------------------------------
# Residuals
# ---------------------------------------------------------------------------
def _residuals(params: np.ndarray, observations, sigma: float) -> np.ndarray:
    x0, y0, vx, vy = params
    out = np.empty(len(observations))
    inv_2sig2 = 1.0 / (2.0 * sigma * sigma)
    for k, (dt, tx, rx, seg_len, obs) in enumerate(observations):
        px = x0 + vx * dt
        py = y0 + vy * dt
        d_pT = math.hypot(px - tx[0], py - tx[1])
        d_pR = math.hypot(px - rx[0], py - rx[1])
        detour = d_pT + d_pR - seg_len
        if detour < 0.0:
            detour = 0.0                                 # numerical guard
        pred = math.exp(-(detour * detour) * inv_2sig2)
        out[k] = pred - obs
    return out


# ---------------------------------------------------------------------------
# Initial guesses
# ---------------------------------------------------------------------------
def _initial_guesses() -> List[np.ndarray]:
    angle  = math.radians(EXPECTED_DIRECTION_DEG)
    vy0    = EXPECTED_SPEED_MPS * math.sin(angle)
    vx_mag = max(0.5, abs(EXPECTED_SPEED_MPS * math.cos(angle)))
    y0     = max(-0.5, EXPECTED_GATE_Y_M - 0.5 * EXPECTED_SPEED_MPS * 0.1)
    guesses = []
    for x0 in (-0.8, 0.0, +0.8):
        for vx0 in (-vx_mag, 0.0, +vx_mag):
            guesses.append(np.array([x0, y0, vx0, vy0]))
    return guesses


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def fit_trajectory(frames_by_rx: Dict[int, List[CSIFrame]]) -> TrajectoryFit | None:
    """Fit a 2D constant-velocity trajectory through the bistatic-Fresnel
    perturbation curves observed at the three Rx.
    """
    all_frames: List[CSIFrame] = [f for fs in frames_by_rx.values() for f in fs]
    if len(all_frames) < 6:
        return None
    t_ref_us = min(f.t_us for f in all_frames)

    observations = _gather_observations(frames_by_rx, t_ref_us)
    if len(observations) < 6:
        return None

    bounds = (np.array(SOLVER_BOUNDS_LO), np.array(SOLVER_BOUNDS_HI))
    best       = None
    best_rmse  = math.inf
    for x0_guess in _initial_guesses():
        try:
            res = least_squares(
                _residuals,
                x0     = x0_guess,
                args   = (observations, FRESNEL_SIGMA_M),
                bounds = bounds,
                method = "trf",
                loss   = "soft_l1",
                max_nfev = 300,
            )
        except Exception:
            continue
        if not res.success:
            continue
        rmse_i = float(np.sqrt(np.mean(res.fun ** 2)))
        if rmse_i < best_rmse:
            best_rmse = rmse_i
            best = res

    if best is None:
        return None

    fx0, fy0, fvx, fvy = best.x
    speed   = math.hypot(fvx, fvy)
    bearing = math.degrees(math.atan2(fvy, fvx))
    return TrajectoryFit(
        p0          = (float(fx0), float(fy0)),
        v           = (float(fvx), float(fvy)),
        t_ref_us    = t_ref_us,
        speed       = float(speed),
        bearing_deg = float(bearing),
        rmse        = best_rmse,
        n_frames    = len(all_frames),
    )
