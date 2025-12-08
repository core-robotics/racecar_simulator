#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Make smooth left/right track boundaries from a centerline by:
1) Reading center/left/right (raw) polylines.
2) Cleaning loops and ensuring consistent winding (left really on the left, right on the right).
3) Building robust tangents + left-hand unit normals from a smoothed centerline.
4) Estimating per-index widths by projecting the nearest raw boundary point onto the center normal.
   (Very simple + robust; no phase-search, no coarse/refine.)
5) Heavily smoothing the width signals, then reconstructing smooth boundaries:
      L = C + n * wL_smooth,  R = C - n * wR_smooth
6) Saving a combined CSV and plotting.

Notes
- This drops the complicated phase-alignment. Nearest-neighbor + projection
  works well when raw boundaries are reasonably close to the center.
- All smoothing windows are expressed in number of center samples and wrap around (closed loop).
- If your raw left/right are very noisy or sparse, this still produces clean, printable boundaries.
"""

from __future__ import annotations
import math
from pathlib import Path
from typing import Tuple
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ====== Paths (edit here) ======
CENTER_PATH = Path("/home/a/racecar_simulator/src/racecar_simulator/maps/f1tenth_racetracks/iccas2025/iccas2025_path.csv")
LEFT_PATH   = Path("/home/a/racecar_simulator/src/racecar_simulator/maps/f1tenth_racetracks/iccas2025/iccas2025_left.csv")
RIGHT_PATH  = Path("/home/a/racecar_simulator/src/racecar_simulator/maps/f1tenth_racetracks/iccas2025/iccas2025_right.csv")
OUT_CSV     = Path("/home/a/racecar_simulator/src/racecar_simulator/maps/f1tenth_racetracks/iccas2025/iccas2025_centerline_c.csv")
# =================================

# ---------------- I/O helpers ----------------

def read_xy(path: Path) -> pd.DataFrame:
    """Robust CSV/whitespace reader: keep first two numeric columns, ignore headers/comments."""
    df = pd.read_csv(path, sep=r"[,\s;]+", engine="python", header=None, comment="#", dtype=str)
    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(axis=1, how="all").dropna(axis=0, how="any")
    if df.shape[1] < 2:
        raise ValueError(f"Failed to parse at least two numeric columns from: {path}")
    out = df.iloc[:, :2].copy()
    out.columns = ["x_m", "y_m"]
    return out.reset_index(drop=True)

# ---------------- Geometry ----------------

def remove_duplicate_endpoint(xy: np.ndarray, tol: float = 1e-9) -> np.ndarray:
    xy = xy[~np.isnan(xy).any(axis=1)]
    if xy.shape[0] >= 2 and np.linalg.norm(xy[0] - xy[-1]) <= tol:
        xy = xy[:-1]
    return xy

def wrap_diff(xy: np.ndarray) -> np.ndarray:
    return np.diff(xy, axis=0, append=xy[:1])

def cumulative_arclen_param(xy: np.ndarray) -> np.ndarray:
    segs = np.linalg.norm(wrap_diff(xy), axis=1)
    total = float(segs.sum())
    if total <= 0:
        return np.linspace(0, 1, num=xy.shape[0], endpoint=False)
    s = np.cumsum(segs) - segs
    return (s / total)

def circular_moving_average(x: np.ndarray, win: int) -> np.ndarray:
    """Circular (periodic) moving average with odd window size."""
    n = len(x)
    win = max(1, int(win))
    if win % 2 == 0:
        win += 1
    k = win // 2
    # pad
    x_ext = np.concatenate([x[-k:], x, x[:k]])
    ker = np.ones(win) / win
    y = np.convolve(x_ext, ker, mode="valid")
    return y

def smooth_closed_polyline(xy: np.ndarray, win: int) -> np.ndarray:
    x = circular_moving_average(xy[:, 0], win)
    y = circular_moving_average(xy[:, 1], win)
    return np.column_stack([x, y])

def tangents_and_normals(center_xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    diffs = wrap_diff(center_xy)
    norms = np.clip(np.linalg.norm(diffs, axis=1, keepdims=True), 1e-12, None)
    t = diffs / norms
    n = np.column_stack([-t[:, 1], t[:, 0]])  # left-hand normal
    return t, n

# ---------------- Winding fix (left truly on the left) ----------------

def ensure_side(center_xy: np.ndarray, boundary_xy: np.ndarray, expect_left: bool) -> np.ndarray:
    """Flip boundary if the majority of boundary samples lie on the wrong side of the center normals."""
    _, n = tangents_and_normals(center_xy)
    # sample a subset to keep it cheap
    idx = np.linspace(0, len(center_xy) - 1, num=min(400, len(center_xy)), dtype=int)
    c_sub = center_xy[idx]
    n_sub = n[idx]
    # nearest vertex heuristic
    # (vectorized distance to a decimated boundary to avoid O(NM) blowup)
    B = boundary_xy[::max(1, len(boundary_xy)//max(800, len(boundary_xy)))]
    # for each c_sub, pick nearest in B
    d2 = ((c_sub[:, None, :] - B[None, :, :]) ** 2).sum(-1)
    j = d2.argmin(axis=1)
    v = B[j] - c_sub
    proj = (v * n_sub).sum(-1)  # >0 means on the left
    left_ratio = (proj > 0).mean()
    flip = (left_ratio < 0.5) if expect_left else (left_ratio > 0.5)
    return boundary_xy[::-1] if flip else boundary_xy

# ---------------- Nearest-neighbor projection widths ----------------

def nearest_vertex_widths(center_xy: np.ndarray, normals: np.ndarray, boundary_xy: np.ndarray, sign: int,
                           chunk: int = 4096) -> np.ndarray:
    """
    For each center point, find nearest boundary VERTEX (not segment) and project the vector on the normal.
    sign = +1 for left (along +n), -1 for right (along -n). Returns positive widths.
    Chunking avoids huge memory when arrays are large.
    """
    N = len(center_xy)
    M = len(boundary_xy)
    widths = np.zeros(N, dtype=float)
    for a in range(0, N, chunk):
        b = min(N, a + chunk)
        C = center_xy[a:b][:, None, :]  # (B,1,2)
        B = boundary_xy[None, :, :]     # (1,M,2)
        d2 = ((C - B) ** 2).sum(-1)     # (B,M)
        j = d2.argmin(axis=1)           # (B,)
        v = boundary_xy[j] - center_xy[a:b]
        proj = (v * normals[a:b]).sum(-1)  # signed
        widths[a:b] = np.maximum(0.0, sign * proj)
    return widths

# ---------------- Width smoothing ----------------

def circular_median(x: np.ndarray, win: int) -> np.ndarray:
    win = max(1, int(win))
    if win % 2 == 0:
        win += 1
    k = win // 2
    x_ext = np.concatenate([x[-k:], x, x[:k]])
    out = np.empty_like(x)
    for i in range(len(x)):
        out[i] = np.median(x_ext[i:i+win])
    return out

def smooth_widths(w: np.ndarray, med_win: int, mean_win: int) -> np.ndarray:
    if med_win > 1:
        w = circular_median(w, med_win)
    if mean_win > 1:
        w = circular_moving_average(w, mean_win)
    return w

# ---------------- Pipeline ----------------

def compute_smooth_boundaries(center_df: pd.DataFrame,
                              left_df: pd.DataFrame,
                              right_df: pd.DataFrame,
                              center_smooth_win: int = 9,
                              width_med_win: int = 51,
                              width_mean_win: int = 101):
    """Return (combined_df, C_smooth, L_smooth, R_smooth, wL_raw, wR_raw, wL_smooth, wR_smooth)."""
    # 1) Clean
    C = remove_duplicate_endpoint(center_df[["x_m", "y_m"]].to_numpy(float))
    L_raw = remove_duplicate_endpoint(left_df[["x_m", "y_m"]].to_numpy(float))
    R_raw = remove_duplicate_endpoint(right_df[["x_m", "y_m"]].to_numpy(float))

    # 2) Smooth centerline (coordinates), recompute tangents/normals on smoothed C
    C_s = smooth_closed_polyline(C, win=center_smooth_win)
    _, n = tangents_and_normals(C_s)

    # 3) Ensure left really lies on +n, right on -n
    L = ensure_side(C_s, L_raw, expect_left=True)
    R = ensure_side(C_s, R_raw, expect_left=False)

    # 4) Nearest-vertex widths (simple but robust)
    wL = nearest_vertex_widths(C_s, n, L, sign=+1)
    wR = nearest_vertex_widths(C_s, n, R, sign=-1)

    # 5) Smooth widths and clamp to reasonable range
    wL_s = smooth_widths(wL, med_win=width_med_win, mean_win=width_mean_win)
    wR_s = smooth_widths(wR, med_win=width_med_win, mean_win=width_mean_win)

    # low-pass artifacts guard
    wL_s = np.clip(wL_s, 0.0, np.percentile(wL, 99.5) * 1.2 + 1e-6)
    wR_s = np.clip(wR_s, 0.0, np.percentile(wR, 99.5) * 1.2 + 1e-6)

    # 6) Reconstruct smooth boundaries
    L_s = C_s + n * wL_s[:, None]
    R_s = C_s - n * wR_s[:, None]

    # 7) Output frame (aligned by center indices)
    out = pd.DataFrame({
        "x_m": C_s[:, 0],
        "y_m": C_s[:, 1],
        "left_x_m":  L_s[:, 0],
        "left_y_m":  L_s[:, 1],
        "right_x_m": R_s[:, 0],
        "right_y_m": R_s[:, 1],
        "w_tr_left_m":  wL_s,
        "w_tr_right_m": wR_s,
        "w_total_m":    wL_s + wR_s,
    })

    return out, C_s, L_s, R_s, wL, wR, wL_s, wR_s

# ---------------- Visualization ----------------

def show_track(center_xy: np.ndarray, left_xy: np.ndarray, right_xy: np.ndarray):
    plt.figure(figsize=(7, 7))
    plt.plot(center_xy[:, 0], center_xy[:, 1], label="center (smoothed)", linewidth=1.6)
    plt.plot(left_xy[:, 0],   left_xy[:, 1],   label="left (reconstructed)", linewidth=1.2)
    plt.plot(right_xy[:, 0],  right_xy[:, 1],  label="right (reconstructed)", linewidth=1.2)
    plt.axis("equal")
    plt.title("Track layout")
    plt.legend()
    plt.tight_layout()
    plt.show()

def show_widths(w_left_raw: np.ndarray, w_right_raw: np.ndarray, w_left: np.ndarray, w_right: np.ndarray):
    plt.figure(figsize=(10, 4))
    plt.plot(w_left_raw,  label="w_left_raw",  alpha=0.35)
    plt.plot(w_right_raw, label="w_right_raw", alpha=0.35)
    plt.plot(w_left,      label="w_left_smooth")
    plt.plot(w_right,     label="w_right_smooth")
    plt.grid(True)
    plt.xlabel("index along lap (center resolution)")
    plt.ylabel("width (m)")
    plt.title("Track widths (raw → smoothed)")
    plt.legend()
    plt.tight_layout()
    plt.show()

# ---------------- Main ----------------

def main():
    center_df = read_xy(CENTER_PATH)
    left_df   = read_xy(LEFT_PATH)
    right_df  = read_xy(RIGHT_PATH)

    out, C_s, L_s, R_s, wL_raw, wR_raw, wL_s, wR_s = compute_smooth_boundaries(
        center_df, left_df, right_df,
        center_smooth_win=3,     # increase for noisier centerlines
        width_med_win=7,        # robust outlier removal
        width_mean_win=7       # strong low-pass smoothing
    )

    # Save
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    out.to_csv(OUT_CSV, index=False)

    # Plots
    show_track(C_s, L_s, R_s)
    show_widths(wL_raw, wR_raw, out["w_tr_left_m"].to_numpy(), out["w_tr_right_m"].to_numpy())

    # Console
    print(f"Saved combined CSV to: {OUT_CSV} (rows={len(out)})")
    print(f"Left width  min/mean/max (raw):  {wL_raw.min():.3f}/{wL_raw.mean():.3f}/{wL_raw.max():.3f} m")
    print(f"Right width min/mean/max (raw):  {wR_raw.min():.3f}/{wR_raw.mean():.3f}/{wR_raw.max():.3f} m")
    print(f"Left width  min/mean/max (sm.):  {wL_s.min():.3f}/{wL_s.mean():.3f}/{wL_s.max():.3f} m")
    print(f"Right width min/mean/max (sm.):  {wR_s.min():.3f}/{wR_s.mean():.3f}/{wR_s.max():.3f} m")
    print(f"Total width mean (smoothed): {(wL_s + wR_s).mean():.3f} m")

if __name__ == "__main__":
    main()
