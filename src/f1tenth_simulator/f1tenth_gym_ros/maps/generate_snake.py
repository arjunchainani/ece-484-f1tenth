"""Generate a track that snakes left-right, then wraps back via 90 deg corners.

Same fillet construction as `generate_circuit.py`. The bottom of the
track is a 5-bend serpentine — bends alternate sign in the pattern
(+30, -60, +60, -60, +30) so total turn over the snake is 0 deg
(it enters heading east and exits heading east). Above the snake the
path wraps back around as a rectangle: 90 deg corner up, top straight
heading west, 90 deg corner down, left straight heading south, then
the bottom-left 90 deg corner brings it back into the snake. Four
right-angle corners contribute the 360 deg of total turning needed
to close a CCW polygon.

The snake-vertex radii (0.4 m) are below the car's 0.74 m minimum
turn radius — the controller is meant to track a centerline it
physically cannot follow exactly through the apex, relying on the
0.8 m lane width for clearance.

Run:
    /usr/bin/python3 generate_snake.py
"""

import math
import os

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.signal import sawtooth


# Layout: 4-corner rectangular wrap with a 5-bend snake along the bottom.
# Snake bends are at fixed segment length 0.5 m, computed offline so that
# the alternating +30 / -60 / +60 / -60 / +30 turns produce a symmetric
# zigzag that enters and exits heading east.
WAYPOINTS = [
    (-1.850, -1.000),   # V0  — bottom-left corner (south -> east, +90)
    (-0.866, -1.000),   # V1  — snake bend 1  (+30)
    (-0.433, -0.750),   # V2  — snake bend 2  (-60)
    ( 0.000, -1.000),   # V3  — snake bend 3  (+60)
    ( 0.433, -0.750),   # V4  — snake bend 4  (-60)
    ( 0.866, -1.000),   # V5  — snake bend 5  (+30)
    ( 1.850, -1.000),   # V6  — bottom-right corner (east -> north, +90)
    ( 1.850,  1.000),   # V7  — top-right corner    (north -> west, +90)
    (-1.850,  1.000),   # V8  — top-left corner     (west -> south, +90)
]
# Wrap corners at the car's tight-turn limit (0.85 m). Snake vertices use
# a much tighter 0.4 m to make the zigzag visually obvious; the car will
# necessarily run wider than the centerline through each snake apex.
RADII = [0.85, 0.40, 0.40, 0.40, 0.40, 0.40, 0.85, 0.85, 0.85]

LANE_WIDTH = 0.80
PADDING = 0.6
RESOLUTION = 0.005      # 5 mm/pixel

SAW_WAVELENGTH = 0.03
SAW_AMPLITUDE = SAW_WAVELENGTH / 2.0    # -> 90 deg tooth tips

CENTERLINE_DS = 0.002   # 2 mm sampling along centerline


def build_centerline(waypoints, radii, ds):
    """Return (xs, ys, arclengths) sampled along the filleted closed track.

    Sequence: arc at V_i, straight from V_i tangent-out to V_{i+1}
    tangent-in, arc at V_{i+1}, straight, ...
    """
    N = len(waypoints)
    P = np.array(waypoints, dtype=float)
    r = np.array(radii, dtype=float)

    TI = np.zeros((N, 2))
    TO = np.zeros((N, 2))
    arcs = []

    for i in range(N):
        prev_i = (i - 1) % N
        next_i = (i + 1) % N
        u = P[i] - P[prev_i]
        u /= np.linalg.norm(u)
        v = P[next_i] - P[i]
        v /= np.linalg.norm(v)

        dot = float(np.clip(u[0] * v[0] + u[1] * v[1], -1.0, 1.0))
        theta = math.acos(dot)
        cross = u[0] * v[1] - u[1] * v[0]
        sgn = 1.0 if cross > 0 else -1.0

        half = theta / 2.0
        t = r[i] * math.tan(half)
        TI[i] = P[i] - u * t
        TO[i] = P[i] + v * t

        if sgn > 0:
            n_in = np.array([-u[1], u[0]])
        else:
            n_in = np.array([u[1], -u[0]])
        center = TI[i] + n_in * r[i]

        a_start = math.atan2(TI[i][1] - center[1], TI[i][0] - center[0])
        a_end = math.atan2(TO[i][1] - center[1], TO[i][0] - center[0])
        if sgn > 0 and a_end < a_start:
            a_end += 2 * math.pi
        elif sgn < 0 and a_end > a_start:
            a_end -= 2 * math.pi
        arcs.append((center, r[i], a_start, a_end, sgn))

    for i in range(N):
        next_i = (i + 1) % N
        L = float(np.linalg.norm(P[next_i] - P[i]))
        t_out_i = float(np.linalg.norm(TO[i] - P[i]))
        t_in_next = float(np.linalg.norm(TI[next_i] - P[next_i]))
        if t_out_i + t_in_next >= L:
            raise ValueError(
                f"Fillet overlap on segment V{i} -> V{next_i}: "
                f"{t_out_i:.3f} + {t_in_next:.3f} >= {L:.3f}m. "
                f"Reduce radius at V{i} or V{next_i}."
            )

    xs, ys, ss = [], [], []
    s_cum = 0.0
    for i in range(N):
        center, rr, a_s, a_e, _ = arcs[i]
        sweep = a_e - a_s
        arc_len = abs(sweep) * rr
        n = max(8, int(arc_len / ds))
        a = np.linspace(a_s, a_e, n, endpoint=False)
        xs.extend((center[0] + rr * np.cos(a)).tolist())
        ys.extend((center[1] + rr * np.sin(a)).tolist())
        ss.extend((s_cum + np.abs(a - a_s) * rr).tolist())
        s_cum += arc_len

        next_i = (i + 1) % N
        dvec = TI[next_i] - TO[i]
        L = float(np.linalg.norm(dvec))
        n = max(8, int(L / ds))
        t = np.linspace(0.0, 1.0, n, endpoint=False)
        xs.extend((TO[i][0] + t * dvec[0]).tolist())
        ys.extend((TO[i][1] + t * dvec[1]).tolist())
        ss.extend((s_cum + t * L).tolist())
        s_cum += L

    return np.asarray(xs), np.asarray(ys), np.asarray(ss), s_cum, TO, TI


def main():
    xs, ys, arclen, total_len, TO, TI = build_centerline(
        WAYPOINTS, RADII, CENTERLINE_DS)

    half_w = LANE_WIDTH / 2.0 + SAW_AMPLITUDE + PADDING
    x_min = float(xs.min()) - half_w
    x_max = float(xs.max()) + half_w
    y_min = float(ys.min()) - half_w
    y_max = float(ys.max()) + half_w

    w_m = x_max - x_min
    h_m = y_max - y_min
    w_px = int(np.ceil(w_m / RESOLUTION))
    h_px = int(np.ceil(h_m / RESOLUTION))
    print(f"Map size: {w_px} x {h_px} px ({w_m:.2f} x {h_m:.2f} m)")

    px = np.clip(((xs - x_min) / RESOLUTION).astype(np.int64), 0, w_px - 1)
    py = np.clip(((y_max - ys) / RESOLUTION).astype(np.int64), 0, h_px - 1)

    centerline_absent = np.ones((h_px, w_px), dtype=bool)
    centerline_absent[py, px] = False

    arclen_raster = np.zeros((h_px, w_px), dtype=np.float32)
    arclen_raster[py, px] = arclen.astype(np.float32)

    print("Computing EDT...")
    dist_px, (iy, ix) = distance_transform_edt(
        centerline_absent, return_indices=True)
    dist_m = dist_px * RESOLUTION
    nearest_arclen = arclen_raster[iy, ix]

    wall_offset = SAW_AMPLITUDE * sawtooth(
        2.0 * np.pi * nearest_arclen / SAW_WAVELENGTH, width=0.5)

    free = dist_m <= (LANE_WIDTH / 2.0 - wall_offset)
    img = np.where(free, 255, 0).astype(np.uint8)

    here = os.path.dirname(os.path.abspath(__file__))
    png_path = os.path.join(here, 'snake.png')
    yaml_path = os.path.join(here, 'snake.yaml')
    Image.fromarray(img, mode='L').save(png_path)

    with open(yaml_path, 'w') as f:
        f.write(
            f"image: snake.png\n"
            f"resolution: {RESOLUTION:.6f}\n"
            f"origin: [{x_min:.6f}, {y_min:.6f}, 0.000000]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )

    # Spawn mid-left-side straight (V8 -> V0), heading south. The car will
    # immediately hit the bottom-left corner, traverse the snake, then the
    # bottom-right + top wrap — one lap exercises every feature.
    spawn_x = 0.5 * (TO[8][0] + TI[0][0])
    spawn_y = 0.5 * (TO[8][1] + TI[0][1])
    spawn_heading = math.atan2(TI[0][1] - TO[8][1], TI[0][0] - TO[8][0])

    print(f"Wrote {png_path}")
    print(f"Wrote {yaml_path}")
    print(f"Centerline length: {total_len:.2f} m, {len(xs)} samples")
    print(f"World extent: x=[{x_min:.2f}, {x_max:.2f}], "
          f"y=[{y_min:.2f}, {y_max:.2f}]")
    print(f"Spawn pose (mid left-side straight, facing into corner):")
    print(f"  sx: {spawn_x:.3f}")
    print(f"  sy: {spawn_y:.3f}")
    print(f"  stheta: {spawn_heading:.4f}")


if __name__ == '__main__':
    main()
