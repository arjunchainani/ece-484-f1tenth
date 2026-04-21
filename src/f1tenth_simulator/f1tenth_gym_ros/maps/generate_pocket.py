"""Generate a C-shaped track with a deep right-side pocket.

Same fillet construction as `generate_circuit.py`. The track is a
rectangular outer loop (5 x 5 m) with a 2.5 x 1.5 m notch carved into
the right side — so the perimeter has two convex outer "finger"
sections (top and bottom arms) and two concave corners at the back of
the notch where the path swings inward, then back outward. Going CCW,
the notch contributes the back-and-forth motion: the path enters the
notch heading west, turns north up the back wall, exits heading east.

Sized so each pair of "back-and-forth" corners around the notch has
a comfortable straight between them (1.5 m on the top/bottom of the
notch, 0.5 m on the back wall) — substantially longer than the snake
track's ~0.04 m free-straight between snake bends, which was the
specific complaint that motivated this design.

Run:
    /usr/bin/python3 generate_pocket.py
"""

import math
import os

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.signal import sawtooth


# 8 vertices, CCW. Outer corners (V0/V1/V6/V7) at the standard 0.85 m
# radius. Notch corners (V2..V5) at a tighter 0.5 m so the back-and-
# forth has long free straights between fillets without ballooning the
# overall footprint.
WAYPOINTS = [
    (-2.500, -2.500),   # V0  — bottom-left outer    (S->E, +90)
    ( 2.500, -2.500),   # V1  — bottom-right outer   (E->N, +90)
    ( 2.500, -0.750),   # V2  — bottom-arm tip       (N->W, +90, into notch)
    ( 0.000, -0.750),   # V3  — bottom of back wall  (W->N, -90, concave)
    ( 0.000,  0.750),   # V4  — top of back wall     (N->E, -90, concave)
    ( 2.500,  0.750),   # V5  — top-arm tip          (E->N, +90, out of notch)
    ( 2.500,  2.500),   # V6  — top-right outer      (N->W, +90)
    (-2.500,  2.500),   # V7  — top-left outer       (W->S, +90)
]
RADII = [0.85, 0.85, 0.50, 0.50, 0.50, 0.50, 0.85, 0.85]

LANE_WIDTH = 0.80
PADDING = 0.6
RESOLUTION = 0.005      # 5 mm/pixel

SAW_WAVELENGTH = 0.03
SAW_AMPLITUDE = SAW_WAVELENGTH / 2.0

CENTERLINE_DS = 0.002


def build_centerline(waypoints, radii, ds):
    """Return (xs, ys, arclengths) sampled along the filleted closed track."""
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
    png_path = os.path.join(here, 'pocket.png')
    yaml_path = os.path.join(here, 'pocket.yaml')
    Image.fromarray(img, mode='L').save(png_path)

    with open(yaml_path, 'w') as f:
        f.write(
            f"image: pocket.png\n"
            f"resolution: {RESOLUTION:.6f}\n"
            f"origin: [{x_min:.6f}, {y_min:.6f}, 0.000000]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )

    # Spawn mid-bottom-straight (V0 -> V1), heading east. Longest free
    # straight on the track; the car will hit the bottom-right corner,
    # head up the bottom arm, then traverse the notch (the back-and-
    # forth pocket) before coming out the top arm.
    spawn_x = 0.5 * (TO[0][0] + TI[1][0])
    spawn_y = 0.5 * (TO[0][1] + TI[1][1])
    spawn_heading = math.atan2(TI[1][1] - TO[0][1], TI[1][0] - TO[0][0])

    print(f"Wrote {png_path}")
    print(f"Wrote {yaml_path}")
    print(f"Centerline length: {total_len:.2f} m, {len(xs)} samples")
    print(f"World extent: x=[{x_min:.2f}, {x_max:.2f}], "
          f"y=[{y_min:.2f}, {y_max:.2f}]")
    print(f"Spawn pose (mid bottom straight, facing east):")
    print(f"  sx: {spawn_x:.3f}")
    print(f"  sy: {spawn_y:.3f}")
    print(f"  stheta: {spawn_heading:.4f}")


if __name__ == '__main__':
    main()
