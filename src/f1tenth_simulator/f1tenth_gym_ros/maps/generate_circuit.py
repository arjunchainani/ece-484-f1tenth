"""Generate a traditional closed racing circuit for f1tenth_gym_ros.

The centerline is a closed polygon with each vertex replaced by a
circular arc of per-vertex radius, tangent to the incoming and
outgoing straight segments. The result is a track composed of
straights + arcs with sharp curvature discontinuities at each
segment boundary — the shape of a real race circuit, not a smoothly
curving blob.

8 vertices chosen to give varied corner profiles: one long main
straight, a couple of medium-radius sweepers, and a ~110° hairpin.

Walls carry the same right-angle sawtooth profile as the curvy map
(3 cm wavelength, 1.5 cm amplitude -> 90° tooth tips).

Run:
    /usr/bin/python3 generate_circuit.py
"""

import math
import os

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.signal import sawtooth


# Waypoints (meters), CCW. Footprint ~5.4 x 5.4 m (fits in 5 x 6 m).
# All centerline radii are >= 0.8 m so they exceed the car's minimum
# turn radius (~0.74 m at max steering). No hairpin (108° corner with
# r=0.7 is geometrically infeasible at this scale — the car physically
# can't turn that tight).
WAYPOINTS = [
    (-2.0, -2.0),   # V0 — main-straight start, 90° left
    ( 2.0, -2.0),   # V1 — end of main straight, 90° left
    ( 2.0,  2.0),   # V2 — top-right, 63° left
    ( 0.0,  2.0),   # V3 — top, 41° left (mild)
    (-2.0,  1.5),   # V4 — top-left, 76° left
]
RADII = [1.2, 1.2, 1.0, 0.8, 1.0]

LANE_WIDTH = 0.80
PADDING = 0.6
RESOLUTION = 0.005      # 5 mm/pixel

SAW_WAVELENGTH = 0.03
SAW_AMPLITUDE = SAW_WAVELENGTH / 2.0    # -> 90° tooth tips

CENTERLINE_DS = 0.002   # 2 mm sampling along centerline


def build_centerline(waypoints, radii, ds):
    """Return (xs, ys, arclengths) sampled along the filleted closed track.

    Sequence: arc at V_i, straight from V_i tangent-out to V_{i+1}
    tangent-in, arc at V_{i+1}, straight, ...
    """
    N = len(waypoints)
    P = np.array(waypoints, dtype=float)
    r = np.array(radii, dtype=float)

    TI = np.zeros((N, 2))   # tangent point entering each vertex's arc
    TO = np.zeros((N, 2))   # tangent point leaving each vertex's arc
    arcs = []               # (center, r, angle_start, angle_end, sign)

    for i in range(N):
        prev_i = (i - 1) % N
        next_i = (i + 1) % N
        u = P[i] - P[prev_i]
        u /= np.linalg.norm(u)
        v = P[next_i] - P[i]
        v /= np.linalg.norm(v)

        dot = float(np.clip(u[0] * v[0] + u[1] * v[1], -1.0, 1.0))
        theta = math.acos(dot)                   # unsigned turn angle
        cross = u[0] * v[1] - u[1] * v[0]
        sgn = 1.0 if cross > 0 else -1.0         # +1 = left turn (CCW)

        # Tangent distance: for a fillet of radius r at a vertex with turn
        # angle theta, the tangent points sit t = r * tan(theta/2) from the
        # vertex along each side. Earlier version had this inverted.
        half = theta / 2.0
        t = r[i] * math.tan(half)
        TI[i] = P[i] - u * t
        TO[i] = P[i] + v * t

        # Inward normal at the entering tangent point
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

    # Sanity check: adjacent tangent distances must fit within each straight.
    for i in range(N):
        next_i = (i + 1) % N
        L = float(np.linalg.norm(P[next_i] - P[i]))
        t_out_i = float(np.linalg.norm(TO[i] - P[i]))
        t_in_next = float(np.linalg.norm(TI[next_i] - P[next_i]))
        if t_out_i + t_in_next >= L:
            raise ValueError(
                f"Fillet overlap on segment V{i} -> V{next_i}: "
                f"{t_out_i:.2f} + {t_in_next:.2f} >= {L:.2f}m. "
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

    # Map bounds with padding for lane + sawtooth + framing
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

    # Right-angle symmetric triangle wave on wall offset
    wall_offset = SAW_AMPLITUDE * sawtooth(
        2.0 * np.pi * nearest_arclen / SAW_WAVELENGTH, width=0.5)

    free = dist_m <= (LANE_WIDTH / 2.0 - wall_offset)
    img = np.where(free, 255, 0).astype(np.uint8)

    here = os.path.dirname(os.path.abspath(__file__))
    png_path = os.path.join(here, 'circuit.png')
    yaml_path = os.path.join(here, 'circuit.yaml')
    Image.fromarray(img, mode='L').save(png_path)

    with open(yaml_path, 'w') as f:
        f.write(
            f"image: circuit.png\n"
            f"resolution: {RESOLUTION:.6f}\n"
            f"origin: [{x_min:.6f}, {y_min:.6f}, 0.000000]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )

    # Spawn on the V0->V1 straight, halfway between the two tangent points.
    spawn_x = 0.5 * (TO[0][0] + TI[1][0])
    spawn_y = 0.5 * (TO[0][1] + TI[1][1])
    spawn_heading = math.atan2(TI[1][1] - TO[0][1], TI[1][0] - TO[0][0])

    print(f"Wrote {png_path}")
    print(f"Wrote {yaml_path}")
    print(f"Centerline length: {total_len:.2f} m, {len(xs)} samples")
    print(f"World extent: x=[{x_min:.2f}, {x_max:.2f}], "
          f"y=[{y_min:.2f}, {y_max:.2f}]")
    print(f"Spawn pose (mid-straight, facing along track):")
    print(f"  sx: {spawn_x:.3f}")
    print(f"  sy: {spawn_y:.3f}")
    print(f"  stheta: {spawn_heading:.4f}")


if __name__ == '__main__':
    main()
