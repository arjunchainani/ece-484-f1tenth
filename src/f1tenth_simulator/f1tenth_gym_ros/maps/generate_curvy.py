"""Generate a curvy track with sawtooth walls for f1tenth_gym_ros.

Centerline is a 3-lobed rose r(theta) = R + A*cos(3*theta) in polar
coordinates. With A small enough relative to R the curve stays convex
around the origin, but the sign of the curvature still flips between
each lobe tip and the next — so tracing CCW you get alternating tight
left turns (at lobe tips) and gentler right-hand S-curves (between
lobes).

Each wall carries a right-angle sawtooth (symmetric triangle wave)
profile normal to the centerline. The triangle's peak angle is 90°
exactly when amplitude = wavelength/2, which is what we use.

Resolution is 5 mm/pixel so the 3 cm wavelength and 60 cm lane width
are actually resolvable on the bitmap.

Run:
    /usr/bin/python3 generate_curvy.py
"""

import os

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.signal import sawtooth


# Track geometry (meters)
R_BASE = 2.8            # mean centerline radius
LOBE_AMP = 0.7          # lobe amplitude (controls curve severity)
N_LOBES = 3             # number of lobes -> alternating curvature
LANE_WIDTH = 0.80       # 80 cm
PADDING = 0.4           # free space past outer wall for map framing

# Wall sawtooth profile (meters)
SAW_WAVELENGTH = 0.03   # 3 cm, per spec
SAW_AMPLITUDE = SAW_WAVELENGTH / 2.0   # amp = lambda/2 -> 90° teeth

# Map resolution
RESOLUTION = 0.005      # 5 mm/pixel

# Centerline sampling. Want inter-sample spacing well under RESOLUTION
# to avoid gaps when rasterizing.
N_CENTERLINE_SAMPLES = 20000


def centerline_samples():
    """Return (x, y, arclen) arrays sampled along the closed centerline."""
    t = np.linspace(0.0, 2.0 * np.pi, N_CENTERLINE_SAMPLES, endpoint=False)
    r = R_BASE + LOBE_AMP * np.cos(N_LOBES * t)
    x = r * np.cos(t)
    y = r * np.sin(t)

    # Cumulative arclength around the closed loop (starting from t=0).
    dx = np.diff(x, append=x[0])
    dy = np.diff(y, append=y[0])
    ds = np.hypot(dx, dy)
    s = np.concatenate([[0.0], np.cumsum(ds[:-1])])
    return x, y, s


def main():
    x, y, s = centerline_samples()
    total_length = s[-1] + np.hypot(x[0] - x[-1], y[0] - y[-1])

    extent = R_BASE + LOBE_AMP + LANE_WIDTH / 2.0 + PADDING
    w_px = int(np.ceil(2 * extent / RESOLUTION))
    h_px = int(np.ceil(2 * extent / RESOLUTION))

    # Rasterize centerline. Image row 0 = top = world y = +extent.
    px = np.clip(((x + extent) / RESOLUTION).astype(np.int64), 0, w_px - 1)
    py = np.clip(((extent - y) / RESOLUTION).astype(np.int64), 0, h_px - 1)

    # Binary mask: True where centerline is absent (required by EDT).
    centerline_absent = np.ones((h_px, w_px), dtype=bool)
    centerline_absent[py, px] = False

    # Arclength lookup table on the raster grid — only valid at centerline
    # pixels, but EDT's nearest-index field tells us which pixel to read.
    arclen_raster = np.zeros((h_px, w_px), dtype=np.float32)
    arclen_raster[py, px] = s.astype(np.float32)

    # Distance transform + nearest-centerline-pixel indices.
    dist_px, (iy, ix) = distance_transform_edt(
        centerline_absent, return_indices=True)
    dist_m = dist_px * RESOLUTION
    nearest_arclen = arclen_raster[iy, ix]

    # Right-angle symmetric triangle wave: sawtooth(..., width=0.5).
    # Returns values in [-1, +1]; scale by amplitude.
    wall_offset = SAW_AMPLITUDE * sawtooth(
        2.0 * np.pi * nearest_arclen / SAW_WAVELENGTH, width=0.5)

    # Both walls share the same arclength at any given pixel, so a positive
    # offset pinches the lane from both sides simultaneously (teeth at the
    # same arclength position on both walls). That's a literal sawtooth
    # surface on each wall.
    free = dist_m <= (LANE_WIDTH / 2.0 - wall_offset)

    img = np.where(free, 255, 0).astype(np.uint8)

    here = os.path.dirname(os.path.abspath(__file__))
    png_path = os.path.join(here, 'curvy.png')
    yaml_path = os.path.join(here, 'curvy.yaml')
    Image.fromarray(img, mode='L').save(png_path)

    with open(yaml_path, 'w') as f:
        f.write(
            f"image: curvy.png\n"
            f"resolution: {RESOLUTION:.6f}\n"
            f"origin: [{-extent:.6f}, {-extent:.6f}, 0.000000]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )

    # Start pose: lobe tip at theta=0 is world (R_BASE + LOBE_AMP, 0); the
    # tangent there is (0, +1), i.e. heading +y (pi/2 radians).
    sx = R_BASE + LOBE_AMP
    sy = 0.0
    stheta = np.pi / 2.0

    print(f"Wrote {png_path} ({w_px}x{h_px} px)")
    print(f"Wrote {yaml_path}")
    print(f"Map extent: ±{extent:.3f} m ({2*extent:.2f} m across)")
    print(f"Centerline length: {total_length:.2f} m")
    print(f"Lane width: {LANE_WIDTH*100:.0f} cm")
    print(f"Sawtooth: wavelength={SAW_WAVELENGTH*100:.1f} cm, "
          f"amplitude={SAW_AMPLITUDE*100:.2f} cm (90° teeth)")
    print(f"Suggested start pose (lobe tip @ theta=0, heading +y):")
    print(f"  sx: {sx:.3f}")
    print(f"  sy: {sy:.3f}")
    print(f"  stheta: {stheta:.4f}  # pi/2")


if __name__ == '__main__':
    main()
