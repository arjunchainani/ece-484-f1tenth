"""Generate an oval track map (PNG + YAML) for f1tenth_gym_ros.

Track: two horizontal straights joined by semicircular ends. The centerline
traces a rectangle-with-rounded-ends shape. A pixel is "free" (white) iff
its distance to the centerline is within W/2.

Run:
    python3 generate_oval.py

Writes oval.png and oval.yaml in this directory.
"""

import os

import numpy as np
from PIL import Image


# Track geometry (meters)
R_CENTER = 4.0      # centerline radius at the curved ends
STRAIGHT_LEN = 6.0  # length of each straight
LANE_WIDTH = 2.5    # lane width
PADDING = 1.0       # free space past the outer wall for map framing

# Map resolution
RESOLUTION = 0.05   # meters per pixel


def signed_distance_to_centerline(wx, wy):
    """Perpendicular distance from world point (wx, wy) to the oval centerline.

    Centerline: y = ±R_CENTER for |x| ≤ STRAIGHT_LEN/2, plus semicircles of
    radius R_CENTER centered at (±STRAIGHT_LEN/2, 0).
    """
    hl = STRAIGHT_LEN / 2.0

    dist_straight = np.minimum(np.abs(wy - R_CENTER), np.abs(wy + R_CENTER))
    dist_right_arc = np.abs(np.hypot(wx - hl, wy) - R_CENTER)
    dist_left_arc = np.abs(np.hypot(wx + hl, wy) - R_CENTER)

    dist = np.where(
        wx > hl, dist_right_arc,
        np.where(wx < -hl, dist_left_arc, dist_straight),
    )
    return dist


def main():
    outer_radius = R_CENTER + LANE_WIDTH / 2.0
    half_len = STRAIGHT_LEN / 2.0

    x_extent = half_len + outer_radius + PADDING
    y_extent = outer_radius + PADDING

    width_px = int(np.ceil(2 * x_extent / RESOLUTION))
    height_px = int(np.ceil(2 * y_extent / RESOLUTION))

    # World X/Y of each pixel. Image row 0 is the top, which maps to world
    # y = +y_extent; row h-1 is the bottom, which maps to y = -y_extent.
    # ROS map origin = world coords of pixel (0, h-1): (-x_extent, -y_extent).
    xs = -x_extent + (np.arange(width_px) + 0.5) * RESOLUTION
    ys = +y_extent - (np.arange(height_px) + 0.5) * RESOLUTION
    X, Y = np.meshgrid(xs, ys)

    dist = signed_distance_to_centerline(X, Y)
    free = dist <= LANE_WIDTH / 2.0

    img = np.where(free, 255, 0).astype(np.uint8)
    here = os.path.dirname(os.path.abspath(__file__))
    png_path = os.path.join(here, 'oval.png')
    Image.fromarray(img, mode='L').save(png_path)

    yaml_path = os.path.join(here, 'oval.yaml')
    with open(yaml_path, 'w') as f:
        f.write(
            f"image: oval.png\n"
            f"resolution: {RESOLUTION:.6f}\n"
            f"origin: [{-x_extent:.6f}, {-y_extent:.6f}, 0.000000]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )

    print(f"Wrote {png_path} ({width_px}x{height_px} px)")
    print(f"Wrote {yaml_path}")
    print(f"Map spans x=[{-x_extent:.2f}, {x_extent:.2f}], "
          f"y=[{-y_extent:.2f}, {y_extent:.2f}] meters")
    print(f"Suggested start pose (bottom straight, heading +x):")
    print(f"  sx: 0.0")
    print(f"  sy: {-R_CENTER:.1f}")
    print(f"  stheta: 0.0")


if __name__ == '__main__':
    main()
