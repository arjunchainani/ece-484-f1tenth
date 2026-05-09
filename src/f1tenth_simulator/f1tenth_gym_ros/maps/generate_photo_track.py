"""Generate a custom track map traced from the provided track-photo sketch.

The reference sketch is 600 x 300 px and the requested scale is:

    1 px = 0.1 ft = 0.03048 m

This script keeps that pixel geometry so the generated ROS map has the same
physical dimensions as the sketch: 60 ft x 30 ft (18.288 m x 9.144 m).

Run:
    python generate_photo_track.py

Writes photo_track.png and photo_track.yaml in this directory.
"""

import math
import os

from PIL import Image, ImageDraw


WIDTH_PX = 600
HEIGHT_PX = 300
FEET_PER_PIXEL = 0.1
METERS_PER_FOOT = 0.3048
RESOLUTION = FEET_PER_PIXEL * METERS_PER_FOOT

WALL_WIDTH_PX = 11


def quadratic(p0, p1, p2, samples=20):
    """Sample a quadratic Bezier curve."""
    pts = []
    for i in range(samples):
        t = i / float(samples)
        u = 1.0 - t
        x = u * u * p0[0] + 2.0 * u * t * p1[0] + t * t * p2[0]
        y = u * u * p0[1] + 2.0 * u * t * p1[1] + t * t * p2[1]
        pts.append((x, y))
    pts.append(p2)
    return pts


def cubic(p0, p1, p2, p3, samples=24):
    """Sample a cubic Bezier curve."""
    pts = []
    for i in range(samples):
        t = i / float(samples)
        u = 1.0 - t
        x = (
            u * u * u * p0[0]
            + 3.0 * u * u * t * p1[0]
            + 3.0 * u * t * t * p2[0]
            + t * t * t * p3[0]
        )
        y = (
            u * u * u * p0[1]
            + 3.0 * u * u * t * p1[1]
            + 3.0 * u * t * t * p2[1]
            + t * t * t * p3[1]
        )
        pts.append((x, y))
    pts.append(p3)
    return pts


def draw_round_polyline(draw, points, width, fill, closed=False):
    """Draw a polyline with round caps/joints using circles at vertices."""
    int_points = [(int(round(x)), int(round(y))) for x, y in points]
    if closed:
        int_points = int_points + [int_points[0]]

    draw.line(int_points, fill=fill, width=width)

    radius = width // 2
    for x, y in int_points:
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=fill)


def draw_filled_path(draw, points, fill):
    int_points = [(int(round(x)), int(round(y))) for x, y in points]
    draw.polygon(int_points, fill=fill)


def build_outer_wall():
    pts = []
    pts.extend([(145, 5), (410, 5)])
    pts.extend(cubic((410, 5), (432, 5), (443, 12), (456, 31))[1:])
    pts.extend(quadratic((456, 31), (464, 42), (480, 42))[1:])
    pts.extend(cubic((480, 42), (482, 20), (495, 5), (520, 5))[1:])
    pts.extend([(558, 5)])
    pts.extend(cubic((558, 5), (581, 5), (596, 22), (596, 44))[1:])
    pts.extend([(596, 190)])
    pts.extend(cubic((596, 190), (596, 226), (574, 245), (540, 245))[1:])
    pts.extend([(154, 245)])
    pts.extend(cubic((154, 245), (123, 245), (105, 221), (105, 190))[1:])
    pts.extend([(105, 60)])
    pts.extend(cubic((105, 60), (105, 27), (116, 5), (145, 5))[1:])
    return pts


def build_outer_fill():
    """Approximate the area enclosed by the photographed outside wall."""
    pts = []
    pts.extend([(145, 10), (407, 10)])
    pts.extend(cubic((407, 10), (431, 10), (441, 16), (453, 36), 16)[1:])
    pts.extend([(478, 46)])
    pts.extend(cubic((478, 46), (486, 20), (499, 10), (521, 10), 16)[1:])
    pts.extend([(556, 10)])
    pts.extend(cubic((556, 10), (578, 10), (590, 25), (590, 45), 16)[1:])
    pts.extend([(590, 190)])
    pts.extend(cubic((590, 190), (590, 222), (570, 239), (538, 239), 16)[1:])
    pts.extend([(155, 239)])
    pts.extend(cubic((155, 239), (127, 239), (111, 218), (111, 190), 16)[1:])
    pts.extend([(111, 61)])
    pts.extend(cubic((111, 61), (111, 30), (121, 10), (145, 10), 16)[1:])
    return pts


def build_long_inner_wall():
    pts = []
    pts.extend([(411, 55)])
    pts.extend(quadratic((411, 55), (423, 71), (428, 84), 12)[1:])
    pts.extend(quadratic((428, 84), (431, 96), (416, 99), 12)[1:])
    pts.extend(cubic((416, 99), (371, 90), (291, 66), (228, 52), 24)[1:])
    pts.extend(cubic((228, 52), (194, 45), (160, 52), (160, 92), 24)[1:])
    pts.extend([(160, 146)])
    pts.extend(cubic((160, 146), (160, 174), (180, 184), (204, 184), 20)[1:])
    pts.extend(cubic((204, 184), (236, 184), (251, 166), (276, 164), 22)[1:])
    pts.extend(cubic((276, 164), (306, 161), (315, 178), (348, 174), 22)[1:])
    pts.extend(cubic((348, 174), (378, 170), (388, 158), (416, 169), 22)[1:])
    pts.extend(cubic((416, 169), (435, 177), (441, 183), (462, 183), 16)[1:])
    pts.extend([(504, 183)])
    pts.extend(cubic((504, 183), (525, 183), (535, 166), (535, 149), 18)[1:])
    pts.extend([(535, 65)])
    return pts


def build_inner_island():
    pts = []
    pts.extend([(375, 126)])
    pts.extend(cubic((375, 126), (335, 119), (275, 101), (235, 92), 24)[1:])
    pts.extend(cubic((235, 92), (221, 89), (212, 99), (212, 115), 18)[1:])
    pts.extend([(212, 133)])
    pts.extend(cubic((212, 133), (212, 148), (226, 151), (238, 146), 18)[1:])
    pts.extend(cubic((238, 146), (258, 136), (273, 133), (292, 135), 18)[1:])
    pts.extend(cubic((292, 135), (316, 137), (331, 148), (352, 142), 18)[1:])
    pts.extend(cubic((352, 142), (363, 139), (368, 131), (375, 126), 14)[1:])
    return pts


def pixel_to_world(col, row):
    """Convert image pixel coordinates to ROS map world coordinates."""
    x = col * RESOLUTION
    y = (HEIGHT_PX - 1 - row) * RESOLUTION
    return x, y


def main():
    img = Image.new("L", (WIDTH_PX, HEIGHT_PX), 0)
    draw = ImageDraw.Draw(img)

    # The photo shows walls on a flat floor. For simulator occupancy, the
    # floor enclosed by the outside wall is free and all walls are occupied.
    draw_filled_path(draw, build_outer_fill(), 255)
    draw_round_polyline(draw, build_outer_wall(), WALL_WIDTH_PX, 0, closed=True)
    draw_round_polyline(draw, build_long_inner_wall(), WALL_WIDTH_PX, 0)

    island = build_inner_island()
    draw_filled_path(draw, island, 0)
    draw_round_polyline(draw, island, WALL_WIDTH_PX, 0, closed=True)

    here = os.path.dirname(os.path.abspath(__file__))
    png_path = os.path.join(here, "photo_track.png")
    yaml_path = os.path.join(here, "photo_track.yaml")

    img.save(png_path)

    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(
            "image: photo_track.png\n"
            f"resolution: {RESOLUTION:.5f}\n"
            "origin: [0.000000, 0.000000, 0.000000]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )

    spawn_x, spawn_y = pixel_to_world(132, 137)
    spawn_heading = -math.pi / 2.0

    print(f"Wrote {png_path} ({WIDTH_PX}x{HEIGHT_PX} px)")
    print(f"Wrote {yaml_path}")
    print("Scale: 1 px = 0.1 ft = 0.03048 m")
    print(
        f"Map spans x=[0.000, {WIDTH_PX * RESOLUTION:.3f}], "
        f"y=[0.000, {HEIGHT_PX * RESOLUTION:.3f}] meters"
    )
    print("Suggested start pose at the checkered start line, heading down-track:")
    print(f"  sx: {spawn_x:.3f}")
    print(f"  sy: {spawn_y:.3f}")
    print(f"  stheta: {spawn_heading:.4f}")


if __name__ == "__main__":
    main()
