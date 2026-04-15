import os
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.interpolate import splprep, splev
from skimage.morphology import skeletonize as _sk_skeletonize


def load_map(yaml_path):
    """Load a ROS map YAML and its associated image.

    Returns:
        (free_mask, resolution, origin, img_height)
        free_mask: boolean 2D array, True = free space
    """
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    img_dir = os.path.dirname(yaml_path)
    img_path = os.path.join(img_dir, meta['image'])
    img = np.array(Image.open(img_path).convert('L'), dtype=np.float64) / 255.0

    occupied_thresh = meta.get('occupied_thresh', 0.65)
    negate = meta.get('negate', 0)

    if negate:
        img = 1.0 - img

    free_mask = img > occupied_thresh
    resolution = float(meta['resolution'])
    origin = [float(v) for v in meta['origin']]

    return free_mask, resolution, origin, img.shape[0]


def pixel_to_world(pixels, origin, resolution, img_height):
    """Convert pixel coordinates (col, row) to world coordinates (x, y).

    ROS map convention: origin is at bottom-left of image, y-axis is flipped.
    """
    world = np.zeros_like(pixels, dtype=np.float64)
    world[:, 0] = origin[0] + pixels[:, 0] * resolution
    world[:, 1] = origin[1] + (img_height - 1 - pixels[:, 1]) * resolution
    return world


def world_to_pixel(points, origin, resolution, img_height):
    """Convert world coordinates (x, y) to pixel coordinates (col, row)."""
    pixels = np.zeros_like(points, dtype=np.float64)
    pixels[:, 0] = (points[:, 0] - origin[0]) / resolution
    pixels[:, 1] = (img_height - 1) - (points[:, 1] - origin[1]) / resolution
    return pixels


def _skeletonize(mask):
    """Morphological thinning to a single-pixel skeleton."""
    return _sk_skeletonize(mask.astype(bool))


def _prune_branches(skeleton):
    """Remove branch endpoints iteratively until only cycles remain."""
    skel = skeleton.copy()
    struct = np.ones((3, 3), dtype=bool)

    changed = True
    while changed:
        changed = False
        neighbor_count = ndimage.convolve(
            skel.astype(np.int32), struct.astype(np.int32), mode='constant', cval=0
        ) - skel.astype(np.int32)

        endpoints = skel & (neighbor_count <= 1)
        if np.any(endpoints):
            skel[endpoints] = False
            changed = True

    return skel


def _order_loop(skeleton, start_row, start_col):
    """Order skeleton pixels into a continuous loop via nearest-neighbor traversal."""
    points = np.argwhere(skeleton)  # (row, col)
    if len(points) == 0:
        return np.empty((0, 2))

    # Find the starting point nearest to (start_row, start_col)
    dists = np.sqrt((points[:, 0] - start_row) ** 2 + (points[:, 1] - start_col) ** 2)
    start_idx = np.argmin(dists)

    visited = np.zeros(len(points), dtype=bool)
    order = [start_idx]
    visited[start_idx] = True

    for _ in range(len(points) - 1):
        curr = points[order[-1]]
        remaining = np.where(~visited)[0]
        if len(remaining) == 0:
            break
        d = np.sqrt(
            (points[remaining, 0] - curr[0]) ** 2
            + (points[remaining, 1] - curr[1]) ** 2
        )
        nearest = remaining[np.argmin(d)]
        visited[nearest] = True
        order.append(nearest)

    ordered = points[order]
    # Return as (col, row) to match (x_pixel, y_pixel) convention
    return np.column_stack([ordered[:, 1], ordered[:, 0]])


def smooth_and_resample(world_coords, spacing=0.2):
    """Fit a periodic cubic spline and resample at uniform arc-length spacing.

    Args:
        world_coords: Nx2 array of [x, y] in world coordinates (closed loop).
        spacing: desired distance between consecutive waypoints in meters.

    Returns:
        Mx2 resampled waypoints (closed loop, last point != first point).
    """
    coords = np.array(world_coords, dtype=np.float64)

    # Close the loop for spline fitting
    coords_closed = np.vstack([coords, coords[0]])

    # Compute cumulative arc length
    diffs = np.diff(coords_closed, axis=0)
    seg_lengths = np.sqrt(np.sum(diffs ** 2, axis=1))
    cum_length = np.concatenate([[0], np.cumsum(seg_lengths)])
    total_length = cum_length[-1]

    # Fit periodic spline
    # Use smoothing to handle pixel-level noise
    tck, u = splprep(
        [coords_closed[:, 0], coords_closed[:, 1]],
        u=cum_length / total_length,
        s=len(coords) * 0.1,
        per=True,
        k=3,
    )

    # Resample at uniform spacing
    n_points = int(total_length / spacing)
    u_new = np.linspace(0, 1, n_points, endpoint=False)
    x_new, y_new = splev(u_new, tck)

    return np.column_stack([x_new, y_new])


def compute_normals(centerline):
    """Compute outward-pointing unit normals at each centerline point.

    Normal is 90-deg CCW rotation of the tangent (points to the left).

    Args:
        centerline: Nx2 closed-loop waypoints.

    Returns:
        Nx2 unit normal vectors.
    """
    N = len(centerline)
    tangents = np.zeros((N, 2))
    for i in range(N):
        tangents[i] = centerline[(i + 1) % N] - centerline[(i - 1) % N]

    norms = np.linalg.norm(tangents, axis=1, keepdims=True)
    tangents = tangents / (norms + 1e-12)

    normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
    return normals


def compute_track_widths(centerline, normals, free_mask, resolution, origin, img_height):
    """Compute track width to left and right of each centerline point.

    Casts rays along +-normal until hitting a wall.

    Args:
        centerline: Nx2 world coordinates.
        normals: Nx2 unit normals (left-pointing).
        free_mask: boolean occupancy grid.
        resolution: meters per pixel.
        origin: [origin_x, origin_y, ...].
        img_height: pixel height of the image.

    Returns:
        (w_left, w_right) arrays of length N in meters.
    """
    N = len(centerline)
    w_left = np.zeros(N)
    w_right = np.zeros(N)

    step = resolution * 0.5
    max_dist = 10.0

    h, w = free_mask.shape

    for i in range(N):
        for sign, arr in [(1.0, w_left), (-1.0, w_right)]:
            direction = sign * normals[i]
            dist = 0.0
            while dist < max_dist:
                dist += step
                test = centerline[i] + dist * direction
                col = int(round((test[0] - origin[0]) / resolution))
                row = int(round((img_height - 1) - (test[1] - origin[1]) / resolution))

                if row < 0 or row >= h or col < 0 or col >= w:
                    break
                if not free_mask[row, col]:
                    break

            arr[i] = dist

    return w_left, w_right


def extract_centerline(map_yaml_path, spawn_x=0.0, spawn_y=0.0, spacing=0.2):
    """Full pipeline: map image -> smoothed centerline waypoints.

    Args:
        map_yaml_path: path to the map YAML file.
        spawn_x, spawn_y: spawn position in world coordinates (for ordering).
        spacing: desired waypoint spacing in meters.

    Returns:
        centerline: Nx2 array of [x, y] world coordinates.
        w_left: N array of left track widths (meters).
        w_right: N array of right track widths (meters).
        free_mask: the binary occupancy grid.
        resolution: meters per pixel.
        origin: map origin.
        img_height: image height.
    """
    free_mask, resolution, origin, img_height = load_map(map_yaml_path)

    # Skeletonize
    skeleton = _skeletonize(free_mask)

    # Prune branches to keep only cycles
    pruned = _prune_branches(skeleton)

    # If pruning removed everything, fall back to the raw skeleton
    if not np.any(pruned):
        pruned = skeleton

    # Convert spawn to pixel coords
    spawn_col = (spawn_x - origin[0]) / resolution
    spawn_row = (img_height - 1) - (spawn_y - origin[1]) / resolution

    # Order into continuous loop
    ordered_px = _order_loop(pruned, int(round(spawn_row)), int(round(spawn_col)))

    if len(ordered_px) < 10:
        raise RuntimeError(
            f"Centerline extraction failed: only {len(ordered_px)} points found. "
            "Check that the map has a closed loop of free space."
        )

    # Convert to world coordinates
    world_coords = pixel_to_world(ordered_px, origin, resolution, img_height)

    # Smooth and resample
    centerline = smooth_and_resample(world_coords, spacing=spacing)

    # Compute normals and track widths
    normals = compute_normals(centerline)
    w_left, w_right = compute_track_widths(
        centerline, normals, free_mask, resolution, origin, img_height
    )

    return centerline, w_left, w_right, free_mask, resolution, origin, img_height
