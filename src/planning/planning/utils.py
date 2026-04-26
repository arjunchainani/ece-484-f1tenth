import numpy as np


def closest_point_on_path(path, point):
    """Find the closest point on a piecewise-linear path to a given point.

    Args:
        path: Nx2 array of [x, y] waypoints.
        point: [x, y] query point.

    Returns:
        (closest_point, tangent_vector, segment_index)
    """
    A = path[:-1]
    B = path[1:]

    vec_AB = B - A
    vec_AP = point - A

    t = np.sum(vec_AP * vec_AB, axis=1) / (np.sum(vec_AB ** 2, axis=1) + 1e-12)
    t = np.clip(t, 0.0, 1.0)

    candidates = A + t[:, np.newaxis] * vec_AB
    distances = np.linalg.norm(candidates - point, axis=1)
    best_idx = np.argmin(distances)

    return candidates[best_idx], vec_AB[best_idx], best_idx


def compute_xte(path, point):
    """Compute signed cross-track error from point to path.

    Positive XTE means the point is to the left of the path direction.

    Args:
        path: Nx2 array of waypoints.
        point: [x, y] query point.

    Returns:
        (xte, closest_point, segment_index)
    """
    closest, tangent, idx = closest_point_on_path(path, point)
    dist = np.linalg.norm(point - closest)

    vec_to_point = point - closest
    side = tangent[0] * vec_to_point[1] - tangent[1] * vec_to_point[0]
    xte = dist if side > 0 else -dist

    return xte, closest, idx
    

def quaternion_to_yaw(q):
    """Extract yaw from a quaternion (x, y, z, w) or geometry_msgs Quaternion."""
    if hasattr(q, 'x'):
        x, y, z, w = q.x, q.y, q.z, q.w
    else:
        x, y, z, w = q

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

    
    
    
    
