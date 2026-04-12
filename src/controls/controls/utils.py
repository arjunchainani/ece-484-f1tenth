import math
import numpy as np


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
