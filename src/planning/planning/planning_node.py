import os
import math
import copy
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

from .min_curvature import generate_raceline
from .utils import compute_xte


class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')

        # Declare parameters
        self.declare_parameter('map_path', 'levine')
        self.declare_parameter('map_img_ext', '.png')
        self.declare_parameter('waypoint_spacing', 0.2)
        self.declare_parameter('v_max', 10.0)
        self.declare_parameter('v_min', 1.5)
        self.declare_parameter('a_lat_max', 6.0)
        self.declare_parameter('a_lon_max', 6.0)
        self.declare_parameter('xte_threshold', 0.15)
        self.declare_parameter('k_xte', 0.8)
        self.declare_parameter('planning_frequency', 20.0)
        self.declare_parameter('vehicle_width', 0.31)
        # Spawn pose (should match sim.yaml)
        self.declare_parameter('sx', 0.0)
        self.declare_parameter('sy', 0.0)
        self.declare_parameter('stheta', 0.0)

        # Read parameters
        self.v_max = self.get_parameter('v_max').value
        self.v_min = self.get_parameter('v_min').value
        self.a_lat_max = self.get_parameter('a_lat_max').value
        self.a_lon_max = self.get_parameter('a_lon_max').value
        self.xte_threshold = self.get_parameter('xte_threshold').value
        self.k_xte = self.get_parameter('k_xte').value
        vehicle_width = self.get_parameter('vehicle_width').value
        spacing = self.get_parameter('waypoint_spacing').value
        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value

        self.spacing = spacing

        # Raceline state — populated asynchronously
        self.raceline = None
        self.centerline = None
        self.w_left = None
        self.w_right = None

        # Waypoint tracking state
        self.current_wp_idx = 0
        self.skip_radius = 0.3

        # Latest odometry
        self.current_odom = None

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/ego_racecar/scan', self.scan_callback, 10)

        # Publisher
        self.trajectory_pub = self.create_publisher(Path, '/planning/trajectory', 10)

        # Planning timer
        freq = self.get_parameter('planning_frequency').value
        self.plan_timer = self.create_timer(1.0 / freq, self.plan_callback)

        # Kick off raceline generation on a background thread so __init__
        # returns immediately and the publisher/timer are live.
        map_name = self.get_parameter('map_path').value
        map_yaml = self._resolve_map_path(map_name)
        self.get_logger().info(
            f'Planning node initialized; generating raceline for map: {map_name} in background...'
        )
        self._raceline_thread = threading.Thread(
            target=self._generate_raceline_async,
            args=(map_yaml, sx, sy, stheta, vehicle_width, spacing),
            daemon=True,
        )
        self._raceline_thread.start()

    def _generate_raceline_async(self, map_yaml, sx, sy, stheta, vehicle_width, spacing):
        try:
            raceline, centerline, w_left, w_right = generate_raceline(
                map_yaml, sx, sy, stheta, vehicle_width, spacing
            )
        except Exception as e:
            self.get_logger().error(f'Raceline generation failed: {e}')
            return

        self.centerline = centerline
        self.w_left = w_left
        self.w_right = w_right
        # Assign raceline last — plan_callback gates on this field.
        self.raceline = raceline
        self.get_logger().info(
            f'Raceline generated: {len(raceline)} waypoints, '
            f'track length ~{len(raceline) * spacing:.1f}m'
        )

    def _resolve_map_path(self, map_name):
        """Resolve map name to the full YAML path."""
        # Try f1tenth_gym_ros package share first
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('f1tenth_gym_ros')
            yaml_path = os.path.join(pkg_share, 'maps', map_name + '.yaml')
            if os.path.exists(yaml_path):
                return yaml_path
        except Exception:
            pass

        # Fallback: search relative to this package
        base = os.path.dirname(os.path.abspath(__file__))
        for search in [
            os.path.join(base, '..', '..', '..', 'f1tenth_simulator',
                         'f1tenth_gym_ros', 'maps', map_name + '.yaml'),
        ]:
            if os.path.exists(search):
                return os.path.abspath(search)

        raise FileNotFoundError(
            f"Could not find map YAML for '{map_name}'. "
            "Ensure f1tenth_gym_ros is built and installed."
        )

    def odom_callback(self, msg):
        self.current_odom = msg

    def scan_callback(self, msg):
        pass  # Reserved for future perception integration

    def _extract_state(self):
        """Extract (x, y, yaw, velocity) from current odometry."""
        odom = self.current_odom
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        q = odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        vel = math.hypot(vx, vy)

        return x, y, yaw, vel

    def _advance_waypoint(self, curr_x, curr_y):
        """Advance current_wp_idx to the closest waypoint ahead."""
        N = len(self.raceline)
        # Search forward from current index
        search_range = min(N, 50)
        min_dist = float('inf')
        best_offset = 0

        for offset in range(search_range):
            idx = (self.current_wp_idx + offset) % N
            dx = self.raceline[idx, 0] - curr_x
            dy = self.raceline[idx, 1] - curr_y
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                min_dist = dist
                best_offset = offset

        self.current_wp_idx = (self.current_wp_idx + best_offset) % N

    def _velocity_plan(self, curr_x, curr_y, curr_vel, curr_yaw):
        """4-pass velocity planner ported from MP2.

        Returns:
            velocities: list of target velocities for upcoming waypoints.
        """
        N = len(self.raceline)
        horizon = min(N, 400)

        # Build path from current position + future waypoints
        path = [[curr_x, curr_y]]
        for i in range(horizon):
            idx = (self.current_wp_idx + i) % N
            path.append(self.raceline[idx].tolist())

        n = len(path)
        if n < 3:
            return [self.v_max] * max(n, 1)

        # Pass 1: Curvature / grip limit
        v_limit = [self.v_max] * n
        kappa = [0.0] * n
        dist = [0.0] * n

        for i in range(1, n - 1):
            p1 = np.array(path[i - 1])
            p2 = np.array(path[i])
            p3 = np.array(path[i + 1])

            d1 = np.linalg.norm(p2 - p1)
            d2 = np.linalg.norm(p3 - p2)
            dist[i - 1] = d1
            dist[i] = d2

            if i > 1 and d1 > 0.01 and d2 > 0.01:
                yaw1 = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
                yaw2 = np.arctan2(p3[1] - p2[1], p3[0] - p2[0])
                d_yaw = (yaw2 - yaw1 + np.pi) % (2 * np.pi) - np.pi
                curvature = abs(d_yaw) / ((d1 + d2) / 2.0)
                kappa[i] = curvature

                v_curve = np.sqrt(self.a_lat_max / (curvature + 1e-6))
                v_limit[i] = np.clip(v_curve, self.v_min, self.v_max)

        # Pass 2: Backward deceleration (friction ellipse)
        v_bwd = list(v_limit)
        for i in range(n - 2, -1, -1):
            if dist[i] > 0.01:
                a_lat_req = (v_bwd[i + 1] ** 2) * kappa[i + 1]
                a_lat_req = min(a_lat_req, self.a_lat_max)
                a_lon_avail = self.a_lon_max * np.sqrt(
                    max(0.0, 1.0 - (a_lat_req / self.a_lat_max) ** 2)
                )
                v_brake = np.sqrt(v_bwd[i + 1] ** 2 + 2 * a_lon_avail * dist[i])
                v_bwd[i] = min(v_limit[i], v_brake)
            else:
                v_bwd[i] = v_limit[i]

        # Pass 3: Forward acceleration (friction ellipse)
        v_fwd = [0.0] * n
        v_fwd[0] = curr_vel
        for i in range(n - 1):
            if dist[i] > 0.01:
                a_lat_req = (v_fwd[i] ** 2) * kappa[i]
                a_lat_req = min(a_lat_req, self.a_lat_max)
                a_lon_avail = self.a_lon_max * np.sqrt(
                    max(0.0, 1.0 - (a_lat_req / self.a_lat_max) ** 2)
                )
                v_accel = np.sqrt(v_fwd[i] ** 2 + 2 * a_lon_avail * dist[i])
                v_fwd[i + 1] = min(v_bwd[i + 1], v_accel)
            else:
                v_fwd[i + 1] = min(v_bwd[i + 1], v_fwd[i])

        # Pass 4: XTE penalty
        target_vel = float(np.clip(v_fwd[1], self.v_min, self.v_max))

        xte, _, _ = compute_xte(self.raceline, np.array([curr_x, curr_y]))
        xte = abs(xte)
        if xte > self.xte_threshold:
            penalty = max(0.4, 1.0 - self.k_xte * (xte - self.xte_threshold))
            target_vel *= penalty

        target_vel = float(np.clip(target_vel, self.v_min, self.v_max))

        # Return velocity profile for the horizon (skip index 0 which is current pos)
        velocities = [
            float(np.clip(v_fwd[i + 1], self.v_min, self.v_max))
            for i in range(min(horizon, n - 1))
        ]
        # Apply XTE penalty to first velocity
        if velocities:
            velocities[0] = target_vel

        return velocities

    def plan_callback(self):
        if self.current_odom is None or self.raceline is None:
            return

        curr_x, curr_y, curr_yaw, curr_vel = self._extract_state()

        # Advance waypoint tracker
        self._advance_waypoint(curr_x, curr_y)

        # Run velocity planner
        velocities = self._velocity_plan(curr_x, curr_y, curr_vel, curr_yaw)

        # Build Path message
        N = len(self.raceline)
        horizon = min(len(velocities), 100)  # Publish up to 100 waypoints ahead

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        for i in range(horizon):
            idx = (self.current_wp_idx + i) % N
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(self.raceline[idx, 0])
            pose.pose.position.y = float(self.raceline[idx, 1])
            # Encode target velocity in z
            pose.pose.position.z = float(velocities[i]) if i < len(velocities) else self.v_min
            # Set orientation along path tangent
            next_idx = (self.current_wp_idx + i + 1) % N
            dx = self.raceline[next_idx, 0] - self.raceline[idx, 0]
            dy = self.raceline[next_idx, 1] - self.raceline[idx, 1]
            yaw = math.atan2(dy, dx)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)

        self.trajectory_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
