import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped

from .utils import quaternion_to_yaw, normalize_angle


class ControlsNode(Node):
    def __init__(self):
        super().__init__('controls_node')

        # Declare parameters
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('lookahead_gain', 0.5)
        self.declare_parameter('min_lookahead', 0.5)
        self.declare_parameter('max_lookahead', 3.0)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('max_speed', 10.0)
        self.declare_parameter('control_frequency', 50.0)

        # Read parameters
        self.L = self.get_parameter('wheelbase').value
        self.k_ld = self.get_parameter('lookahead_gain').value
        self.min_ld = self.get_parameter('min_lookahead').value
        self.max_ld = self.get_parameter('max_lookahead').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.max_speed = self.get_parameter('max_speed').value

        # Latest state
        self.current_odom = None
        self.current_trajectory = None

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.trajectory_sub = self.create_subscription(
            Path, '/planning/trajectory', self.trajectory_callback, 10)

        # Publisher
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/ego_racecar/drive', 10)

        # Timer-based control loop
        freq = self.get_parameter('control_frequency').value
        self.control_timer = self.create_timer(1.0 / freq, self.control_loop)

        self.get_logger().info('Controls node initialized')

    def odom_callback(self, msg):
        self.current_odom = msg

    def trajectory_callback(self, msg):
        self.current_trajectory = msg

    def _extract_state(self):
        """Extract (x, y, yaw, velocity) from odometry."""
        odom = self.current_odom
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        yaw = quaternion_to_yaw(odom.pose.pose.orientation)
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        vel = math.hypot(vx, vy)
        return x, y, yaw, vel

    def _find_closest_path_index(self, x, y, poses):
        """Find index of the closest pose in the trajectory."""
        min_dist = float('inf')
        best_idx = 0
        for i, pose in enumerate(poses):
            dx = pose.pose.position.x - x
            dy = pose.pose.position.y - y
            d = math.hypot(dx, dy)
            if d < min_dist:
                min_dist = d
                best_idx = i
        return best_idx

    def _pure_pursuit(self, curr_x, curr_y, curr_yaw, curr_vel, poses):
        """Pure pursuit lateral controller with dynamic lookahead.

        Ported from MP2 controller.py:179-234, adapted for F1Tenth.

        Returns:
            (steering_angle, target_speed)
        """
        if not poses:
            return 0.0, 0.0

        # Dynamic lookahead distance
        ld_des = np.clip(self.k_ld * curr_vel + self.min_ld, self.min_ld, self.max_ld)

        # Find lookahead point via circle-line intersection
        curr_pos = np.array([curr_x, curr_y])
        lookahead_point = np.array([
            poses[-1].pose.position.x, poses[-1].pose.position.y
        ])
        lookahead_vel = poses[-1].pose.position.z

        for i in range(len(poses) - 1):
            p1 = np.array([poses[i].pose.position.x, poses[i].pose.position.y])
            p2 = np.array([poses[i + 1].pose.position.x, poses[i + 1].pose.position.y])

            d = p2 - p1
            f = p1 - curr_pos

            a = np.dot(d, d)
            b_coeff = 2.0 * np.dot(f, d)
            c = np.dot(f, f) - ld_des ** 2

            discriminant = b_coeff ** 2 - 4.0 * a * c
            if discriminant >= 0 and a > 1e-8:
                sqrt_disc = np.sqrt(discriminant)
                t2 = (-b_coeff + sqrt_disc) / (2.0 * a)

                if 0.0 <= t2 <= 1.0:
                    lookahead_point = p1 + t2 * d
                    # Interpolate velocity
                    v1 = poses[i].pose.position.z
                    v2 = poses[i + 1].pose.position.z
                    lookahead_vel = v1 + t2 * (v2 - v1)
                    break

        # Compute steering angle
        dx = lookahead_point[0] - curr_x
        dy = lookahead_point[1] - curr_y
        ld = math.hypot(dx, dy)
        if ld < 0.1:
            ld = 0.1

        target_yaw = math.atan2(dy, dx)
        alpha = normalize_angle(target_yaw - curr_yaw)

        steering = math.atan2(2.0 * self.L * math.sin(alpha), ld)
        steering = np.clip(steering, -self.max_steer, self.max_steer)

        target_speed = np.clip(lookahead_vel, 0.0, self.max_speed)

        return float(steering), float(target_speed)

    def control_loop(self):
        if self.current_odom is None:
            return

        drive_msg = AckermannDriveStamped()

        if self.current_trajectory is None or len(self.current_trajectory.poses) == 0:
            # No trajectory — stop
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
        else:
            curr_x, curr_y, curr_yaw, curr_vel = self._extract_state()
            poses = self.current_trajectory.poses

            steering, speed = self._pure_pursuit(
                curr_x, curr_y, curr_yaw, curr_vel, poses
            )

            drive_msg.drive.speed = speed
            drive_msg.drive.steering_angle = steering

        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
