from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share_dir = get_package_share_directory('planning')

    controls_config = os.path.join(
        package_share_dir,
        'config',
        'controls_params.yaml'
    )

    controls_node = Node(
        package='planning',
        executable='controls_node',
        name='controls_node',
        parameters=[controls_config],
        remappings=[
            ('/ego_racecar/scan', '/scan'),
            ('/ego_racecar/drive', '/ackermann_cmd'),
        ],
        output='screen',
    )

    return LaunchDescription([
        controls_node,
    ])
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class PlanningNode(Node):
    """Deprecated compatibility node.

    The real-car autonomy stack is scan-reactive and does not use odometry,
    map raceline generation, or /planning/trajectory anymore. This node is
    intentionally inert so old launch/run commands do not trigger map-based
    raceline generation or block waiting for /odom.
    """

    def __init__(self):
        super().__init__('planning_node')
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/ego_racecar/scan',
            self.scan_callback,
            10,
        )
        self.get_logger().info(
            'Planning node disabled: using scan-reactive controls_node only.'
        )

    def scan_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import os
import math
import copy
import threading
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy, LaserScan


class ControlsNode(Node):
    def __init__(self):
        super().__init__('controls_node')

        # Declare parameters
        self.declare_parameter('max_speed', 8.0)
        self.declare_parameter('max_steer', 0.418879)  # ~24 degrees in radians

        # Read parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steer = self.get_parameter('max_steer').value

        # Latest state
        self.current_scan = None
        self.enabled = False

        # EOHDemo LiDAR fallback PID state
        self.prev_error = 0.0
        self.integral_error = 0.0

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/ego_racecar/scan',
            self.scan_callback,
            10,
        )
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10,
        )

        # Publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/ego_racecar/drive', 10)

        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)

    def scan_callback(self, msg):
        self.current_scan = msg

    def joy_callback(self, msg):
        # Enable only while Y button is held
        if len(msg.buttons) > 3:
            self.enabled = (msg.buttons[3] == 1)
            if not self.enabled:
                self.integral_error = 0.0

    def _eoh_lidar_fallback(self, scan_msg):
        """LiDAR-only EOHDemo fallback used when quadratic wall fit is unavailable."""
        rays = np.array(scan_msg.ranges)
        num_rays = len(rays)
        if num_rays < 2:
            return 0.0, 0.0

        min_angle = float(scan_msg.angle_min)
        max_angle = float(scan_msg.angle_max)
        angle_inc = (max_angle - min_angle) / (num_rays - 1)

        def get_index(target_angle):
            idx = int((target_angle - min_angle) / angle_inc)
            return max(0, min(idx, num_rays - 1))

        front_cone = rays[get_index(np.radians(-10)):get_index(np.radians(10)) + 1]
        valid_front = front_cone[np.isfinite(front_cone) & (front_cone > 0.0)]

        front_dist = 0.0
        if len(valid_front) > 0:
            front_dist = np.min(valid_front)

        speed = max(0.0, min(front_dist * 2.0, self.max_speed))

        window = 5
        idx_left = get_index(np.radians(60))
        idx_right = get_index(-np.radians(60))

        left_window = rays[max(0, idx_left - window):min(num_rays, idx_left + window + 1)]
        right_window = rays[max(0, idx_right - window):min(num_rays, idx_right + window + 1)]

        valid_left = left_window[np.isfinite(left_window) & (left_window > 0.0)]
        valid_right = right_window[np.isfinite(right_window) & (right_window > 0.0)]

        dist_left = np.mean(valid_left) if len(valid_left) > 0 else 2.0
        dist_right = np.mean(valid_right) if len(valid_right) > 0 else 2.0

        error = dist_left - dist_right

        kp_steer = 0.6
        ki_steer = 0.005
        kd_steer = 0.2

        p_term = kp_steer * error
        self.integral_error += error
        self.integral_error = max(-20.0, min(self.integral_error, 20.0))
        i_term = ki_steer * self.integral_error
        derivative = error - self.prev_error
        d_term = kd_steer * derivative
        self.prev_error = error

        steering = p_term + i_term + d_term
        steering = np.clip(steering, -self.max_steer, self.max_steer)

        return float(steering), float(speed)

    def control_loop(self):
        drive_msg = AckermannDriveStamped()

        if not self.enabled:
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            return

        if self.current_scan is None:
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            return

        # Prefer the quadratic wall-fit policy if this file defines one.
        # Otherwise use the EOHDemo LiDAR-only fallback.
        if hasattr(self, '_quadratic_wall_fit_policy'):
            steering, speed = self._quadratic_wall_fit_policy(self.current_scan)
        elif hasattr(self, 'quadratic_wall_fit_policy'):
            steering, speed = self.quadratic_wall_fit_policy(self.current_scan)
        else:
            steering, speed = self._eoh_lidar_fallback(self.current_scan)

        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering
        self.drive_pub.publish(drive_msg)
