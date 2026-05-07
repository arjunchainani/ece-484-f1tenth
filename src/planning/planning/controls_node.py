
import math

import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy, LaserScan


class ControlsNode(Node):
    def __init__(self):
        super().__init__('controls_node')

        # Declare parameters
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('quadratic_lookahead', 1.2)
        self.declare_parameter('quadratic_residual_threshold', 0.25)

        # Read parameters
        self.L = float(self.get_parameter('wheelbase').value)
        self.max_steer = float(self.get_parameter('max_steering_angle').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.quad_lookahead = float(self.get_parameter('quadratic_lookahead').value)
        self.quad_residual_threshold = float(
            self.get_parameter('quadratic_residual_threshold').value
        )

        # Latest state. No odom and no trajectory are required.
        self.current_scan = None
        self.enabled = False

        # EOHDemo fallback PID state
        self.prev_error = 0.0
        self.integral_error = 0.0

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
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
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/ackermann_cmd',
            10,
        )

        # Timer-based control loop
        freq = float(self.get_parameter('control_frequency').value)
        self.control_timer = self.create_timer(1.0 / freq, self.control_loop)

        self.get_logger().info('Controls node initialized in scan-only mode')

    def scan_callback(self, msg):
        self.current_scan = msg

    def joy_callback(self, msg):
        # Enable only while Y button is held
        if len(msg.buttons) > 3:
            self.enabled = (msg.buttons[3] == 1)
            if not self.enabled:
                self.integral_error = 0.0

    def _get_index(self, scan_msg, target_angle):
        num_rays = len(scan_msg.ranges)
        if num_rays < 2:
            return 0

        angle_inc = (scan_msg.angle_max - scan_msg.angle_min) / (num_rays - 1)
        idx = int((target_angle - scan_msg.angle_min) / angle_inc)
        return max(0, min(idx, num_rays - 1))

    def _front_distance_speed(self, scan_msg):
        rays = np.array(scan_msg.ranges)
        idx_left = self._get_index(scan_msg, np.radians(10))
        idx_right = self._get_index(scan_msg, np.radians(-10))
        lo = min(idx_left, idx_right)
        hi = max(idx_left, idx_right)

        front_cone = rays[lo:hi + 1]
        valid_front = front_cone[np.isfinite(front_cone) & (front_cone > 0.0)]

        if len(valid_front) == 0:
            return 0.0

        front_dist = float(np.min(valid_front))
        return max(0.0, min(front_dist * 2.0, self.max_speed))

    def _quadratic_wall_fit_policy(self, scan_msg):
        """Fit left/right walls in LiDAR frame and steer toward their centerline.

        Falls back to EOHDemo when there are not enough points or the quadratic
        fit residual is poor.
        """
        rays = np.array(scan_msg.ranges)
        num_rays = len(rays)
        if num_rays < 10:
            return self._eoh_lidar_fallback(scan_msg)

        angles = scan_msg.angle_min + np.arange(num_rays) * scan_msg.angle_increment
        valid = (
            np.isfinite(rays)
            & (rays > scan_msg.range_min)
            & (rays < scan_msg.range_max)
        )

        x = rays[valid] * np.cos(angles[valid])
        y = rays[valid] * np.sin(angles[valid])

        # Only use points in front of the car.
        forward = (x > 0.2) & (x < 4.0)
        x = x[forward]
        y = y[forward]

        left = y > 0.15
        right = y < -0.15

        if np.count_nonzero(left) < 8 or np.count_nonzero(right) < 8:
            return self._eoh_lidar_fallback(scan_msg)

        try:
            left_fit = np.polyfit(x[left], y[left], 2)
            right_fit = np.polyfit(x[right], y[right], 2)
        except Exception:
            return self._eoh_lidar_fallback(scan_msg)

        left_pred = np.polyval(left_fit, x[left])
        right_pred = np.polyval(right_fit, x[right])
        left_rmse = float(np.sqrt(np.mean((left_pred - y[left]) ** 2)))
        right_rmse = float(np.sqrt(np.mean((right_pred - y[right]) ** 2)))

        if max(left_rmse, right_rmse) > self.quad_residual_threshold:
            return self._eoh_lidar_fallback(scan_msg)

        lookahead_x = self.quad_lookahead
        left_y = float(np.polyval(left_fit, lookahead_x))
        right_y = float(np.polyval(right_fit, lookahead_x))
        center_y = 0.5 * (left_y + right_y)

        left_slope = 2.0 * left_fit[0] * lookahead_x + left_fit[1]
        right_slope = 2.0 * right_fit[0] * lookahead_x + right_fit[1]
        center_slope = 0.5 * (left_slope + right_slope)

        target_yaw = math.atan2(center_y, lookahead_x)
        path_yaw = math.atan(center_slope)
        alpha = 0.7 * target_yaw + 0.3 * path_yaw
        ld = max(math.hypot(lookahead_x, center_y), 0.1)

        steering = math.atan2(2.0 * self.L * math.sin(alpha), ld)
        steering = float(np.clip(steering, -self.max_steer, self.max_steer))

        speed = self._front_distance_speed(scan_msg)
        return steering, speed

    def _eoh_lidar_fallback(self, scan_msg):
        """LiDAR-only EOHDemo fallback."""
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

        speed = self._front_distance_speed(scan_msg)

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
        steering = float(np.clip(steering, -self.max_steer, self.max_steer))

        return steering, speed

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

        steering, speed = self._quadratic_wall_fit_policy(self.current_scan)
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = AckermannDriveStamped()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        node.drive_pub.publish(stop_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
