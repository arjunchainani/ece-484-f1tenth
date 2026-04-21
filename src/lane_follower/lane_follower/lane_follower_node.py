"""Lane follower — eoh port.

Direct port of `eoh/main.py` (proven on the real vehicle): mean range over
small windows at ±60° gives left/right wall distances, a PID on the
difference produces steering, and a forward cone gives a clearance-based
speed. Topics are remapped for the sim.

The previous quadratic-fit implementation is preserved at
`lane_follower_node_quadfit.py.bak` next to this file.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class LaneFollowerNode(Node):
    def __init__(self):
        super().__init__('lane_follower_node')

        # Topics
        self.declare_parameter('scan_topic', '/ego_racecar/scan')
        self.declare_parameter('drive_topic', '/ego_racecar/drive')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('viz_topic', '/lane_follower/viz')
        self.declare_parameter('viz_frame', 'ego_racecar/laser')
        self.declare_parameter('publish_viz', True)

        # Enable / joy gate (eoh used Y button on the real car).
        self.declare_parameter('enable_on_startup', True)
        self.declare_parameter('joy_enable_button', 3)

        # eoh window angles (degrees, vehicle frame). +60 = forward-left.
        self.declare_parameter('left_angle_deg', 60.0)
        self.declare_parameter('right_angle_deg', -60.0)
        self.declare_parameter('window_half_size', 5)
        self.declare_parameter('forward_cone_half_deg', 10.0)

        # PID — exact eoh values.
        self.declare_parameter('kp', 0.6)
        self.declare_parameter('ki', 0.005)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('integral_clamp', 20.0)

        # Output limits — eoh values.
        self.declare_parameter('max_steering_angle', 0.4)
        self.declare_parameter('speed_gain', 2.0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('default_wall_dist', 2.0)

        # Snapshot
        self.scan_topic = self.get_parameter('scan_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.joy_topic = self.get_parameter('joy_topic').value
        self.viz_topic = self.get_parameter('viz_topic').value
        self.viz_frame = self.get_parameter('viz_frame').value
        self.publish_viz = bool(self.get_parameter('publish_viz').value)

        self.enabled = bool(self.get_parameter('enable_on_startup').value)
        self.joy_enable_button = int(
            self.get_parameter('joy_enable_button').value)

        self.left_angle = math.radians(
            self.get_parameter('left_angle_deg').value)
        self.right_angle = math.radians(
            self.get_parameter('right_angle_deg').value)
        self.window = int(self.get_parameter('window_half_size').value)
        self.forward_cone_half = math.radians(
            self.get_parameter('forward_cone_half_deg').value)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.integral_clamp = float(self.get_parameter('integral_clamp').value)

        self.max_steer = float(self.get_parameter('max_steering_angle').value)
        self.speed_gain = float(self.get_parameter('speed_gain').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.default_wall_dist = float(
            self.get_parameter('default_wall_dist').value)

        # PID state
        self.prev_error = 0.0
        self.integral_error = 0.0

        # Diagnostics
        self._scan_count = 0
        self._published_once = False

        # I/O
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10)
        self.joy_sub = self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10)
        self.viz_pub = self.create_publisher(
            MarkerArray, self.viz_topic, 10)

        self.get_logger().info(
            f'lane_follower_node (eoh port) up — enabled={self.enabled}, '
            f'scan={self.scan_topic}, drive={self.drive_topic}'
        )

    # ------------------------------------------------------------------

    def joy_callback(self, msg):
        if len(msg.buttons) > self.joy_enable_button:
            new_state = (msg.buttons[self.joy_enable_button] == 1)
            if new_state and not self.enabled:
                self.get_logger().info('lane_follower enabled via joy')
            elif not new_state and self.enabled:
                self.get_logger().info('lane_follower disabled via joy')
                self.integral_error = 0.0
            self.enabled = new_state

    # ------------------------------------------------------------------

    def _driving_policy(self, scan):
        """Direct port of eoh `driving_policy`."""
        rays = scan['rays']
        num_rays = len(rays)
        min_angle = scan['min_angle']
        max_angle = scan['max_angle']
        angle_inc = (max_angle - min_angle) / (num_rays - 1)

        def get_index(target_angle):
            idx = int((target_angle - min_angle) / angle_inc)
            return max(0, min(idx, num_rays - 1))

        # --- SPEED CONTROL ---
        front_lo = get_index(-self.forward_cone_half)
        front_hi = get_index(self.forward_cone_half) + 1
        front_cone = rays[front_lo:front_hi]
        valid_front = front_cone[np.isfinite(front_cone) & (front_cone > 0.0)]
        front_dist = float(np.min(valid_front)) if valid_front.size > 0 else 0.0
        speed = max(0.0, min(front_dist * self.speed_gain, self.max_speed))

        # --- STEERING CONTROL ---
        idx_left = get_index(self.left_angle)
        idx_right = get_index(self.right_angle)

        left_lo = max(0, idx_left - self.window)
        left_hi = min(num_rays, idx_left + self.window + 1)
        right_lo = max(0, idx_right - self.window)
        right_hi = min(num_rays, idx_right + self.window + 1)

        left_window = rays[left_lo:left_hi]
        right_window = rays[right_lo:right_hi]

        valid_left = left_window[np.isfinite(left_window) & (left_window > 0.0)]
        valid_right = right_window[
            np.isfinite(right_window) & (right_window > 0.0)]

        d_left = float(np.mean(valid_left)) if valid_left.size > 0 \
            else self.default_wall_dist
        d_right = float(np.mean(valid_right)) if valid_right.size > 0 \
            else self.default_wall_dist

        error = d_left - d_right

        # PID
        p_term = self.kp * error
        self.integral_error += error
        self.integral_error = max(
            -self.integral_clamp, min(self.integral_error, self.integral_clamp))
        i_term = self.ki * self.integral_error
        d_term = self.kd * (error - self.prev_error)
        self.prev_error = error

        steer = p_term + i_term + d_term
        steer = max(-self.max_steer, min(steer, self.max_steer))

        return float(speed), float(steer), {
            'd_left': d_left, 'd_right': d_right, 'front_dist': front_dist,
            'left_lo': left_lo, 'left_hi': left_hi,
            'right_lo': right_lo, 'right_hi': right_hi,
        }

    def scan_callback(self, msg):
        self._scan_count += 1
        if self._scan_count == 1:
            self.get_logger().info(
                f'first scan: n={len(msg.ranges)}, '
                f'angle_min={msg.angle_min:.3f}, angle_max={msg.angle_max:.3f}'
            )
        if not self.enabled:
            return

        scan = {
            'min_angle': float(msg.angle_min),
            'max_angle': float(msg.angle_max),
            'rays': np.asarray(msg.ranges, dtype=np.float64),
        }

        speed, steer, info = self._driving_policy(scan)

        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.header.frame_id = 'base_link'
        drive.drive.speed = speed
        drive.drive.steering_angle = steer
        self.drive_pub.publish(drive)

        if not self._published_once:
            self._published_once = True
            self.get_logger().info(
                f'first drive published: speed={speed:.2f}, steer={steer:.3f}'
            )
        if self._scan_count % 80 == 0:
            self.get_logger().info(
                f"d_left={info['d_left']:.2f}  d_right={info['d_right']:.2f}  "
                f"err={info['d_left']-info['d_right']:+.2f}  "
                f"steer={steer:+.3f}  speed={speed:.2f}  "
                f"front={info['front_dist']:.2f}"
            )

        if self.publish_viz:
            self._publish_viz(msg, info)

    # ------------------------------------------------------------------

    def _publish_viz(self, scan_msg, info):
        """Show the left/right window points, the forward cone closest hit,
        and a translucent line along each side at the measured wall
        distance."""
        arr = MarkerArray()
        ranges = np.asarray(scan_msg.ranges, dtype=np.float64)
        n = ranges.size
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment

        def header():
            m = Marker()
            m.header.frame_id = self.viz_frame
            m.header.stamp = scan_msg.header.stamp
            m.action = Marker.ADD
            return m

        def points_marker(mid, color, lo, hi):
            m = header()
            m.ns = 'lane_follower'
            m.id = mid
            m.type = Marker.POINTS
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.color = color
            pts = []
            for i in range(lo, hi):
                r = ranges[i]
                if not (np.isfinite(r) and r > 0.0):
                    continue
                a = angle_min + i * angle_inc
                pts.append(Point(x=float(r * math.cos(a)),
                                 y=float(r * math.sin(a)), z=0.0))
            m.points = pts
            return m

        green = ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0)
        red = ColorRGBA(r=0.95, g=0.2, b=0.2, a=1.0)

        arr.markers.append(points_marker(0, green, info['left_lo'], info['left_hi']))
        arr.markers.append(points_marker(1, red, info['right_lo'], info['right_hi']))

        self.viz_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = AckermannDriveStamped()
        stop.drive.speed = 0.0
        stop.drive.steering_angle = 0.0
        node.drive_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
