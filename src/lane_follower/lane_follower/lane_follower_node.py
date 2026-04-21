"""Lane follower — eoh inner loop + bounded fit-based augmentation.

The eoh PID (mean range over ±60° ±5-ray windows, forward-cone clearance
for speed) is the always-on inner safety loop. When `enable_advanced` is
true, a wide-sector quadratic fit per wall produces a small heading
feedforward — but only after the fit has been healthy AND consistent for
`health_window` consecutive scans. Any failure of the fit or its history
zeros the auxiliary, falling back to pure eoh behavior.

Pass 2 will add a curvature-based speed cap, time-constant EMA on the
combined commands, and fitted-curve viz markers.

Original quadfit prototype that motivated the design is at
`lane_follower_node_quadfit.py.bak`.
"""

import math
from collections import deque

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

        # ----- advanced (fit-based) augmentation -----
        self.declare_parameter('enable_advanced', True)

        self.declare_parameter('left_min_deg', 30.0)
        self.declare_parameter('left_max_deg', 120.0)
        self.declare_parameter('right_min_deg', -120.0)
        self.declare_parameter('right_max_deg', -30.0)

        self.declare_parameter('max_range_for_fit', 10.0)
        self.declare_parameter('outlier_mad_k', 2.5)
        self.declare_parameter('min_points_for_fit', 10)
        self.declare_parameter('fit_a_max', 20.0)
        self.declare_parameter('fit_b_max', 10.0)

        self.declare_parameter('health_window', 3)
        self.declare_parameter('b_stability_thresh', 1.0)

        self.declare_parameter('heading_gain', 0.15)
        self.declare_parameter('heading_ff_clamp', 0.12)

        # Pass 2 placeholders — declared so YAML loads cleanly now.
        self.declare_parameter('lateral_accel_limit', 2.0)
        self.declare_parameter('steer_tau', 0.0)
        self.declare_parameter('speed_tau', 0.0)

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

        # ----- advanced snapshot -----
        self.enable_advanced = bool(
            self.get_parameter('enable_advanced').value)
        self.left_min = math.radians(
            self.get_parameter('left_min_deg').value)
        self.left_max = math.radians(
            self.get_parameter('left_max_deg').value)
        self.right_min = math.radians(
            self.get_parameter('right_min_deg').value)
        self.right_max = math.radians(
            self.get_parameter('right_max_deg').value)
        self.max_range_for_fit = float(
            self.get_parameter('max_range_for_fit').value)
        self.outlier_mad_k = float(
            self.get_parameter('outlier_mad_k').value)
        self.min_points_for_fit = int(
            self.get_parameter('min_points_for_fit').value)
        self.fit_a_max = float(self.get_parameter('fit_a_max').value)
        self.fit_b_max = float(self.get_parameter('fit_b_max').value)
        self.health_window = max(
            1, int(self.get_parameter('health_window').value))
        self.b_stability_thresh = float(
            self.get_parameter('b_stability_thresh').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.heading_ff_clamp = float(
            self.get_parameter('heading_ff_clamp').value)
        self.lateral_accel_limit = float(
            self.get_parameter('lateral_accel_limit').value)
        self.steer_tau = float(self.get_parameter('steer_tau').value)
        self.speed_tau = float(self.get_parameter('speed_tau').value)

        # Per-side fit history — entries are (a, b) tuples, or None for an
        # unhealthy frame. Length capped at `health_window`.
        self.left_history = deque(maxlen=self.health_window)
        self.right_history = deque(maxlen=self.health_window)

        # PID state
        self.prev_error = 0.0
        self.integral_error = 0.0

        # EMA / dt state — initialized lazily on the first scan.
        self.smoothed_steer = 0.0
        self.smoothed_speed = 0.0
        self._prev_scan_t = None
        self._ema_initialized = False

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

    # ------------------------------------------------------------------
    # Fit-based augmentation helpers
    # ------------------------------------------------------------------

    def _fit_wall(self, xs, ys):
        """Quadratic fit y = a x^2 + b x + c with MAD outlier reject.

        Returns (a, b, c) or None on insufficient data / degenerate fit.
        """
        if xs.size < self.min_points_for_fit:
            return None
        r = np.hypot(xs, ys)
        med = np.median(r)
        mad = np.median(np.abs(r - med)) + 1e-6
        keep = np.abs(r - med) < self.outlier_mad_k * mad
        xs, ys = xs[keep], ys[keep]
        if xs.size < self.min_points_for_fit:
            return None
        try:
            a, b, c = np.polyfit(xs, ys, 2)
        except (np.linalg.LinAlgError, ValueError):
            return None
        if not (math.isfinite(a) and math.isfinite(b) and math.isfinite(c)):
            return None
        if abs(a) > self.fit_a_max or abs(b) > self.fit_b_max:
            return None
        return float(a), float(b), float(c)

    def _is_healthy(self, history):
        """A side is healthy when the last `health_window` scans all
        produced a valid fit, the sign of `b` is consistent, and the
        spread of `b` is below `b_stability_thresh`."""
        if len(history) < self.health_window:
            return False
        if any(item is None for item in history):
            return False
        bs = np.array([item[1] for item in history], dtype=np.float64)
        if len(np.unique(np.sign(bs[bs != 0.0]))) > 1:
            return False
        if float(np.std(bs)) > self.b_stability_thresh:
            return False
        return True

    def _compute_fit_features(self, msg):
        """Run wide-sector fits, update health histories, return augmentations.

        Returns dict with:
          heading_ff   : bounded steering perturbation (rad)
          v_curv       : curvature-derived speed cap (m/s) or +inf if untrusted
          left_fit     : (a,b,c) or None  — for viz
          right_fit    : (a,b,c) or None  — for viz
          left_pts, right_pts : (xs, ys) arrays of points that fed the fits
          b_l, b_r, kappa, left_ok, right_ok : debug
        """
        result = {
            'heading_ff': 0.0, 'v_curv': float('inf'),
            'left_fit': None, 'right_fit': None,
            'left_pts': (np.empty(0), np.empty(0)),
            'right_pts': (np.empty(0), np.empty(0)),
            'b_l': 0.0, 'b_r': 0.0, 'kappa': 0.0,
            'left_ok': False, 'right_ok': False,
        }
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = ranges.size
        if n == 0:
            return result
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        valid = (np.isfinite(ranges) & (ranges > 0.0)
                 & (ranges < self.max_range_for_fit))
        xs_all = ranges * np.cos(angles)
        ys_all = ranges * np.sin(angles)

        left_mask = valid & (angles >= self.left_min) & (angles <= self.left_max)
        right_mask = (valid & (angles >= self.right_min)
                      & (angles <= self.right_max))

        left_pts = (xs_all[left_mask], ys_all[left_mask])
        right_pts = (xs_all[right_mask], ys_all[right_mask])

        left_fit = self._fit_wall(*left_pts)
        right_fit = self._fit_wall(*right_pts)

        # History uses (a, b) — c is unused for the heading-FF / curvature.
        self.left_history.append(
            (left_fit[0], left_fit[1]) if left_fit is not None else None)
        self.right_history.append(
            (right_fit[0], right_fit[1]) if right_fit is not None else None)

        left_ok = self._is_healthy(self.left_history)
        right_ok = self._is_healthy(self.right_history)
        b_l = left_fit[1] if (left_fit is not None and left_ok) else 0.0
        b_r = right_fit[1] if (right_fit is not None and right_ok) else 0.0

        ff = self.heading_gain * 0.5 * (b_l + b_r)
        ff = max(-self.heading_ff_clamp, min(ff, self.heading_ff_clamp))

        # Curvature speed cap — only when BOTH walls are healthy.
        kappa = 0.0
        v_curv = float('inf')
        if left_ok and right_ok:
            a_avg = 0.5 * (left_fit[0] + right_fit[0])
            b_avg = 0.5 * (b_l + b_r)
            kappa = abs(2.0 * a_avg / (1.0 + b_avg * b_avg) ** 1.5)
            if kappa > 1e-3:
                v_curv = math.sqrt(self.lateral_accel_limit / kappa)

        result.update({
            'heading_ff': float(ff),
            'v_curv': float(v_curv),
            'left_fit': left_fit,
            'right_fit': right_fit,
            'left_pts': left_pts,
            'right_pts': right_pts,
            'b_l': float(b_l), 'b_r': float(b_r),
            'kappa': float(kappa),
            'left_ok': bool(left_ok), 'right_ok': bool(right_ok),
        })
        return result

    # ------------------------------------------------------------------

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

        # 1) eoh inner loop — always runs.
        speed_eoh, steer_eoh, info = self._driving_policy(scan)

        # 2) Bounded fit-based augmentation.
        if self.enable_advanced:
            fit_info = self._compute_fit_features(msg)
        else:
            fit_info = {
                'heading_ff': 0.0, 'v_curv': float('inf'),
                'left_fit': None, 'right_fit': None,
                'left_pts': (np.empty(0), np.empty(0)),
                'right_pts': (np.empty(0), np.empty(0)),
                'b_l': 0.0, 'b_r': 0.0, 'kappa': 0.0,
                'left_ok': False, 'right_ok': False,
            }

        # 3) Combine — bounded perturbation + speed CAP (never raises).
        raw_steer = max(-self.max_steer,
                        min(steer_eoh + fit_info['heading_ff'],
                            self.max_steer))
        raw_speed = min(speed_eoh, fit_info['v_curv'])

        # 4) Time-constant EMA (tau=0 disables, gives raw passthrough).
        now_t = (msg.header.stamp.sec
                 + msg.header.stamp.nanosec * 1e-9)
        if not self._ema_initialized:
            self.smoothed_steer = raw_steer
            self.smoothed_speed = raw_speed
            self._ema_initialized = True
            dt = 0.0
        else:
            dt = max(0.0, now_t - (self._prev_scan_t or now_t))
        self._prev_scan_t = now_t

        if self.steer_tau > 0.0 and dt > 0.0:
            a_s = dt / (self.steer_tau + dt)
            self.smoothed_steer = (a_s * raw_steer
                                   + (1.0 - a_s) * self.smoothed_steer)
        else:
            self.smoothed_steer = raw_steer
        if self.speed_tau > 0.0 and dt > 0.0:
            a_v = dt / (self.speed_tau + dt)
            self.smoothed_speed = (a_v * raw_speed
                                   + (1.0 - a_v) * self.smoothed_speed)
        else:
            self.smoothed_speed = raw_speed

        steer = float(self.smoothed_steer)
        speed = float(self.smoothed_speed)

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
            v_curv_str = ('inf' if not math.isfinite(fit_info['v_curv'])
                          else f"{fit_info['v_curv']:.2f}")
            self.get_logger().info(
                f"d_left={info['d_left']:.2f}  d_right={info['d_right']:.2f}  "
                f"err={info['d_left']-info['d_right']:+.2f}  "
                f"steer={steer:+.3f}  speed={speed:.2f}  "
                f"front={info['front_dist']:.2f}  "
                f"adv={int(self.enable_advanced)} "
                f"L_ok={int(fit_info['left_ok'])} "
                f"R_ok={int(fit_info['right_ok'])} "
                f"b_l={fit_info['b_l']:+.2f} b_r={fit_info['b_r']:+.2f} "
                f"ff={fit_info['heading_ff']:+.3f} "
                f"k={fit_info['kappa']:.2f} v_curv={v_curv_str}"
            )

        if self.publish_viz:
            self._publish_viz(msg, info, fit_info)

    # ------------------------------------------------------------------

    def _publish_viz(self, scan_msg, info, fit_info):
        """Show eoh windows (POINTS) plus, when fits are present, the
        fitted curves (LINE_STRIP) and a centerline midline.

        Marker IDs:
          0 = left window points (green)
          1 = right window points (red)
          2 = left fitted curve  (green LINE_STRIP)
          3 = right fitted curve (red LINE_STRIP)
          4 = centerline midline (blue LINE_STRIP)
        """
        arr = MarkerArray()
        ranges = np.asarray(scan_msg.ranges, dtype=np.float64)
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

        def curve_marker(mid, color, coef, x_lo, x_hi):
            a, b, c = coef
            xs = np.linspace(x_lo, x_hi, 40)
            ys = a * xs * xs + b * xs + c
            m = header()
            m.ns = 'lane_follower'
            m.id = mid
            m.type = Marker.LINE_STRIP
            m.scale.x = 0.03
            m.color = color
            m.points = [Point(x=float(x), y=float(y), z=0.0)
                        for x, y in zip(xs, ys)]
            return m

        def delete_marker(mid):
            m = header()
            m.ns = 'lane_follower'
            m.id = mid
            m.action = Marker.DELETE
            return m

        green = ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0)
        red = ColorRGBA(r=0.95, g=0.2, b=0.2, a=1.0)
        blue = ColorRGBA(r=0.2, g=0.5, b=1.0, a=1.0)

        arr.markers.append(
            points_marker(0, green, info['left_lo'], info['left_hi']))
        arr.markers.append(
            points_marker(1, red, info['right_lo'], info['right_hi']))

        left_fit = fit_info['left_fit']
        right_fit = fit_info['right_fit']
        xs_l, _ = fit_info['left_pts']
        xs_r, _ = fit_info['right_pts']

        if left_fit is not None and xs_l.size > 1:
            arr.markers.append(curve_marker(
                2, green, left_fit, float(xs_l.min()), float(xs_l.max())))
        else:
            arr.markers.append(delete_marker(2))

        if right_fit is not None and xs_r.size > 1:
            arr.markers.append(curve_marker(
                3, red, right_fit, float(xs_r.min()), float(xs_r.max())))
        else:
            arr.markers.append(delete_marker(3))

        if (left_fit is not None and right_fit is not None
                and xs_l.size > 1 and xs_r.size > 1):
            x_lo = max(float(xs_l.min()), float(xs_r.min()))
            x_hi = min(float(xs_l.max()), float(xs_r.max()))
            if x_hi > x_lo:
                xs = np.linspace(x_lo, x_hi, 40)
                al, bl, cl = left_fit
                ar, br, cr = right_fit
                yc = 0.5 * ((al * xs * xs + bl * xs + cl)
                            + (ar * xs * xs + br * xs + cr))
                m = header()
                m.ns = 'lane_follower'
                m.id = 4
                m.type = Marker.LINE_STRIP
                m.scale.x = 0.04
                m.color = blue
                m.points = [Point(x=float(x), y=float(y), z=0.0)
                            for x, y in zip(xs, yc)]
                arr.markers.append(m)
            else:
                arr.markers.append(delete_marker(4))
        else:
            arr.markers.append(delete_marker(4))

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
