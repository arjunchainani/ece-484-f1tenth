import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')

        # Latest state
        self.current_odom = None
        self.current_scan = None

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/ego_racecar/scan',
            self.scan_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)

        # Publisher
        self.trajectory_pub = self.create_publisher(
            Path,
            '/planning/trajectory',
            10)

        # Planning timer at 20Hz
        self.plan_timer = self.create_timer(0.05, self.plan_callback)

        self.get_logger().info('Planning node initialized')

    def scan_callback(self, msg):
        self.current_scan = msg

    def odom_callback(self, msg):
        self.current_odom = msg

    def plan_callback(self):
        if self.current_odom is None:
            return

        # TODO: implement actual path planning algorithm
        # For now, publish a simple straight-line path ahead of the car
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        x = self.current_odom.pose.pose.position.x
        y = self.current_odom.pose.pose.position.y

        # Extract yaw from quaternion
        q = self.current_odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Generate waypoints 0.5m apart, 5m ahead
        for i in range(10):
            pose = PoseStamped()
            pose.header = path.header
            dist = 0.5 * (i + 1)
            pose.pose.position.x = x + dist * math.cos(yaw)
            pose.pose.position.y = y + dist * math.sin(yaw)
            pose.pose.orientation = q
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
