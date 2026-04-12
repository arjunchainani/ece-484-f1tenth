import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped


class ControlsNode(Node):
    def __init__(self):
        super().__init__('controls_node')

        # Latest state
        self.current_odom = None
        self.current_trajectory = None

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        self.trajectory_sub = self.create_subscription(
            Path,
            '/planning/trajectory',
            self.trajectory_callback,
            10)

        # Publisher
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/ego_racecar/drive',
            10)

        # Timer-based control loop at 50Hz
        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('Controls node initialized')

    def odom_callback(self, msg):
        self.current_odom = msg

    def trajectory_callback(self, msg):
        self.current_trajectory = msg

    def control_loop(self):
        if self.current_odom is None:
            return

        drive_msg = AckermannDriveStamped()

        if self.current_trajectory is None or len(self.current_trajectory.poses) == 0:
            # No trajectory received yet -- stop
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
        else:
            # TODO: implement trajectory tracking controller
            # For now, drive straight slowly to verify the pipeline works
            drive_msg.drive.speed = 1.0
            drive_msg.drive.steering_angle = 0.0

        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
