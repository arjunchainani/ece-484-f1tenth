from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share_dir = get_package_share_directory('lane_follower')
    config = os.path.join(package_share_dir, 'config', 'lane_follower_params.yaml')

    lane_follower_node = Node(
        package='lane_follower',
        executable='lane_follower_node',
        name='lane_follower_node',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([lane_follower_node])
