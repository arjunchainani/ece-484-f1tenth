from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share_dir = get_package_share_directory('planning')
    config = os.path.join(package_share_dir, 'config', 'planning_params.yaml')

    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning_node',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([planning_node])
