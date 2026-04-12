from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share_dir = get_package_share_directory('controls')
    config = os.path.join(package_share_dir, 'config', 'controls_params.yaml')

    controls_node = Node(
        package='controls',
        executable='controls_node',
        name='controls_node',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([controls_node])
