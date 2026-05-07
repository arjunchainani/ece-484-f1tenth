from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share_dir = get_package_share_directory('planning')

    planning_config = os.path.join(
        package_share_dir,
        'config',
        'planning_params.yaml'
    )

    control_config = os.path.join(
        package_share_dir,
        'config',
        'controls_params.yaml'
    )

    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning_node',
        parameters=[planning_config],
        remappings=[
            ('/ego_racecar/odom', '/odom'),
            ('/ego_racecar/scan', '/scan'),
        ],
        output='screen',
    )

    controls_node = Node(
        package='planning',
        executable='controls_node',
        name='controls_node',
        parameters=[control_config],
        remappings=[
            ('/ego_racecar/odom', '/odom'),
            ('/ego_racecar/drive', '/ackermann_cmd'),
        ],
        output='screen',
    )

    return LaunchDescription([
        planning_node,
        controls_node,
    ])