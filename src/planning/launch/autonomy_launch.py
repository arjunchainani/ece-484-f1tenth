from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controls_node = Node(
        package='planning',
        executable='controls_node',
        name='controls_node',
        output='screen',
    )

    return LaunchDescription([
        controls_node,
    ])