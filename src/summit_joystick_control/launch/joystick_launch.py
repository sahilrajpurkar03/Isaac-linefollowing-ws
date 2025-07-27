from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'deadzone': 0.2}],
            output='screen'
        ),
        Node(
            package='summit_joystick_control',
            executable='joystick_controller',
            name='joystick_controller',
            output='screen'
        )
    ])
