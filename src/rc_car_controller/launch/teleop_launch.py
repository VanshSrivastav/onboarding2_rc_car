from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1️⃣ Built-in joystick node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0
            }]
        ),

        # 2️⃣ Your custom joystick receiver node
        Node(
            package='rc_car_controller',
            executable='joystick_receiver_node',
            name='joystick_receiver_node',
            output='screen'
        ),

        # 3️⃣ Your drive control/subscriber node
        Node(
            package='rc_car_controller',
            executable='subscriber_node',
            name='subscriber_node',
            output='screen'
        )
    ])
