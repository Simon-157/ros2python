import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='services_quiz',
            executable='services_quiz_server',
            name='services_quiz_server',
            output='screen',
        )
    ])
