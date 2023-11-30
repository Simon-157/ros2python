import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='topics_quiz',
            executable='topics_quiz_node',
            name='topics_quiz_node',
            output='screen'
        )
    ])


