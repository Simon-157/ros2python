import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='services_quiz',
            executable='services_quiz_client',
            name='services_quiz_client',
            output='screen',
        )
    ])
