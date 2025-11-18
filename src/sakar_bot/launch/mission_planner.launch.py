from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='sakar_bot',
            executable='mission_planner',
            output='screen'
        )
    ])

