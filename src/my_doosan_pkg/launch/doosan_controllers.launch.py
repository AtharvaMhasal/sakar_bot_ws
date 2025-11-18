from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    # START ros2_control controllers only
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_jtc = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        load_jsb,
        load_jtc
    ])
