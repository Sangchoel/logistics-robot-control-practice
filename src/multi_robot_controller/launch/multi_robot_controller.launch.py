from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = {
        'robot_namespaces': ['tb1', 'tb2', 'tb3', 'tb4'],
        'allocation_policy': 'eta',
        'nominal_speed_mps': 0.25,
        'stuck_timeout_sec': 20.0,
        'goal_timeout_sec': 180.0,
        'coordination_radius_m': 0.8,
        'tasks_yaml_path': '',   # set absolute path if you want to preload tasks
    }

    return LaunchDescription([
        Node(
            package='multi_robot_controller',
            executable='multi_robot_controller',
            name='multi_robot_controller',
            output='screen',
            parameters=[params],
        ),
    ])

