from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project_part2',
            executable='pre_approach',
            output='screen',
            parameters=[ {"is_gazebo": True} ]
        ),
    ])