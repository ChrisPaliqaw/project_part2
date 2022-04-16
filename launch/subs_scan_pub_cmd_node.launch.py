from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='project_part2', executable='subs_scan_pub_cmd_node', output='screen'),
    ])