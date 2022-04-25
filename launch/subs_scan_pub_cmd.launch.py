import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='project2_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='project_part2',
                    plugin='project_part2::SubsScanPubCmd',
                    name='subs_scan_pub_cmd_component'),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])