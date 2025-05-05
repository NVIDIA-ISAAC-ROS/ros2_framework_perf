from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description for EmitterNode."""
    container = ComposableNodeContainer(
        name='emitter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ros2_framework_perf',
                plugin='ros2_framework_perf::EmitterNode',
                name='emitter_node'
            )
        ],
        output='screen',
        arguments=['--ros-args', '--executor', 'single-threaded']
    )

    return LaunchDescription([
        container
    ])
