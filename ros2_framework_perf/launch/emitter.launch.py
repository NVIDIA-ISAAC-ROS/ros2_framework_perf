from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import EmitEvent
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    """Generate launch description for EmitterNode."""
    # Create EmitterNode
    emitter_node = ComposableNode(
        package='ros2_framework_perf',
        plugin='ros2_framework_perf::EmitterNode',
        name='emitter_node',
        parameters=[{
            'node_name': 'CameraImager01'
        }]
    )

    # Create container with EmitterNode
    container = ComposableNodeContainer(
        name='emitter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[emitter_node],
        output='screen'
    )

    return LaunchDescription([
        container
    ])
