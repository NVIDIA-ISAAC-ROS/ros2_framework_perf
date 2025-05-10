import os
import time
import unittest

import launch
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import pytest
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from launch_testing.actions import ReadyToTest
from ros2_framework_perf_interfaces.msg import MessageWithPayload
from ros2_framework_perf_interfaces.srv import GetPublishedMessages


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description for testing EmitterNode."""

    # Create EmitterNode
    camera_node_composable = ComposableNode(
        package='ros2_framework_perf',
        plugin='ros2_framework_perf::EmitterNode',
        name='camera_node',
        parameters=[{
            'node_name': 'camera_node',
            'yaml_config': '''
timer_groups:
  - name: "camera"
    frequency: 10.0
publishers:
  - topic_name: "/camera_image_left"
    message_size: 1024
    trigger:
      type: "timer"
      timer_group: "camera"
  - topic_name: "/camera_info_left"
    message_size: 1024
    trigger:
      type: "timer"
      timer_group: "camera"
'''
        }]
    )
    print("Camera node configuration:")
    print(camera_node_composable.parameters[0].get('yaml_config', ''))

    rectify_node_composable = ComposableNode(
        package='ros2_framework_perf',
        plugin='ros2_framework_perf::EmitterNode',
        name='rectify_node',
        parameters=[{
            'node_name': 'rectify_node',
            'yaml_config': '''
publishers:
  - topic_name: "/camera_image_left_rectified"
    message_size: 2048
    trigger:
      type: "message_received"
      mode: "exact_time"
      topics:
        - "/camera_image_left"
        - "/camera_info_left"
'''
        }]
    )

    tensor_encoder_node_composable = ComposableNode(
        package='ros2_framework_perf',
        plugin='ros2_framework_perf::EmitterNode',
        name='tensor_encoder_node',
        parameters=[{
            'node_name': 'tensor_encoder_node',
            'yaml_config': '''
publishers:
  - topic_name: "/tensor_encoder_output"
    message_size: 1024
    trigger:
      type: "timer"
      frequency: 5.0
      subscription_topics:
        - topic_name: "/camera_image_left_rectified"
          mode: "window"
          window_time: 0.25
'''
        }]
    )

    tensor_inference_node_composable = ComposableNode(
        package='ros2_framework_perf',
        plugin='ros2_framework_perf::EmitterNode',
        name='tensor_inference_node',
        parameters=[{
            'node_name': 'tensor_inference_node',
            'yaml_config': '''
publishers:
  - topic_name: "/tensor_inference_output"
    message_size: 2048
    trigger:
      type: "message_received"
      mode: "exact_time"
      topics:
        - "/tensor_encoder_output"
'''
        }]
    )

    tensor_decode_node_composable = ComposableNode(
        package='ros2_framework_perf',
        plugin='ros2_framework_perf::EmitterNode',
        name='tensor_decode_node',
        parameters=[{
            'node_name': 'tensor_decode_node',
            'yaml_config': '''
publishers:
  - topic_name: "/robot_cmd"
    message_size: 512
    trigger:
      type: "message_received"
      mode: "exact_time"
      topics:
        - "/tensor_inference_output"
'''
        }]
    )

    actuator_node_composable = ComposableNode(
        package='ros2_framework_perf',
        plugin='ros2_framework_perf::EmitterNode',
        name='actuator_node',
        parameters=[{
            'node_name': 'actuator_node',
            'yaml_config': '''
subscriptions:
  - topic_name: "/robot_cmd"
'''
        }]
    )

    # Create container with EmitterNode
    container = ComposableNodeContainer(
        name='emitter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            camera_node_composable,
            rectify_node_composable,
            tensor_encoder_node_composable,
            tensor_inference_node_composable,
            tensor_decode_node_composable,
            actuator_node_composable
        ],
        output='screen'
    )

    return LaunchDescription([
        container,
        ReadyToTest()
    ])


class LifecycleTestNode(Node):
    def __init__(self, node_names):
        super().__init__('lifecycle_test_node')
        self.node_names = node_names
        self.lifecycle_clients = {}

        # Create lifecycle clients for each node
        for node_name in node_names:
            self.lifecycle_clients[node_name] = self.create_client(
                ChangeState,
                f'/{node_name}/change_state'
            )

    def _call_lifecycle_service(self, node_name, transition_id):
        """Helper method to call lifecycle service for a specific node."""
        if node_name not in self.lifecycle_clients:
            self.get_logger().error(f'No lifecycle client found for node: {node_name}')
            return False

        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.lifecycle_clients[node_name].call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def configure_nodes(self):
        """Configure all nodes."""
        results = []
        for node_name in self.node_names:
            result = self._call_lifecycle_service(node_name, Transition.TRANSITION_CONFIGURE)
            results.append(result)
        return all(results)

    def activate_nodes(self):
        """Activate all nodes."""
        results = []
        for node_name in self.node_names:
            print(f"Activating node: {node_name}")
            result = self._call_lifecycle_service(node_name, Transition.TRANSITION_ACTIVATE)
            print(f"Activation result for {node_name}: {result}")
            results.append(result)
        return all(results)

    def deactivate_nodes(self):
        """Deactivate all nodes."""
        results = []
        for node_name in self.node_names:
            result = self._call_lifecycle_service(node_name, Transition.TRANSITION_DEACTIVATE)
            results.append(result)
        return all(results)

    def shutdown_nodes(self):
        """Shutdown all nodes."""
        results = []
        for node_name in self.node_names:
            result = self._call_lifecycle_service(node_name, Transition.TRANSITION_INACTIVE_SHUTDOWN)
            results.append(result)
        return all(results)


class TestEmitterNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        print("\n=== Setting up Test Class ===")
        rclpy.init()
        cls.node = Node('test_emitter')

        # Define the set of node names
        cls.node_names = [
            'camera_node',
            'rectify_node',
            'tensor_encoder_node',
            'tensor_inference_node',
            'tensor_decode_node',
            'actuator_node'
        ]

        cls.lifecycle_node = LifecycleTestNode(cls.node_names)
        cls.received_messages = []

        # Wait for the nodes to be ready
        time.sleep(2.0)

        # Configure the nodes
        print("Configuring nodes")
        cls.lifecycle_node.configure_nodes()
        print("Configured nodes")

    @classmethod
    def tearDownClass(cls):
        print("\n=== Cleaning up Test Class ===")
        cls.node.destroy_node()
        print("Shutting down nodes")
        cls.lifecycle_node.shutdown_nodes()
        print("Shut down nodes")
        print("Destroying node")
        cls.lifecycle_node.destroy_node()
        print("Destroyed node")
        rclpy.shutdown()

    def setUp(self):
        # Clear received messages
        self.received_messages = []

        # Wait for the nodes to be ready
        time.sleep(2.0)

        # Activate the nodes
        print("Activating nodes...")
        activation_result = self.lifecycle_node.activate_nodes()
        print(f"Node activation result: {activation_result}")
        print("Activated nodes")

        # Create service clients for each node
        self.message_clients = {}
        service_name_by_node_name = {}

        # Print all available services first
        print("\nAvailable services in system:")
        for service, types in self.node.get_service_names_and_types():
            print(f"  {service}: {types}")

        for node_name in self.node_names:
            # Match exactly how the C++ code constructs the service name: "/" + get_name() + "/get_published_messages"
            service_name = f"/{node_name}/get_published_messages"  # No extra spaces, exact string construction
            print(f"\nTrying to connect to service: {service_name}")
            service_name_by_node_name[node_name] = service_name
            self.message_clients[node_name] = self.node.create_client(
                GetPublishedMessages,
                service_name
            )

        # Wait for all services to be available
        print("\nWaiting for services to be available...")
        start_time = time.time()
        services_ready = {node_name: False for node_name in self.node_names}

        while not all(services_ready.values()) and (time.time() - start_time) < 10.0:
            # Check each service in round-robin fashion
            for node_name, client in self.message_clients.items():
                if not services_ready[node_name]:
                    try:
                        # Use a very short timeout to check service availability
                        services_ready[node_name] = client.wait_for_service(timeout_sec=0.1)
                        if services_ready[node_name]:
                            print(f"Service {service_name_by_node_name[node_name]} is ready!")
                    except Exception as e:
                        print(f"Error checking {node_name} service: {str(e)}")

            # Print status of services not yet ready
            not_ready = [node_name for node_name, ready in services_ready.items() if not ready]
            if not_ready:
                print(f"\nServices not ready after {time.time() - start_time:.1f}s:")
                for node_name in not_ready:
                    print(f"  {service_name_by_node_name[node_name]}")
                    # Print service type for debugging
                    for service, types in self.node.get_service_names_and_types():
                        if service == service_name_by_node_name[node_name]:
                            print(f"    Type: {types}")

            # Small sleep to prevent tight loop
            time.sleep(0.1)

        # Final status check
        if all(services_ready.values()):
            print("\nAll services are ready!")
        else:
            print("\nSome services failed to become ready:")
            for node_name, ready in services_ready.items():
                if not ready:
                    print(f"  {service_name_by_node_name[node_name]}")
                    # Print service type for debugging
                    for service, types in self.node.get_service_names_and_types():
                        if service == service_name_by_node_name[node_name]:
                            print(f"    Type: {types}")

    def tearDown(self):
        self.lifecycle_node.deactivate_nodes()
        print("Deactivated nodes")

        # Destroy all message clients
        for client in self.message_clients.values():
            self.node.destroy_client(client)
        self.message_clients.clear()

    def test_get_published_messages_service(self):
        """Test the get_published_messages service."""
        # Wait for the service to be available
        self.assertTrue(
            self.message_clients['actuator_node'].wait_for_service(timeout_sec=5.0),
            "get_published_messages service not available"
        )

        time.sleep(5.0)

        # Call the service
        request = GetPublishedMessages.Request()
        future = self.message_clients['actuator_node'].call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        # Get the response
        response = future.result()
        self.assertTrue(response.success, "Service call failed")

        # Print published messages for each topic
        print("\nPublished Messages:")
        for i, topic in enumerate(response.published_topic_names):
            print(f"\nTopic: {topic}")
            for msg in response.published_messages:
                if msg.originator == topic.split('/')[0]:  # Get node name from topic
                    print(f"  Message: {msg.identifier}")
                    print(f"  Timestamp: {msg.publish_timestamp.sec}.{msg.publish_timestamp.nanosec}")

        # Print received messages for each topic
        print("\nReceived Messages:")
        for i, topic in enumerate(response.received_topic_names):
            print(f"\nTopic: {topic}")
            for j, msg_id in enumerate(response.received_message_timestamps[i].message_identifiers):
                timestamp = response.received_message_timestamps[i].timestamps[j]
                print(f"  Message: {msg_id}")
                print(f"  Timestamp: {timestamp.sec}.{timestamp.nanosec}")

        # Analyze latency between tensor decode and actuator nodes
        print("\nLatency Analysis (Tensor Decode -> Actuator):")

        # Create a map of message IDs to their publish timestamps from tensor decode node
        tensor_decode_timestamps = {}
        print("\nTensor Decode Published Messages:")
        for msg in response.published_messages:
            print(f"  Message: {msg.originator} {msg.identifier}")
            if msg.originator == "tensor_decode_node":
                print(f"  Message: {msg.identifier}")
                print(f"  Timestamp: {msg.publish_timestamp.sec}.{msg.publish_timestamp.nanosec}")
                tensor_decode_timestamps[msg.identifier] = msg.publish_timestamp

        # Find actuator node's received messages
        actuator_topic_index = None
        print("\nActuator Received Messages:")
        for i, topic in enumerate(response.received_topic_names):
            if topic == "/robot_cmd":  # Actuator node's subscription topic
                actuator_topic_index = i
                actuator_data = response.received_message_timestamps[i]
                print(f"Found actuator topic at index {i}")
                for j, msg_id in enumerate(actuator_data.message_identifiers):
                    timestamp = actuator_data.timestamps[j]
                    print(f"  Message: {msg_id}")
                    print(f"  Timestamp: {timestamp.sec}.{timestamp.nanosec}")
                break

        if actuator_topic_index is not None:
            actuator_data = response.received_message_timestamps[actuator_topic_index]
            print("\nMatching messages:")
            for j, msg_id in enumerate(actuator_data.message_identifiers):
                if msg_id in tensor_decode_timestamps:
                    publish_time = tensor_decode_timestamps[msg_id]
                    receive_time = actuator_data.timestamps[j]

                    # Calculate latency in seconds
                    latency_sec = (receive_time.sec - publish_time.sec) + \
                                (receive_time.nanosec - publish_time.nanosec) * 1e-9

                    print(f"\nMessage: {msg_id}")
                    print(f"  Published at: {publish_time.sec}.{publish_time.nanosec}")
                    print(f"  Received at: {receive_time.sec}.{receive_time.nanosec}")
                    print(f"  Latency: {latency_sec:.6f} seconds")
                else:
                    print(f"\nNo matching publish timestamp found for message: {msg_id}")


def main():
    unittest.main()


if __name__ == '__main__':
    main()