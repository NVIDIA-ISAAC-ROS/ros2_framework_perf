import os
import time
import unittest
import json
from datetime import datetime

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
    frequency: 30.0
publishers:
  - topic_name: "/camera_image_left"
    message_size: 1024
    message_type: "sensor_msgs/Image"
    trigger:
      type: "timer"
      timer_group: "camera"
  - topic_name: "/camera_info_left"
    message_size: 1024
    message_type: "sensor_msgs/CameraInfo"
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
    message_type: "sensor_msgs/Image"
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
    message_type: "std_msgs/String"
    trigger:
      type: "timer"
      frequency: 500.0
      subscription_topics:
        - topic_name: "/camera_image_left_rectified"
          mode: "window"
          window_time: 0.1
        - topic_name: "/joint_states"
          mode: "window"
          window_time: 0.1
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
    message_type: "std_msgs/String"
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
    message_type: "std_msgs/String"
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

    joint_state_node_composable = ComposableNode(
        package='ros2_framework_perf',
        plugin='ros2_framework_perf::EmitterNode',
        name='joint_state_node',
        parameters=[{
            'node_name': 'joint_state_node',
            'yaml_config': '''
publishers:
  - topic_name: "/joint_states"
    message_size: 512
    message_type: "sensor_msgs/JointState"
    trigger:
      type: "timer"
      frequency: 500.0
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
            actuator_node_composable,
            joint_state_node_composable
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
            'actuator_node',
            'joint_state_node'
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

    def print_parent_chain(self, msg_id, node_name, published_messages_by_node, receive_time_sec, indent="", output_file=None):
        # Search for the message in any node's published messages
        msg = None
        found_node = None
        for candidate_node, node_data in published_messages_by_node.items():
            candidate_msgs = node_data['published_messages']
            msg = next((m for m in candidate_msgs if m.identifier == msg_id), None)
            if msg:
                found_node = candidate_node
                break
        if not msg:
            output_file.write(f"{indent}{node_name}: {msg_id} (not found in any node)\n")
            return
        pub_time = msg.publish_timestamp.sec + msg.publish_timestamp.nanosec * 1e-9
        latency = receive_time_sec - pub_time

        output_file.write(f"{indent}{found_node}: {msg.identifier} | Publish: {msg.publish_timestamp.sec}.{msg.publish_timestamp.nanosec} | Latency from Actuator: {latency:.6f}s\n")
        if msg.parent_messages:
            for parent_id in msg.parent_messages:
                self.print_parent_chain(parent_id, found_node, published_messages_by_node, receive_time_sec, indent + "    └── ", output_file)


    def collect_message_data(self, published_messages_by_node):
        """Collect message data from all nodes.

        Args:
            published_messages_by_node: Dictionary to store message data
        """
        # Collect messages from all nodes
        for node_name in self.node_names:
            print(f"Collecting messages from {node_name}...")
            self.assertTrue(
                self.message_clients[node_name].wait_for_service(timeout_sec=5.0),
                f"get_published_messages service not available for {node_name}"
            )

            # Call the service
            request = GetPublishedMessages.Request()
            future = self.message_clients[node_name].call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            # Get the response
            response = future.result()
            self.assertTrue(response.success, f"Service call failed for {node_name}")

            # Store published messages for this node
            published_messages_by_node[node_name] = {
                'published_messages': response.published_messages,
                'published_topics': response.published_topic_names,
                'received_messages': response.received_message_timestamps,
                'received_topics': response.received_topic_names
            }

    def write_raw_message_data(self, published_messages_by_node, output_file):
        """Write raw message data to a JSON file for further processing.

        Args:
            published_messages_by_node: Dictionary containing message data
            output_file: File object to write JSON data to
        """
        # Convert the data to a serializable format
        serializable_data = {}

        for node_name, node_data in published_messages_by_node.items():
            serializable_data[node_name] = {
                'published_messages': [
                    {
                        'identifier': msg.identifier,
                        'publish_timestamp': {
                            'sec': msg.publish_timestamp.sec,
                            'nanosec': msg.publish_timestamp.nanosec
                        },
                        'parent_messages': list(msg.parent_messages)
                    }
                    for msg in node_data['published_messages']
                ],
                'published_topics': list(node_data['published_topics']),
                'received_messages': [
                    {
                        'topic': topic,
                        'messages': [
                            {
                                'identifier': msg_id,
                                'timestamp': {
                                    'sec': timestamps.timestamps[j].sec,
                                    'nanosec': timestamps.timestamps[j].nanosec
                                }
                            }
                            for j, msg_id in enumerate(timestamps.message_identifiers)
                        ]
                    }
                    for i, (topic, timestamps) in enumerate(zip(
                        node_data['received_topics'],
                        node_data['received_messages']
                    ))
                ]
            }

        # Add metadata
        metadata = {
            'timestamp': datetime.now().isoformat(),
            'node_names': self.node_names
        }

        # Write to file
        json.dump({
            'metadata': metadata,
            'message_data': serializable_data
        }, output_file, indent=2)

    def test_get_published_messages_service(self):
        """Test the get_published_messages service and write raw data to file."""
        # Wait for 2 seconds to let the nodes publish messages
        time.sleep(2.0)

        # Dictionary to store published messages by node
        published_messages_by_node = {}

        # Collect message data from all nodes
        self.collect_message_data(published_messages_by_node)

        # Write raw message data to JSON file
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        raw_data_filename = f"raw_message_data_{timestamp}.json"
        with open(raw_data_filename, 'w') as raw_data_file:
            self.write_raw_message_data(published_messages_by_node, raw_data_file)
        print(f"Raw message data has been written to {raw_data_filename}")

def main():
    unittest.main()


if __name__ == '__main__':
    main()