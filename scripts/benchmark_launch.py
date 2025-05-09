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
            'node_name': 'CameraImager01',
            'yaml_config': '''
timer_groups:
  - name: "camera"
    frequency: 5.0
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

    rectify_node_composable = ComposableNode(
        package='ros2_framework_perf',
        plugin='ros2_framework_perf::EmitterNode',
        name='rectify_node',
        parameters=[{
            'node_name': 'RectifyNode',
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
            'node_name': 'TensorEncoderNode',
            'yaml_config': '''
publishers:
  - topic_name: "/tensor_encoder_output"
    message_size: 1024
    trigger:
      type: "timer"
      frequency: 10.0
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
            'node_name': 'TensorInferenceNode',
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
            'node_name': 'TensorDecodeNode',
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
            'node_name': 'ActuatorNode',
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
            result = self._call_lifecycle_service(node_name, Transition.TRANSITION_ACTIVATE)
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
        self.lifecycle_node.activate_nodes()
        print("Activated nodes")

        # Create service clients to get published messages
        self.camera_messages_client = self.node.create_client(
            GetPublishedMessages,
            '/camera_node/get_published_messages'
        )
        self.rectify_messages_client = self.node.create_client(
            GetPublishedMessages,
            '/rectify_node/get_published_messages'
        )
        self.tensor_encoder_messages_client = self.node.create_client(
            GetPublishedMessages,
            '/tensor_encoder_node/get_published_messages'
        )
        self.tensor_inference_messages_client = self.node.create_client(
            GetPublishedMessages,
            '/tensor_inference_node/get_published_messages'
        )
        self.tensor_decode_messages_client = self.node.create_client(
            GetPublishedMessages,
            '/tensor_decode_node/get_published_messages'
        )
        self.actuator_messages_client = self.node.create_client(
            GetPublishedMessages,
            '/actuator_node/get_published_messages'
        )

        # Wait for services to be available
        while not self.camera_messages_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for camera get_published_messages service...')
        while not self.rectify_messages_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for rectify get_published_messages service...')
        while not self.tensor_encoder_messages_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for tensor encoder get_published_messages service...')
        while not self.tensor_inference_messages_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for tensor inference get_published_messages service...')
        while not self.tensor_decode_messages_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for tensor decode get_published_messages service...')
        while not self.actuator_messages_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for actuator get_published_messages service...')

    def tearDown(self):
        self.lifecycle_node.deactivate_nodes()
        print("Deactivated nodes")

        self.node.destroy_client(self.camera_messages_client)
        self.node.destroy_client(self.rectify_messages_client)
        self.node.destroy_client(self.tensor_encoder_messages_client)
        self.node.destroy_client(self.tensor_inference_messages_client)
        self.node.destroy_client(self.tensor_decode_messages_client)
        self.node.destroy_client(self.actuator_messages_client)

    def test_get_published_messages_service(self):
        """Test the get_published_messages service."""
        # Wait for some messages to be published
        time.sleep(2.0)

        # Call the camera service
        request = GetPublishedMessages.Request()
        future = self.camera_messages_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        # Check the camera response
        response = future.result()
        self.assertTrue(response.success, "Camera service call was not successful")
        self.assertIn('/camera_image_left', response.published_topic_names, "Camera image topic not found in response")
        self.assertIn('/camera_info_left', response.published_topic_names, "Camera info topic not found in response")
        self.assertGreater(len(response.published_messages), 0, "No published messages in camera response")

        # Call the rectify service
        request = GetPublishedMessages.Request()
        future = self.rectify_messages_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        # Check the rectify response
        response = future.result()
        self.assertTrue(response.success, "Rectify service call was not successful")
        self.assertIn('/camera_image_left_rectified', response.published_topic_names, "Rectified image topic not found in response")
        self.assertGreater(len(response.published_messages), 0, "No published messages in rectify response")

        # Call the tensor encoder service
        request = GetPublishedMessages.Request()
        future = self.tensor_encoder_messages_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        # Check the tensor encoder response
        response = future.result()
        self.assertTrue(response.success, "Tensor encoder service call was not successful")
        self.assertIn('/tensor_encoder_output', response.published_topic_names, "Tensor encoder output topic not found in response")
        self.assertGreater(len(response.published_messages), 0, "No published messages in tensor encoder response")

        # Call the tensor inference service
        request = GetPublishedMessages.Request()
        future = self.tensor_inference_messages_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        # Check the tensor inference response
        response = future.result()
        self.assertTrue(response.success, "Tensor inference service call was not successful")
        self.assertIn('/tensor_inference_output', response.published_topic_names, "Tensor inference output topic not found in response")
        self.assertGreater(len(response.published_messages), 0, "No published messages in tensor inference response")

        # Call the tensor decode service
        request = GetPublishedMessages.Request()
        future = self.tensor_decode_messages_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        # Check the tensor decode response
        response = future.result()
        self.assertTrue(response.success, "Tensor decode service call was not successful")
        self.assertIn('/robot_cmd', response.published_topic_names, "Robot command topic not found in response")
        self.assertGreater(len(response.published_messages), 0, "No published messages in tensor decode response")

        print(f"Camera response, published messages: {response.published_messages}")
        print(f"Rectify response, published messages: {response.published_messages}")
        print(f"Tensor encoder response, published messages: {response.published_messages}")
        print(f"Tensor inference response, published messages: {response.published_messages}")
        print(f"Tensor decode response, published messages: {response.published_messages}")


def main():
    unittest.main()


if __name__ == '__main__':
    main()