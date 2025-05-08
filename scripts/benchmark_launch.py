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
    # Create container with EmitterNode
    container = ComposableNodeContainer(
        name='emitter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[camera_node_composable, rectify_node_composable],
        output='screen'
    )

    return LaunchDescription([
        container,
        ReadyToTest()
    ])


class LifecycleTestNode(Node):
    def __init__(self):
        super().__init__('lifecycle_test_node')
        self.camera_lifecycle_client = self.create_client(
            ChangeState,
            '/camera_node/change_state'
        )
        self.rectify_lifecycle_client = self.create_client(
            ChangeState,
            '/rectify_node/change_state'
        )

    def configure_nodes(self):
        # Configure camera node
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        future = self.camera_lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        camera_result = future.result()

        # Configure rectify node
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        future = self.rectify_lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        rectify_result = future.result()

        return camera_result and rectify_result

    def activate_nodes(self):
        # Activate camera node
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        future = self.camera_lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        camera_result = future.result()

        # Activate rectify node
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        future = self.rectify_lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        rectify_result = future.result()

        return camera_result and rectify_result

    def deactivate_nodes(self):
        # Deactivate camera node
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        future = self.camera_lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        camera_result = future.result()

        # Deactivate rectify node
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        future = self.rectify_lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        rectify_result = future.result()

        return camera_result and rectify_result

    def shutdown_nodes(self):
        # Shutdown camera node
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_INACTIVE_SHUTDOWN
        future = self.camera_lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        camera_result = future.result()

        # Shutdown rectify node
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_INACTIVE_SHUTDOWN
        future = self.rectify_lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        rectify_result = future.result()

        return camera_result and rectify_result


class TestEmitterNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        print("\n=== Setting up Test Class ===")
        rclpy.init()
        cls.node = Node('test_emitter')
        cls.lifecycle_node = LifecycleTestNode()
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

        # Wait for services to be available
        while not self.camera_messages_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for camera get_published_messages service...')
        while not self.rectify_messages_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for rectify get_published_messages service...')

    def tearDown(self):
        self.lifecycle_node.deactivate_nodes()
        print("Deactivated nodes")

        self.node.destroy_client(self.camera_messages_client)
        self.node.destroy_client(self.rectify_messages_client)

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

        print(f"Camera response, published messages: {response.published_messages}")
        print(f"Rectify response, published messages: {response.published_messages}")


def main():
    unittest.main()


if __name__ == '__main__':
    main()