import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
import pytest
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

from ros2_framework_perf_interfaces.msg import MessageWithPayload
from ros2_framework_perf_interfaces.srv import GetPublishedMessages

class LifecycleTestNode(Node):
    def __init__(self):
        super().__init__('lifecycle_test_node')
        self.lifecycle_client = self.create_client(
            ChangeState,
            '/emitter_node/change_state'
        )

    def configure_node(self):
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        future = self.lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def activate_node(self):
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        future = self.lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def deactivate_node(self):
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        future = self.lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def inactive_shutdown_node(self):
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_INACTIVE_SHUTDOWN
        future = self.lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description for testing EmitterNode."""
    # Get the launch directory
    launch_dir = os.path.join(get_package_share_directory('ros2_framework_perf'), 'launch')

    # Include the emitter launch file
    emitter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'emitter.launch.py')),
    )

    print("\n=== Starting Test Run ===")
    return LaunchDescription([
        emitter_launch,
        # Tell launch when to start the test
        ReadyToTest()
    ])


class TestEmitterNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        print("\n=== Setting up Test Class ===")
        rclpy.init()
        cls.node = Node('test_emitter')
        cls.lifecycle_node = LifecycleTestNode()
        cls.received_messages = []

        # Wait for the node to be ready
        time.sleep(2.0)

        # Configure the node
        print("Configuring node")
        cls.lifecycle_node.configure_node()
        print("Configured node")


    @classmethod
    def tearDownClass(cls):
        print("\n=== Cleaning up Test Class ===")
        cls.node.destroy_node()
        print("Unconfiguring node")
        cls.lifecycle_node.inactive_shutdown_node()
        print("Unconfigured node")
        print("Destroying node")
        cls.lifecycle_node.destroy_node()
        print("Destroyed node")
        rclpy.shutdown()

    def setUp(self):
        # Wait for the node to be ready
        time.sleep(2.0)

        # Activate the node
        self.lifecycle_node.activate_node()
        print("Activated node")



        # Create service client to get published messages
        self.get_published_messages_client = self.node.create_client(
            GetPublishedMessages,
            'get_published_messages'
        )

        # Wait for service to be available
        while not self.get_published_messages_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for get_published_messages service...')



    def tearDown(self):
        self.lifecycle_node.deactivate_node()
        print("Deactivated node")


        self.node.destroy_client(self.get_published_messages_client)

    # def test_emitter_publishes_messages(self):
    #     """Test that the emitter node publishes messages."""

    #     # Create subscription to receive messages
    #     subscription = self.node.create_subscription(
    #         MessageWithPayload,
    #         'output',
    #         lambda msg: (
    #             self.received_messages.append(msg),
    #             print(f"Received message: {msg.info.identifier}")
    #         ),
    #         10
    #     )

    #     # Spin for 2 seconds to receive messages
    #     start_time = time.time()
    #     while rclpy.ok() and (time.time() - start_time) < 2.0:
    #         rclpy.spin_once(self.node, timeout_sec=0.1)

    #     # Check that we received messages
    #     self.assertGreater(len(self.received_messages), 0, "No messages were received")

    #     # Check message structure
    #     msg = self.received_messages[0]
    #     self.assertIsNotNone(msg.header.stamp, "Message header stamp is None")
    #     self.assertIsNotNone(msg.info.publish_timestamp, "Message info publish_timestamp is None")
    #     self.assertIsNotNone(msg.info.identifier, "Message info identifier is None")
    #     self.assertIsNotNone(msg.info.type, "Message info type is None")
    #     self.assertIsNotNone(msg.info.sequence_number, "Message info sequence_number is None")
    #     self.assertIsNotNone(msg.info.originator, "Message info originator is None")
    #     self.assertIsNotNone(msg.payload, "Message payload is None")

    #     self.node.destroy_subscription(subscription)

    def test_get_published_messages_service(self):
        """Test the get_published_messages service."""
        # Wait for some messages to be published
        time.sleep(1.0)

        # Call the service
        request = GetPublishedMessages.Request()
        future = self.get_published_messages_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        # Check the response
        response = future.result()
        self.assertTrue(response.success, "Service call was not successful")
        self.assertIn('output', response.published_topic_names, "Output topic not found in response")
        print(f"Response, published messages: {response.published_messages}")


def main():
    unittest.main()


if __name__ == '__main__':
    main()