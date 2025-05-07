import time
import unittest

import rclpy
from rclpy.node import Node
from ros2_framework_perf_interfaces.msg import MessageWithPayload
from ros2_framework_perf_interfaces.srv import GetPublishedMessages


class TestEmitterNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_emitter')
        cls.received_messages = []

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        # Create subscription to receive messages
        self.subscription = self.node.create_subscription(
            MessageWithPayload,
            'output',
            self.message_callback,
            10
        )

        # Create service client to get published messages
        self.get_published_messages_client = self.node.create_client(
            GetPublishedMessages,
            'get_published_messages'
        )

        # Wait for service to be available
        while not self.get_published_messages_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for get_published_messages service...')

    def tearDown(self):
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_client(self.get_published_messages_client)

    def message_callback(self, msg):
        self.received_messages.append(msg)

    def test_emitter_publishes_messages(self):
        """Test that the emitter node publishes messages."""
        # Wait for some messages to be published
        time.sleep(2.0)

        # Check that we received messages
        self.assertGreater(len(self.received_messages), 0, "No messages were received")

        # Check message structure
        msg = self.received_messages[0]
        self.assertIsNotNone(msg.header.stamp, "Message header stamp is None")
        self.assertIsNotNone(msg.info.publish_timestamp, "Message info publish_timestamp is None")
        self.assertIsNotNone(msg.info.identifier, "Message info identifier is None")
        self.assertIsNotNone(msg.info.type, "Message info type is None")
        self.assertIsNotNone(msg.info.sequence_number, "Message info sequence_number is None")
        self.assertIsNotNone(msg.info.originator, "Message info originator is None")
        self.assertIsNotNone(msg.payload, "Message payload is None")

    def test_get_published_messages_service(self):
        """Test the get_published_messages service."""
        # Wait for some messages to be published
        time.sleep(2.0)

        # Call the service
        request = GetPublishedMessages.Request()
        future = self.get_published_messages_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        # Check the response
        response = future.result()
        self.assertTrue(response.success, "Service call was not successful")
        self.assertGreater(len(response.topic_names), 0, "No topics were returned")
        self.assertGreater(len(response.topic_message_timestamps), 0, "No message timestamps were returned")

        # Check that the output topic is in the response
        self.assertIn('output', response.topic_names, "Output topic not found in response")


def main():
    unittest.main()


if __name__ == '__main__':
    main()