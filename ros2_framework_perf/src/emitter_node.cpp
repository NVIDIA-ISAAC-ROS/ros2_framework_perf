// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "ros2_framework_perf/emitter_node.hpp"
#include "ros2_framework_perf_interfaces/msg/message_with_payload.hpp"
#include "ros2_framework_perf_interfaces/srv/get_published_messages.hpp"
#include "ros2_framework_perf_interfaces/msg/message_id_with_timestamps.hpp"
namespace ros2_framework_perf
{

EmitterNode::EmitterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("EmitterNode", options)
{
  // Declare parameters with default values
  declare_parameter("node_name", "EmitterNode");
  declare_parameter("frequency_hz", 10.0);

  // Get parameter values
  node_name_ = get_parameter("node_name").as_string();
  frequency_hz_ = get_parameter("frequency_hz").as_double();
  auto period_ms = static_cast<int64_t>(1000.0 / frequency_hz_);

  publisher_ = create_publisher<ros2_framework_perf_interfaces::msg::MessageWithPayload>("output", 10);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&EmitterNode::timer_callback, this));

  subscriber_ = create_subscription<ros2_framework_perf_interfaces::msg::MessageWithPayload>(
    "input",
    10,
    std::bind(&EmitterNode::subscriber_callback, this, std::placeholders::_1));

  get_published_messages_service_ = create_service<ros2_framework_perf_interfaces::srv::GetPublishedMessages>(
    "get_published_messages",
    std::bind(&EmitterNode::handle_get_published_messages, this, std::placeholders::_1, std::placeholders::_2));
}

void EmitterNode::subscriber_callback(const ros2_framework_perf_interfaces::msg::MessageWithPayload::SharedPtr msg)
{
  rclcpp::Time current_time = this->now();
  auto message = builtin_interfaces::msg::Time();
  message.sec = current_time.seconds();
  message.nanosec = current_time.nanoseconds();

  received_messages_by_topic_["input"].timestamps.push_back(message);
  received_messages_by_topic_["input"].message_identifiers.push_back(msg->info.identifier);
}

void EmitterNode::timer_callback()
{
  rclcpp::Time current_time = this->now();

  auto message = ros2_framework_perf_interfaces::msg::MessageWithPayload();

  // Set timestamp in both header and info
  message.header.stamp.sec = current_time.seconds();
  message.header.stamp.nanosec = current_time.nanoseconds();


  message.info.publish_timestamp.sec = current_time.seconds();
  message.info.publish_timestamp.nanosec = current_time.nanoseconds();
  message.info.type = "std_msgs/String";
  message.info.sequence_number = sequence_number_++;
  message.info.originator = this->node_name_;
  message.info.identifier = message.info.originator + "_" + message.info.type + "_" + std::to_string(message.info.sequence_number);
  message.info.parent_messages = {};

  message.payload = {1, 2, 3, 4, 5};

  publisher_->publish(message);
  published_messages_by_topic_["output"].push_back(message.info);
}

void EmitterNode::handle_get_published_messages(
  const std::shared_ptr<ros2_framework_perf_interfaces::srv::GetPublishedMessages::Request> request,
  std::shared_ptr<ros2_framework_perf_interfaces::srv::GetPublishedMessages::Response> response)
{
  response->success = true;

  // Reserve space for efficiency
  response->topic_names.reserve(published_messages_by_topic_.size());
  response->topic_message_timestamps.reserve(published_messages_by_topic_.size());

  // Convert the published messages map to the response format
  for (const auto& [topic, messages] : published_messages_by_topic_) {
    // Add topic name to the response
    response->topic_names.push_back(topic);

    ros2_framework_perf_interfaces::msg::MessageIdWithTimestamps topic_data;

    // Reserve space for efficiency
    topic_data.message_identifiers.reserve(messages.size());
    topic_data.timestamps.reserve(messages.size());

    // Add all message identifiers and timestamps for this topic
    for (const auto& msg : messages) {
      topic_data.message_identifiers.push_back(msg.info.identifier);
      topic_data.timestamps.push_back(msg.info.stamp);
    }

    response->topic_message_timestamps.push_back(topic_data);
  }
}

}  // namespace ros2_framework_perf


// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_framework_perf::EmitterNode)
