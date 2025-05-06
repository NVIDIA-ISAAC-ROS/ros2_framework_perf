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
#include "ros2_framework_perf_interfaces/msg/message_with_header.hpp"
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

  publisher_ = create_publisher<ros2_framework_perf_interfaces::msg::MessageWithHeader>("output", 10);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&EmitterNode::timer_callback, this));
}

void EmitterNode::timer_callback()
{

  // Subscribe to all topics specified
  // Configure the window, latest, or match across topics (Exact or approx)

  auto message = ros2_framework_perf_interfaces::msg::MessageWithHeader();
  rclcpp::Time current_time = this->now();
  message.header.stamp.sec = current_time.seconds();
  message.header.stamp.nanosec = current_time.nanoseconds();

  message.type = "std_msgs/String";
  message.sequence_number = sequence_number_++;
  message.originator = this->node_name_;
  message.identifier = message.originator + "_" + message.type + "_" + std::to_string(message.sequence_number);
  message.parent_messages = {};
  message.content = {1, 2, 3, 4, 5};

  publisher_->publish(message);


}

}  // namespace ros2_framework_perf


// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_framework_perf::EmitterNode)
