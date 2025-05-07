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

#ifndef ROS2_FRAMEWORK_PERF_EMITTER_NODE_HPP_
#define ROS2_FRAMEWORK_PERF_EMITTER_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_framework_perf_interfaces/msg/message_with_payload.hpp"
#include "ros2_framework_perf_interfaces/msg/message_id_with_timestamps.hpp"
#include "ros2_framework_perf_interfaces/srv/get_published_messages.hpp"

namespace ros2_framework_perf
{

class EmitterNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EmitterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Lifecycle state transitions
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  void subscriber_callback(const ros2_framework_perf_interfaces::msg::MessageWithPayload::SharedPtr msg);
  void timer_callback();
  void handle_get_published_messages(
    const std::shared_ptr<ros2_framework_perf_interfaces::srv::GetPublishedMessages::Request> request,
    std::shared_ptr<ros2_framework_perf_interfaces::srv::GetPublishedMessages::Response> response);

  std::string node_name_;
  double frequency_hz_;
  uint64_t sequence_number_ = 0;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<ros2_framework_perf_interfaces::msg::MessageWithPayload>::SharedPtr publisher_;
  rclcpp::Subscription<ros2_framework_perf_interfaces::msg::MessageWithPayload>::SharedPtr subscriber_;
  rclcpp::Service<ros2_framework_perf_interfaces::srv::GetPublishedMessages>::SharedPtr get_published_messages_service_;

  struct TopicData
  {
    std::vector<std::string> message_identifiers;
    std::vector<builtin_interfaces::msg::Time> timestamps;
  };

  std::unordered_map<std::string, TopicData> received_messages_by_topic_;
  std::unordered_map<std::string, std::vector<ros2_framework_perf_interfaces::msg::MessageInfo>> published_messages_by_topic_;
};

}  // namespace ros2_framework_perf

#endif  // ROS2_FRAMEWORK_PERF_EMITTER_NODE_HPP_
