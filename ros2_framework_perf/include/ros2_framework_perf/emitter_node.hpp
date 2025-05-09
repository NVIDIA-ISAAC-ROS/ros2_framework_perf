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
#include <variant>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/sync_policies/exact_time.hpp"
#include "ros2_framework_perf_interfaces/msg/message_with_payload.hpp"
#include "ros2_framework_perf_interfaces/msg/message_id_with_timestamps.hpp"
#include "ros2_framework_perf_interfaces/srv/get_published_messages.hpp"
#include <yaml-cpp/yaml.h>

namespace ros2_framework_perf
{

struct SubscriptionConfig {
  std::string topic;
  std::string mode;  // "window" or "latest"
  double window_time;  // Required when mode is "window"
};

struct TimerTriggerConfig {
  std::optional<double> frequency;  // Optional frequency if not using a timer group
  std::optional<std::string> timer_group_name;  // Optional timer group name
  std::vector<SubscriptionConfig> subscription_topics;
};

struct MessageReceivedTriggerConfig {
  std::string mode;  // "exact_time" or "approximate_time"
  double time_delta;  // Only used for approximate_time
  size_t window_size{10};  // Only used for exact_time, default to 10 messages
  std::vector<std::string> topics;
};

struct TimerGroupConfig {
  double frequency;
};

struct PublisherConfig {
  std::string topic_name;
  size_t message_size;
  std::variant<TimerTriggerConfig, MessageReceivedTriggerConfig> trigger;
};

class EmitterNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EmitterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Lifecycle node callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // Helper functions for message synchronization
  template<typename Policy>
  void setup_synchronizer_2(
    const std::string& topic_name,
    const MessageReceivedTriggerConfig& msg_config,
    const std::vector<std::shared_ptr<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>>& subs,
    Policy& policy);

  template<typename Policy>
  void setup_synchronizer_3(
    const std::string& topic_name,
    const MessageReceivedTriggerConfig& msg_config,
    const std::vector<std::shared_ptr<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>>& subs,
    Policy& policy);

  template<typename Policy>
  void setup_synchronizer_4(
    const std::string& topic_name,
    const MessageReceivedTriggerConfig& msg_config,
    const std::vector<std::shared_ptr<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>>& subs,
    Policy& policy);

  void setup_synchronizer(
    const std::string& topic_name,
    const MessageReceivedTriggerConfig& msg_config,
    const std::vector<std::shared_ptr<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>>& subs,
    const std::string& mode);

  // Callback functions
  void subscriber_callback(
    const ros2_framework_perf_interfaces::msg::MessageWithPayload::SharedPtr msg,
    const std::string& topic_name);
  void timer_callback();
  void handle_timer_trigger(
    const std::string& topic_name,
    const TimerTriggerConfig& config,
    const rclcpp::Time& current_time);
  void handle_subscription_message(
    const std::string& topic_name,
    const SubscriptionConfig& config,
    const ros2_framework_perf_interfaces::msg::MessageWithPayload::SharedPtr msg);
  void handle_message_received_trigger(
    const std::string& topic_name,
    const MessageReceivedTriggerConfig& config,
    const std::vector<ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr>& msgs);
  void handle_get_published_messages(
    const std::shared_ptr<ros2_framework_perf_interfaces::srv::GetPublishedMessages::Request> request,
    std::shared_ptr<ros2_framework_perf_interfaces::srv::GetPublishedMessages::Response> response);

  // Member variables
  std::string node_name_;
  std::string yaml_config_;
  uint64_t sequence_number_{0};
  size_t message_window_size_{10};  // Default window size for exact time matching
  std::map<std::string, rclcpp::Publisher<ros2_framework_perf_interfaces::msg::MessageWithPayload>::SharedPtr> publishers_;
  std::map<std::string, rclcpp::Subscription<ros2_framework_perf_interfaces::msg::MessageWithPayload>::SharedPtr> subscribers_;
  std::map<std::string, rclcpp::TimerBase::SharedPtr> timers_;
  std::map<std::string, TimerGroupConfig> timer_groups_;  // Map of timer group name to config
  std::map<std::string, std::vector<std::string>> timer_group_publishers_;  // Map of timer group name to list of publisher topics
  rclcpp::Service<ros2_framework_perf_interfaces::srv::GetPublishedMessages>::SharedPtr get_published_messages_service_;
  std::map<std::string, ros2_framework_perf_interfaces::msg::MessageIdWithTimestamps> received_messages_by_topic_;
  std::map<std::string, std::vector<ros2_framework_perf_interfaces::msg::MessageInfo>> published_messages_by_topic_;
  std::vector<PublisherConfig> publisher_configs_;

  // Message filters
  using MessageType = ros2_framework_perf_interfaces::msg::MessageWithPayload;
  std::map<std::string, std::shared_ptr<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>> message_filters_subscribers_;
  std::map<std::string, std::vector<std::shared_ptr<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>>> message_filters_subscribers_by_topic_;
  std::map<std::string, std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>>>> exact_syncs_2_;
  std::map<std::string, std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>>>> exact_syncs_3_;
  std::map<std::string, std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>>>> exact_syncs_4_;
  std::map<std::string, std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>>>> approx_syncs_2_;
  std::map<std::string, std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>>>> approx_syncs_3_;
  std::map<std::string, std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>>>> approx_syncs_4_;
};

}  // namespace ros2_framework_perf

#endif  // ROS2_FRAMEWORK_PERF_EMITTER_NODE_HPP_
