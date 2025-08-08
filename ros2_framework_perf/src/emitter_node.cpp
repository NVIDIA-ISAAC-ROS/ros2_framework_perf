// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
// SPDX-Generated-By: Cursor

#include "ros2_framework_perf/emitter_node.hpp"
#include "ros2_framework_perf_interfaces/msg/message_with_payload.hpp"
#include "ros2_framework_perf_interfaces/srv/get_published_messages.hpp"
#include "ros2_framework_perf_interfaces/msg/message_id_with_timestamps.hpp"
#include "ros2_framework_perf_interfaces/msg/lifecycle_transition.hpp"
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/exact_time.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executor_options.hpp"
#include "rclcpp/context.hpp"

namespace ros2_framework_perf
{

EmitterNode::EmitterNode(const rclcpp::NodeOptions & options)
: LifecycleNode("EmitterNode", options)
{
  // Log the intraprocess setting from NodeOptions
  bool intra_process_enabled = (options.use_intra_process_comms());
  RCLCPP_INFO(get_logger(), "EmitterNode created with use_intra_process_comms: %s",
    intra_process_enabled ? "enabled" : "disabled");

  steady_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
}

// Record lifecycle state transitions with steady-clock timestamps
void EmitterNode::record_lifecycle_transition(uint8_t state_id, const std::string& state_label) {
  ros2_framework_perf_interfaces::msg::LifecycleTransition transition;
  transition.state_id = state_id;
  transition.state_label = state_label;
  auto now = steady_clock_->now();
  transition.transition_time.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000);
  transition.transition_time.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);
  lifecycle_transitions_.push_back(transition);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmitterNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(get_logger(), "Configuring EmitterNode...");

  // Read parameters and YAML scenario description
  node_name_ = declare_parameter("node_name", "EmitterNode");
  yaml_config_ = declare_parameter<std::string>("yaml_config", "");
  size_t expected_messages = declare_parameter("expected_messages", 1000);  // Default to 1000 if not specified

  RCLCPP_INFO(get_logger(), "Expected messages for preallocation: %zu", expected_messages);

  // Parse YAML config
  YAML::Node config = YAML::Load(yaml_config_);

  // Parse timer groups (shared cadence for multiple publishers)
  if (config["timer_groups"]) {
    for (const auto& group : config["timer_groups"]) {
      std::string name = group["name"].as<std::string>();
      double frequency = group["frequency"].as<double>();
      timer_groups_[name] = TimerGroupConfig{frequency};
      timer_group_publishers_[name] = std::vector<std::string>();
    }
  }

  // Parse publishers and create per-topic resources
  if (config["publishers"]) {
    for (const auto& pub : config["publishers"]) {
      PublisherConfig pub_config;
      pub_config.topic_name = pub["topic_name"].as<std::string>();
      pub_config.message_size = pub["message_size"].as<size_t>();
      pub_config.message_type = pub["message_type"].as<std::string>();

      // Create publisher
      publishers_[pub_config.topic_name] = create_publisher<ros2_framework_perf_interfaces::msg::MessageWithPayload>(
        pub_config.topic_name,
        rclcpp::QoS(10).reliable().durability_volatile());

      // Preallocate storage to reduce runtime allocations
      published_messages_by_topic_[pub_config.topic_name].reserve(expected_messages);

      // Parse trigger
      if (pub["trigger"]["type"].as<std::string>() == "timer") {
        TimerTriggerConfig timer_config;
        if (pub["trigger"]["frequency"]) {
          timer_config.frequency = pub["trigger"]["frequency"].as<double>();
        }
        if (pub["trigger"]["timer_group"]) {
          timer_config.timer_group_name = pub["trigger"]["timer_group"].as<std::string>();
          timer_group_publishers_[*timer_config.timer_group_name].push_back(pub_config.topic_name);
        }
        if (pub["trigger"]["subscription_topics"]) {
          for (const auto& sub : pub["trigger"]["subscription_topics"]) {
            PublisherSubscriptionConfig sub_config;
            sub_config.topic = sub["topic_name"].as<std::string>();
            sub_config.mode = sub["mode"].as<std::string>();
            if (sub_config.mode == "window") {
              sub_config.window_time = sub["window_time"].as<double>();
            }
            timer_config.subscription_topics.push_back(sub_config);
          }
        }
        pub_config.trigger = timer_config;
      } else if (pub["trigger"]["type"].as<std::string>() == "message_received") {
        MessageReceivedTriggerConfig msg_config;
        msg_config.mode = pub["trigger"]["mode"].as<std::string>();
        if (msg_config.mode == "approximate_time") {
          msg_config.time_delta = pub["trigger"]["time_delta"].as<double>();
        } else if (msg_config.mode == "exact_time") {
          if (pub["trigger"]["window_size"]) {
            msg_config.window_size = pub["trigger"]["window_size"].as<size_t>();
          }
        }
        for (const auto& topic : pub["trigger"]["topics"]) {
          msg_config.topics.push_back(topic.as<std::string>());
        }
        pub_config.trigger = msg_config;
      }
      publisher_configs_.push_back(pub_config);
    }
  }

  // Parse standalone subscriptions used by timer-window/latest aggregation
  if (config["subscriptions"]) {
    for (const auto& sub : config["subscriptions"]) {
      SubscriptionConfig sub_config;
      sub_config.topic = sub["topic_name"].as<std::string>();
      subscription_configs_.push_back(sub_config);

      // Preallocate message storage based on expected messages
      received_messages_by_topic_[sub_config.topic].timestamps.reserve(expected_messages);
      received_messages_by_topic_[sub_config.topic].message_identifiers.reserve(expected_messages);

      // Create subscriber
      subscribers_[sub_config.topic] = create_subscription<ros2_framework_perf_interfaces::msg::MessageWithPayload>(
        sub_config.topic,
        rclcpp::QoS(10).reliable().durability_volatile(),
        [this, sub_config](const ros2_framework_perf_interfaces::msg::MessageWithPayload::SharedPtr msg) {
          auto now = steady_clock_->now();
          auto timestamp = builtin_interfaces::msg::Time();
          timestamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000);
          timestamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);

          RCLCPP_DEBUG(this->get_logger(), "Received message on topic %s: %s with timestamp %ld",
            sub_config.topic.c_str(), msg->info.identifier.c_str(), now.nanoseconds());

          received_messages_by_topic_[sub_config.topic].timestamps.push_back(timestamp);
          received_messages_by_topic_[sub_config.topic].message_identifiers.push_back(msg->info.identifier);
        });
      RCLCPP_DEBUG(get_logger(), "Created standalone subscriber for topic %s", sub_config.topic.c_str());
    }
  }

  // For message-received triggers: create message_filters subscribers and synchronizers
  for (const auto& config : publisher_configs_) {
    if (std::holds_alternative<MessageReceivedTriggerConfig>(config.trigger)) {
      const auto& msg_config = std::get<MessageReceivedTriggerConfig>(config.trigger);
      std::vector<std::shared_ptr<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>> subs;
      for (const auto& topic : msg_config.topics) {
        // Preallocate message storage based on expected messages
        received_messages_by_topic_[topic].timestamps.reserve(expected_messages);
        received_messages_by_topic_[topic].message_identifiers.reserve(expected_messages);

        // Create message filter subscriber for synchronization
        auto sub = std::make_shared<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>(
          this, topic, rclcpp::QoS(10).reliable().durability_volatile());
        subs.push_back(sub);
        message_filters_subscribers_by_topic_[config.topic_name].push_back(sub);
        RCLCPP_DEBUG(get_logger(), "Created message filter subscriber for topic %s", topic.c_str());
      }
      setup_synchronizer(config.topic_name, msg_config, subs);
    } else if (std::holds_alternative<TimerTriggerConfig>(config.trigger)) {
      const auto& timer_config = std::get<TimerTriggerConfig>(config.trigger);
      // Set up subscribers for each subscription topic used by timer aggregation
      for (const auto& sub_config : timer_config.subscription_topics) {
        // Preallocate message storage based on expected messages
        received_messages_by_topic_[sub_config.topic].timestamps.reserve(expected_messages);
        received_messages_by_topic_[sub_config.topic].message_identifiers.reserve(expected_messages);

        // Create subscriber for this topic
        subscribers_[sub_config.topic] = create_subscription<ros2_framework_perf_interfaces::msg::MessageWithPayload>(
          sub_config.topic,
          rclcpp::QoS(10).reliable().durability_volatile(),
          [this, sub_config](const ros2_framework_perf_interfaces::msg::MessageWithPayload::SharedPtr msg) {
            auto now = steady_clock_->now();
            auto timestamp = builtin_interfaces::msg::Time();
            timestamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000);
            timestamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);

            received_messages_by_topic_[sub_config.topic].timestamps.push_back(timestamp);
            received_messages_by_topic_[sub_config.topic].message_identifiers.push_back(msg->info.identifier);

            RCLCPP_DEBUG(get_logger(), "Received message on topic %s: %s",
              sub_config.topic.c_str(), msg->info.identifier.c_str());
          });
        RCLCPP_DEBUG(get_logger(), "Created subscriber for topic %s", sub_config.topic.c_str());
      }
    }
  }

  // Build integer-indexed lookups to avoid string hashing in hot paths
  size_t topic_id = 0;
  message_pool_by_id_.reserve(publisher_configs_.size());
  identifier_prefix_by_id_.reserve(publisher_configs_.size());
  config_by_id_.reserve(publisher_configs_.size());
  publisher_by_id_.reserve(publisher_configs_.size());

  // Map topic names to integer IDs and pre-create reusable message buffers
  for (const auto& config : publisher_configs_) {
    topic_to_id_map_[config.topic_name] = topic_id;

    auto message = std::make_unique<ros2_framework_perf_interfaces::msg::MessageWithPayload>();
    message->payload.resize(config.message_size, 0);

    message_pool_[config.topic_name] = std::move(message);
    auto message_copy = std::make_unique<ros2_framework_perf_interfaces::msg::MessageWithPayload>();
    message_copy->payload.resize(config.message_size, 0);
    message_pool_by_id_.push_back(std::move(message_copy));

    std::string prefix = node_name_ + "_" + config.message_type + "_";
    identifier_prefix_cache_[config.topic_name] = prefix;
    identifier_prefix_by_id_.push_back(std::move(prefix));

    topic_to_config_map_[config.topic_name] = &config;
    config_by_id_.push_back(&config);
    publisher_by_id_.push_back(publishers_[config.topic_name]);

    topic_id++;
  }

  for (const auto& config : publisher_configs_) {
    if (std::holds_alternative<TimerTriggerConfig>(config.trigger)) {
      const auto& timer_config = std::get<TimerTriggerConfig>(config.trigger);
      double frequency;
      if (timer_config.frequency) {
        frequency = *timer_config.frequency;
      } else {
        // Use timer group frequency
        frequency = timer_groups_[*timer_config.timer_group_name].frequency;
      }

      auto period_ns = std::chrono::nanoseconds(static_cast<int64_t>(1'000'000'000.0 / frequency));

      // Timer callback publishes on configured cadence; group leader publishes for entire group
      timers_[config.topic_name] = create_wall_timer(
        period_ns,
        [this, topic = config.topic_name]() {
          rclcpp::Time current_time = steady_clock_->now();

          auto topic_id_it = this->topic_to_id_map_.find(topic);
          if (topic_id_it != this->topic_to_id_map_.end()) {
            size_t topic_id = topic_id_it->second;
            const auto* pub_config = this->config_by_id_[topic_id];

            if (std::holds_alternative<TimerTriggerConfig>(pub_config->trigger)) {
              const auto& timer_config = std::get<TimerTriggerConfig>(pub_config->trigger);
              if (timer_config.timer_group_name) {

                if (this->timer_group_publishers_[*timer_config.timer_group_name][0] == topic) {
                  for (const auto& group_topic : this->timer_group_publishers_[*timer_config.timer_group_name]) {

                    auto group_id_it = this->topic_to_id_map_.find(group_topic);
                    if (group_id_it != this->topic_to_id_map_.end()) {
                      size_t group_id = group_id_it->second;
                      const auto* group_config = this->config_by_id_[group_id];
                      if (std::holds_alternative<TimerTriggerConfig>(group_config->trigger)) {
                        this->handle_timer_trigger_fast(group_id, std::get<TimerTriggerConfig>(group_config->trigger), current_time);
                      }
                    }
                  }
                }
              } else {
                this->handle_timer_trigger_fast(topic_id, timer_config, current_time);
              }
            }
          }
        });

      timers_[config.topic_name]->cancel();
    }
  }

  // Create service to expose published/received metadata
  get_published_messages_service_ = create_service<ros2_framework_perf_interfaces::srv::GetPublishedMessages>(
    "/" + std::string(get_name()) + "/get_published_messages",
    std::bind(&EmitterNode::handle_get_published_messages, this, std::placeholders::_1, std::placeholders::_2));

  record_lifecycle_transition(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  RCLCPP_DEBUG(get_logger(), "EmitterNode configured successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmitterNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(get_logger(), "Activating EmitterNode...");

  // Start timers
  for (auto& [topic, timer] : timers_) {
    timer->reset();
  }

  record_lifecycle_transition(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  RCLCPP_DEBUG(get_logger(), "EmitterNode activated successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmitterNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(get_logger(), "Deactivating EmitterNode...");

  // Stop timers
  for (auto& [topic, timer] : timers_) {
    timer->cancel();
  }

  record_lifecycle_transition(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  RCLCPP_DEBUG(get_logger(), "EmitterNode deactivated successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmitterNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(get_logger(), "Cleaning up EmitterNode...");

  for (auto& [topic, publisher] : publishers_) {
    publisher.reset();
  }

  for (auto& [topic, subscriber] : subscribers_) {
    subscriber.reset();
  }

  for (auto& [topic, timer] : timers_) {
    timer.reset();
  }

  get_published_messages_service_.reset();
  received_messages_by_topic_.clear();
  published_messages_by_topic_.clear();

  message_pool_.clear();
  identifier_prefix_cache_.clear();
  topic_to_config_map_.clear();

  record_lifecycle_transition(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  RCLCPP_DEBUG(get_logger(), "EmitterNode cleaned up successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmitterNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(get_logger(), "Shutting down EmitterNode...");

  for (auto& [topic, publisher] : publishers_) {
    publisher.reset();
  }

  for (auto& [topic, subscriber] : subscribers_) {
    subscriber.reset();
  }

  for (auto& [topic, timer] : timers_) {
    timer.reset();
  }

  get_published_messages_service_.reset();
  received_messages_by_topic_.clear();
  published_messages_by_topic_.clear();

  message_pool_.clear();
  identifier_prefix_cache_.clear();
  topic_to_config_map_.clear();

  record_lifecycle_transition(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, "finalized");
  RCLCPP_DEBUG(get_logger(), "EmitterNode shut down successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void EmitterNode::handle_timer_trigger(
  const std::string& topic_name,
  const TimerTriggerConfig& config,
  const rclcpp::Time& current_time)
{
  // Build and publish a message for a timer trigger (string-keyed path)
  auto& message = message_pool_[topic_name];

  // Reset message fields (much faster than allocation)
  message->info.parent_messages.clear();

  auto timestamp = builtin_interfaces::msg::Time();
  timestamp.sec = static_cast<int32_t>(current_time.nanoseconds() / 1000000000);
  timestamp.nanosec = static_cast<uint32_t>(current_time.nanoseconds() % 1000000000);

  message->header.stamp = timestamp;
  message->info.publish_timestamp = timestamp;
  message->info.topic_name = topic_name;

  const auto* pub_config = topic_to_config_map_[topic_name];
  message->info.type = pub_config->message_type;

  message->info.sequence_number = sequence_numbers_[topic_name]++;
  message->info.originator = this->node_name_;
  message->info.identifier = identifier_prefix_cache_[topic_name] + std::to_string(message->info.sequence_number);

  // Attach parent identifiers per window/latest policy
  for (const auto& sub_config : config.subscription_topics) {
    const auto& topic_data = received_messages_by_topic_[sub_config.topic];
    if (topic_data.timestamps.empty()) {
      continue;
    }

    if (sub_config.mode == "window") {
      double window_start = current_time.seconds() - sub_config.window_time;
      for (size_t i = 0; i < topic_data.timestamps.size(); ++i) {
        double msg_time = topic_data.timestamps[i].sec + topic_data.timestamps[i].nanosec * 1e-9;
        if (msg_time >= window_start) {
          message->info.parent_messages.push_back(topic_data.message_identifiers[i]);
        } else {
          break;
        }
      }
    } else if (sub_config.mode == "latest") {
      message->info.parent_messages.push_back(topic_data.message_identifiers.back());
    }
  }

  auto message_info = message->info;

  publishers_[topic_name]->publish(*message);
  published_messages_by_topic_[topic_name].push_back(message_info);
}

void EmitterNode::handle_timer_trigger_fast(
  size_t topic_id,
  const TimerTriggerConfig& config,
  const rclcpp::Time& current_time)
{
  // Optimized path using integer IDs and prebuilt buffers
  auto& message = message_pool_by_id_[topic_id];

  message->info.parent_messages.clear();

  auto timestamp = builtin_interfaces::msg::Time();
  timestamp.sec = static_cast<int32_t>(current_time.nanoseconds() / 1000000000);
  timestamp.nanosec = static_cast<uint32_t>(current_time.nanoseconds() % 1000000000);
  message->header.stamp = timestamp;
  message->info.publish_timestamp = timestamp;

  const auto* pub_config = config_by_id_[topic_id];
  message->info.topic_name = pub_config->topic_name;
  message->info.type = pub_config->message_type;

  const std::string& topic_name = pub_config->topic_name;
  message->info.sequence_number = sequence_numbers_[topic_name]++;
  message->info.originator = this->node_name_;
  message->info.identifier = identifier_prefix_by_id_[topic_id] + std::to_string(message->info.sequence_number);

  // Attach parent identifiers per window/latest policy
  for (const auto& sub_config : config.subscription_topics) {
    const auto& topic_data = received_messages_by_topic_[sub_config.topic];
    if (topic_data.timestamps.empty()) {
      continue;
    }

    if (sub_config.mode == "window") {
      double window_start = current_time.seconds() - sub_config.window_time;
      for (size_t i = 0; i < topic_data.timestamps.size(); ++i) {
        double msg_time = topic_data.timestamps[i].sec + topic_data.timestamps[i].nanosec * 1e-9;
        if (msg_time >= window_start) {
          message->info.parent_messages.push_back(topic_data.message_identifiers[i]);
        } else {
          break;
        }
      }
    } else if (sub_config.mode == "latest") {
      message->info.parent_messages.push_back(topic_data.message_identifiers.back());
    }
  }

  auto message_info = message->info;

  publisher_by_id_[topic_id]->publish(*message);
  published_messages_by_topic_[topic_name].push_back(message_info);
}

void EmitterNode::handle_message_received_trigger(
  const std::string& topic_name,
  [[maybe_unused]] const MessageReceivedTriggerConfig& config,
  const std::vector<ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr>& msgs)
{
  // Publish a message when synchronized input messages arrive
  rclcpp::Time current_time = steady_clock_->now();
  auto timestamp = builtin_interfaces::msg::Time();
  timestamp.sec = static_cast<int32_t>(current_time.nanoseconds() / 1000000000);
  timestamp.nanosec = static_cast<uint32_t>(current_time.nanoseconds() % 1000000000);

  auto message = std::make_unique<ros2_framework_perf_interfaces::msg::MessageWithPayload>();
  message->header.stamp = timestamp;
  message->info.publish_timestamp = timestamp;
  message->info.topic_name = topic_name;

  auto it = std::find_if(publisher_configs_.begin(), publisher_configs_.end(),
    [&topic_name](const PublisherConfig& config) { return config.topic_name == topic_name; });
  if (it != publisher_configs_.end()) {
    message->info.type = it->message_type;
    message->payload.resize(it->message_size, 0);
  } else {
    RCLCPP_ERROR(get_logger(), "Publisher config not found for topic %s", topic_name.c_str());
    return;
  }

  if (sequence_numbers_.find(topic_name) == sequence_numbers_.end()) {
    sequence_numbers_[topic_name] = 0;
  }
  message->info.sequence_number = sequence_numbers_[topic_name]++;
  message->info.originator = this->node_name_;
  message->info.identifier = message->info.originator + "_" + message->info.type + "_" + std::to_string(message->info.sequence_number);

  for (const auto& msg : msgs) {
    message->info.parent_messages.push_back(msg->info.identifier);
  }

  RCLCPP_DEBUG(get_logger(), "Publishing on message trigger to topic %s: %s", topic_name.c_str(), message->info.identifier.c_str());

  auto message_info = message->info;
  publishers_[topic_name]->publish(std::move(message));
  published_messages_by_topic_[topic_name].push_back(message_info);
}

void EmitterNode::handle_get_published_messages(
  [[maybe_unused]] const std::shared_ptr<ros2_framework_perf_interfaces::srv::GetPublishedMessages::Request> request,
  std::shared_ptr<ros2_framework_perf_interfaces::srv::GetPublishedMessages::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "Handling GetPublishedMessages service request");
  response->success = true;

  // Handle published messages
  response->published_topic_names.reserve(published_messages_by_topic_.size());
  response->published_messages.reserve(published_messages_by_topic_.size());

  for (const auto& [topic, messages] : published_messages_by_topic_) {
    response->published_topic_names.push_back(topic);
    for (const auto& msg : messages) {
      response->published_messages.push_back(msg);
      RCLCPP_DEBUG(get_logger(), "Added published message: %s with timestamp: %d.%d",
        msg.identifier.c_str(),
        msg.publish_timestamp.sec,
        msg.publish_timestamp.nanosec);
    }
  }

  // Handle received messages
  response->received_topic_names.reserve(received_messages_by_topic_.size());
  response->received_message_timestamps.reserve(received_messages_by_topic_.size());

  for (const auto& [topic, data] : received_messages_by_topic_) {
    response->received_topic_names.push_back(topic);

    ros2_framework_perf_interfaces::msg::MessageIdWithTimestamps topic_data;
    topic_data.message_identifiers = data.message_identifiers;
    topic_data.timestamps = data.timestamps;

    response->received_message_timestamps.push_back(topic_data);
    RCLCPP_DEBUG(get_logger(), "Added received messages for topic: %s, count: %zu",
      topic.c_str(), data.message_identifiers.size());
  }

  response->lifecycle_transitions = lifecycle_transitions_;

  RCLCPP_DEBUG(get_logger(), "GetPublishedMessages service request handled. Found %zu published topics and %zu received topics",
    response->published_topic_names.size(),
    response->received_topic_names.size());
}

// Helper functions for message synchronization
void EmitterNode::setup_synchronizer_2(
  const std::string& topic_name,
  const MessageReceivedTriggerConfig& msg_config,
  const std::vector<std::shared_ptr<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>>& subs)
{
  if (msg_config.mode == "exact_time") {
    using Policy = message_filters::sync_policies::ExactTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>;
    using Sync = message_filters::Synchronizer<Policy>;
    Policy policy(msg_config.window_size);
    auto sync = std::make_shared<Sync>(policy);
    sync->connectInput(*subs[0], *subs[1]);
    sync->registerCallback(
      [this, topic_name, msg_config](const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg1,
                                   const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg2) {
        std::vector<ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr> msgs;
        msgs.push_back(msg1);
        msgs.push_back(msg2);
        this->handle_message_received_trigger(topic_name, msg_config, msgs);
      });
    exact_syncs_2_[topic_name] = sync;
  } else {
    using Policy = message_filters::sync_policies::ApproximateTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>;
    using Sync = message_filters::Synchronizer<Policy>;
    Policy policy(msg_config.window_size);
    policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(msg_config.time_delta));
    auto sync = std::make_shared<Sync>(policy);
    sync->connectInput(*subs[0], *subs[1]);
    sync->registerCallback(
      [this, topic_name, msg_config](const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg1,
                                   const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg2) {
        std::vector<ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr> msgs;
        msgs.push_back(msg1);
        msgs.push_back(msg2);
        this->handle_message_received_trigger(topic_name, msg_config, msgs);
      });
    approx_syncs_2_[topic_name] = sync;
  }
}

void EmitterNode::setup_synchronizer_3(
  const std::string& topic_name,
  const MessageReceivedTriggerConfig& msg_config,
  const std::vector<std::shared_ptr<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>>& subs)
{
  if (msg_config.mode == "exact_time") {
    using Policy = message_filters::sync_policies::ExactTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>;
    using Sync = message_filters::Synchronizer<Policy>;
    Policy policy(msg_config.window_size);
    auto sync = std::make_shared<Sync>(policy);
    sync->connectInput(*subs[0], *subs[1], *subs[2]);
    sync->registerCallback(
      [this, topic_name, msg_config](const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg1,
                                   const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg2,
                                   const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg3) {
        std::vector<ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr> msgs;
        msgs.push_back(msg1);
        msgs.push_back(msg2);
        msgs.push_back(msg3);
        this->handle_message_received_trigger(topic_name, msg_config, msgs);
      });
    exact_syncs_3_[topic_name] = sync;
  } else {
    using Policy = message_filters::sync_policies::ApproximateTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>;
    using Sync = message_filters::Synchronizer<Policy>;
    Policy policy(msg_config.window_size);
    policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(msg_config.time_delta));
    auto sync = std::make_shared<Sync>(policy);
    sync->connectInput(*subs[0], *subs[1], *subs[2]);
    sync->registerCallback(
      [this, topic_name, msg_config](const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg1,
                                   const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg2,
                                   const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg3) {
        std::vector<ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr> msgs;
        msgs.push_back(msg1);
        msgs.push_back(msg2);
        msgs.push_back(msg3);
        this->handle_message_received_trigger(topic_name, msg_config, msgs);
      });
    approx_syncs_3_[topic_name] = sync;
  }
}

void EmitterNode::setup_synchronizer_4(
  const std::string& topic_name,
  const MessageReceivedTriggerConfig& msg_config,
  const std::vector<std::shared_ptr<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>>& subs)
{
  if (msg_config.mode == "exact_time") {
    using Policy = message_filters::sync_policies::ExactTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>;
    using Sync = message_filters::Synchronizer<Policy>;
    Policy policy(msg_config.window_size);
    auto sync = std::make_shared<Sync>(policy);
    sync->connectInput(*subs[0], *subs[1], *subs[2], *subs[3]);
    sync->registerCallback(
      [this, topic_name, msg_config](const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg1,
                                   const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg2,
                                   const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg3,
                                   const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg4) {
        std::vector<ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr> msgs;
        msgs.push_back(msg1);
        msgs.push_back(msg2);
        msgs.push_back(msg3);
        msgs.push_back(msg4);
        this->handle_message_received_trigger(topic_name, msg_config, msgs);
      });
    exact_syncs_4_[topic_name] = sync;
  } else {
    using Policy = message_filters::sync_policies::ApproximateTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>;
    using Sync = message_filters::Synchronizer<Policy>;
    Policy policy(msg_config.window_size);
    policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(msg_config.time_delta));
    auto sync = std::make_shared<Sync>(policy);
    sync->connectInput(*subs[0], *subs[1], *subs[2], *subs[3]);
    sync->registerCallback(
      [this, topic_name, msg_config](const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg1,
                                   const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg2,
                                   const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg3,
                                   const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg4) {
        std::vector<ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr> msgs;
        msgs.push_back(msg1);
        msgs.push_back(msg2);
        msgs.push_back(msg3);
        msgs.push_back(msg4);
        this->handle_message_received_trigger(topic_name, msg_config, msgs);
      });
    approx_syncs_4_[topic_name] = sync;
  }
}

void EmitterNode::setup_synchronizer(
  const std::string& topic_name,
  const MessageReceivedTriggerConfig& msg_config,
  const std::vector<std::shared_ptr<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>>& subs)
{
  // Special case for single topic - no synchronization needed
  if (subs.size() == 1) {
    subs[0]->registerCallback(
      [this, topic_name, msg_config](const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg) {
        // Store received message immediately
        auto now = steady_clock_->now();
        auto timestamp = builtin_interfaces::msg::Time();
        timestamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000);
        timestamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);

        received_messages_by_topic_[msg->info.topic_name].timestamps.push_back(timestamp);
        received_messages_by_topic_[msg->info.topic_name].message_identifiers.push_back(msg->info.identifier);

        // Handle trigger
        std::vector<ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr> msgs;
        msgs.push_back(msg);
        this->handle_message_received_trigger(topic_name, msg_config, msgs);
      });
    return;
  }

  // Ensure we have at least 2 and at most 4 subscribers for synchronization
  if (subs.size() < 2 || subs.size() > 4) {
    RCLCPP_ERROR(get_logger(), "Synchronizer requires between 2 and 4 topics, got %zu", subs.size());
    return;
  }

  // Register callback for each subscriber to record messages
  for (const auto& sub : subs) {
    sub->registerCallback(
      [this](const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg) {
        // Store received message immediately
        auto now = steady_clock_->now();
        auto timestamp = builtin_interfaces::msg::Time();
        timestamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000);
        timestamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);

        received_messages_by_topic_[msg->info.topic_name].timestamps.push_back(timestamp);
        received_messages_by_topic_[msg->info.topic_name].message_identifiers.push_back(msg->info.identifier);
      });
  }

  // Connect all subscribers at once based on number of topics
  switch (subs.size()) {
    case 2:
      setup_synchronizer_2(topic_name, msg_config, subs);
      break;
    case 3:
      setup_synchronizer_3(topic_name, msg_config, subs);
      break;
    case 4:
      setup_synchronizer_4(topic_name, msg_config, subs);
      break;
  }
}

}  // namespace ros2_framework_perf

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_framework_perf::EmitterNode)
