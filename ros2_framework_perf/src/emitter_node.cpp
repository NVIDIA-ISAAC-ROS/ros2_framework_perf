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
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/exact_time.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"

namespace ros2_framework_perf
{

EmitterNode::EmitterNode(const rclcpp::NodeOptions & options)
: LifecycleNode("EmitterNode", options)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmitterNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring EmitterNode...");

  // Get node name and YAML config
  node_name_ = declare_parameter("node_name", "EmitterNode");;
  yaml_config_ = declare_parameter<std::string>("yaml_config", "");

  // Parse YAML config
  YAML::Node config = YAML::Load(yaml_config_);

  // Parse timer groups
  if (config["timer_groups"]) {
    for (const auto& group : config["timer_groups"]) {
      std::string name = group["name"].as<std::string>();
      double frequency = group["frequency"].as<double>();
      timer_groups_[name] = TimerGroupConfig{frequency};
      timer_group_publishers_[name] = std::vector<std::string>();
    }
  }

  // Parse publishers
  if (config["publishers"]) {
    for (const auto& pub : config["publishers"]) {
      PublisherConfig pub_config;
      pub_config.topic_name = pub["topic_name"].as<std::string>();
      pub_config.message_size = pub["message_size"].as<size_t>();

      // Create publisher
      publishers_[pub_config.topic_name] = create_publisher<ros2_framework_perf_interfaces::msg::MessageWithPayload>(
        pub_config.topic_name,
        rclcpp::QoS(10).reliable().durability_volatile());

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
            SubscriptionConfig sub_config;
            sub_config.topic = sub["topic"].as<std::string>();
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

  // Create subscribers for each topic in message received triggers
  for (const auto& config : publisher_configs_) {
    if (std::holds_alternative<MessageReceivedTriggerConfig>(config.trigger)) {
      const auto& msg_config = std::get<MessageReceivedTriggerConfig>(config.trigger);
      std::vector<std::shared_ptr<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>> subs;
      for (const auto& topic : msg_config.topics) {
        // Create message filter subscriber
        auto sub = std::make_shared<message_filters::Subscriber<ros2_framework_perf_interfaces::msg::MessageWithPayload>>(
          this, topic, rclcpp::QoS(10).reliable().durability_volatile());
        subs.push_back(sub);
        message_filters_subscribers_[config.topic_name].push_back(sub);
        RCLCPP_INFO(get_logger(), "Created message filter subscriber for topic %s", topic.c_str());
      }

      if (msg_config.mode == "exact_time") {
        using ExactSync = message_filters::Synchronizer<message_filters::sync_policies::ExactTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>>;
        using ExactPolicy = message_filters::sync_policies::ExactTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>;

        RCLCPP_INFO(get_logger(), "Setting up exact time synchronizer for topic %s with window size %zu",
          config.topic_name.c_str(), msg_config.window_size);

        ExactPolicy policy(msg_config.window_size);
        auto sync = std::make_shared<ExactSync>(policy);

        // Hardcode only two subscribers for now
        sync->connectInput(*subs[0], *subs[1]);

        sync->registerCallback(
          [this, topic_name = config.topic_name, msg_config](const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg1,
                                                           const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg2) {
            std::vector<ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr> msgs;
            msgs.push_back(msg1);
            msgs.push_back(msg2);
            this->handle_message_received_trigger(topic_name, msg_config, msgs);
          });
        exact_syncs_[config.topic_name] = sync;
      } else if (msg_config.mode == "approximate_time") {
        using ApproxSync = message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>>;
        using ApproxPolicy = message_filters::sync_policies::ApproximateTime<ros2_framework_perf_interfaces::msg::MessageWithPayload, ros2_framework_perf_interfaces::msg::MessageWithPayload>;
        ApproxPolicy policy(msg_config.window_size);
        policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(msg_config.time_delta));
        auto sync = std::make_shared<ApproxSync>(policy);
        for (const auto& sub : subs) {
          sync->connectInput(*sub);
        }
        sync->registerCallback(
          [this, topic_name = config.topic_name, msg_config](const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg1,
                                                           const ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr& msg2) {
            std::vector<ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr> msgs;
            msgs.push_back(msg1);
            msgs.push_back(msg2);
            this->handle_message_received_trigger(topic_name, msg_config, msgs);
          });
        approx_syncs_[config.topic_name] = sync;
      }
    }
  }

  // Create timers for each publisher with timer trigger
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
      int period_ms = static_cast<int>(1000.0 / frequency);
      timers_[config.topic_name] = create_wall_timer(
        std::chrono::milliseconds(period_ms),
        [this, topic = config.topic_name]() {
          // For timer groups, publish all messages in the group with the same timestamp
          auto it = std::find_if(this->publisher_configs_.begin(), this->publisher_configs_.end(),
            [&topic](const PublisherConfig& config) { return config.topic_name == topic; });
          if (it != this->publisher_configs_.end() && std::holds_alternative<TimerTriggerConfig>(it->trigger)) {
            const auto& timer_config = std::get<TimerTriggerConfig>(it->trigger);
            if (timer_config.timer_group_name) {
              // Only trigger the first publisher in the group, it will handle all publishers in the group
              if (this->timer_group_publishers_[*timer_config.timer_group_name][0] == topic) {
                rclcpp::Time current_time = this->now();
                for (const auto& group_topic : this->timer_group_publishers_[*timer_config.timer_group_name]) {
                  auto group_it = std::find_if(this->publisher_configs_.begin(), this->publisher_configs_.end(),
                    [&group_topic](const PublisherConfig& config) { return config.topic_name == group_topic; });
                  if (group_it != this->publisher_configs_.end() && std::holds_alternative<TimerTriggerConfig>(group_it->trigger)) {
                    this->handle_timer_trigger(group_topic, std::get<TimerTriggerConfig>(group_it->trigger), current_time);
                  }
                }
              }
            } else {
              this->handle_timer_trigger(topic, timer_config, this->now());
            }
          }
        });
      timers_[config.topic_name]->cancel();  // Don't start timer until node is activated
    }
  }

  // Create service
  get_published_messages_service_ = create_service<ros2_framework_perf_interfaces::srv::GetPublishedMessages>(
    "get_published_messages",
    std::bind(&EmitterNode::handle_get_published_messages, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "EmitterNode configured successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmitterNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating EmitterNode...");

  // Start timers
  for (auto& [topic, timer] : timers_) {
    timer->reset();
  }

  RCLCPP_INFO(get_logger(), "EmitterNode activated successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmitterNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating EmitterNode...");

  // Stop timers
  for (auto& [topic, timer] : timers_) {
    timer->cancel();
  }

  RCLCPP_INFO(get_logger(), "EmitterNode deactivated successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmitterNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up EmitterNode...");

  // Clean up publishers
  for (auto& [topic, publisher] : publishers_) {
    publisher.reset();
  }

  // Clean up subscribers
  for (auto& [topic, subscriber] : subscribers_) {
    subscriber.reset();
  }

  // Clean up timers
  for (auto& [topic, timer] : timers_) {
    timer.reset();
  }

  get_published_messages_service_.reset();
  received_messages_by_topic_.clear();
  published_messages_by_topic_.clear();

  RCLCPP_INFO(get_logger(), "EmitterNode cleaned up successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EmitterNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down EmitterNode...");

  // Clean up publishers
  for (auto& [topic, publisher] : publishers_) {
    publisher.reset();
  }

  // Clean up subscribers
  for (auto& [topic, subscriber] : subscribers_) {
    subscriber.reset();
  }

  // Clean up timers
  for (auto& [topic, timer] : timers_) {
    timer.reset();
  }

  get_published_messages_service_.reset();
  received_messages_by_topic_.clear();
  published_messages_by_topic_.clear();

  RCLCPP_INFO(get_logger(), "EmitterNode shut down successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void EmitterNode::subscriber_callback(
  const ros2_framework_perf_interfaces::msg::MessageWithPayload::SharedPtr msg,
  const std::string& topic_name)
{
  rclcpp::Time current_time = this->now();
  auto timestamp = builtin_interfaces::msg::Time();
  timestamp.sec = current_time.seconds();
  timestamp.nanosec = current_time.nanoseconds();

  RCLCPP_INFO(get_logger(), "Received message on topic %s: %s with timestamp %d.%d",
    topic_name.c_str(), msg->info.identifier.c_str(), timestamp.sec, timestamp.nanosec);

  // Store the received message
  received_messages_by_topic_[topic_name].timestamps.push_back(timestamp);
  received_messages_by_topic_[topic_name].message_identifiers.push_back(msg->info.identifier);

  // Check for publishers with message received triggers
  for (const auto& config : publisher_configs_) {
    if (std::holds_alternative<MessageReceivedTriggerConfig>(config.trigger)) {
      const auto& msg_config = std::get<MessageReceivedTriggerConfig>(config.trigger);
      // Check if this message's topic is in the trigger's topics list
      if (std::find(msg_config.topics.begin(), msg_config.topics.end(), topic_name) != msg_config.topics.end()) {
        // Create a vector with the single message
        std::vector<ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr> msgs;
        msgs.push_back(msg);
        handle_message_received_trigger(config.topic_name, msg_config, msgs);
      }
    } else if (std::holds_alternative<TimerTriggerConfig>(config.trigger)) {
      const auto& timer_config = std::get<TimerTriggerConfig>(config.trigger);
      // Check if this message's topic is in the subscription topics list
      for (const auto& sub_config : timer_config.subscription_topics) {
        if (sub_config.topic == topic_name) {
          handle_subscription_message(config.topic_name, sub_config, msg);
        }
      }
    }
  }
}

void EmitterNode::timer_callback()
{
  rclcpp::Time current_time = this->now();

  // Call handle_timer_trigger for each publisher with a timer trigger
  for (const auto& config : publisher_configs_) {
    if (std::holds_alternative<TimerTriggerConfig>(config.trigger)) {
      const auto& timer_config = std::get<TimerTriggerConfig>(config.trigger);
      handle_timer_trigger(config.topic_name, timer_config, current_time);
    }
  }
}

void EmitterNode::handle_timer_trigger(
  const std::string& topic_name,
  const TimerTriggerConfig& config,
  const rclcpp::Time& current_time)
{
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

  // Get parent messages based on subscription mode
  for (const auto& sub_config : config.subscription_topics) {
    const auto& topic_data = received_messages_by_topic_[sub_config.topic];
    if (topic_data.timestamps.empty()) {
      continue;
    }

    if (sub_config.mode == "window") {
      // For window mode, include all messages within the time range
      double window_start = current_time.seconds() - sub_config.window_time;
      for (size_t i = 0; i < topic_data.timestamps.size(); ++i) {
        double msg_time = topic_data.timestamps[i].sec + topic_data.timestamps[i].nanosec * 1e-9;
        if (msg_time >= window_start) {
          message.info.parent_messages.push_back(topic_data.message_identifiers[i]);
        }
      }
    } else if (sub_config.mode == "latest") {
      // For latest mode, only include the most recent message
      message.info.parent_messages.push_back(topic_data.message_identifiers.back());
    }
  }

  // Get the publisher config to access message_size
  auto it = std::find_if(publisher_configs_.begin(), publisher_configs_.end(),
    [&topic_name](const PublisherConfig& config) { return config.topic_name == topic_name; });
  if (it != publisher_configs_.end()) {
    message.payload.resize(it->message_size, 0);
  }

  RCLCPP_INFO(get_logger(), "Publishing on timer at time %f:%ld on topic %s: %s", current_time.seconds(), current_time.nanoseconds(), topic_name.c_str(), message.info.identifier.c_str());
  publishers_[topic_name]->publish(message);
  published_messages_by_topic_[topic_name].push_back(message.info);
}

void EmitterNode::handle_subscription_message(
  const std::string& topic_name,
  const SubscriptionConfig& config,
  const ros2_framework_perf_interfaces::msg::MessageWithPayload::SharedPtr msg)
{
  rclcpp::Time current_time = this->now();
  auto timestamp = builtin_interfaces::msg::Time();
  timestamp.sec = current_time.seconds();
  timestamp.nanosec = current_time.nanoseconds();

  if (config.mode == "window") {
    // Store all messages in window mode
    received_messages_by_topic_[topic_name].timestamps.push_back(timestamp);
    received_messages_by_topic_[topic_name].message_identifiers.push_back(msg->info.identifier);
  }
  else if (config.mode == "latest") {
    // Only keep the latest message
    if (received_messages_by_topic_[topic_name].timestamps.empty()) {
      received_messages_by_topic_[topic_name].timestamps.push_back(timestamp);
      received_messages_by_topic_[topic_name].message_identifiers.push_back(msg->info.identifier);
    }
    else {
      received_messages_by_topic_[topic_name].timestamps.back() = timestamp;
      received_messages_by_topic_[topic_name].message_identifiers.back() = msg->info.identifier;
    }
  }
}

void EmitterNode::handle_message_received_trigger(
  const std::string& topic_name,
  [[maybe_unused]] const MessageReceivedTriggerConfig& config,
  const std::vector<ros2_framework_perf_interfaces::msg::MessageWithPayload::ConstSharedPtr>& msgs)
{
  rclcpp::Time current_time = this->now();
  auto timestamp = builtin_interfaces::msg::Time();
  timestamp.sec = current_time.seconds();
  timestamp.nanosec = current_time.nanoseconds();

  // Create and publish the message
  auto message = ros2_framework_perf_interfaces::msg::MessageWithPayload();
  message.header.stamp = timestamp;
  message.info.publish_timestamp = timestamp;
  message.info.type = "std_msgs/String";
  message.info.sequence_number = sequence_number_++;
  message.info.originator = this->node_name_;
  message.info.identifier = message.info.originator + "_" + message.info.type + "_" + std::to_string(message.info.sequence_number);

  // Add parent message identifiers
  for (const auto& msg : msgs) {
    message.info.parent_messages.push_back(msg->info.identifier);
  }

  // Get the publisher config to access message_size
  auto it = std::find_if(publisher_configs_.begin(), publisher_configs_.end(),
    [&topic_name](const PublisherConfig& config) { return config.topic_name == topic_name; });
  if (it != publisher_configs_.end()) {
    message.payload.resize(it->message_size, 0);
  }

  RCLCPP_INFO(get_logger(), "Publishing on message trigger to topic %s: %s", topic_name.c_str(), message.info.identifier.c_str());
  publishers_[topic_name]->publish(message);
  published_messages_by_topic_[topic_name].push_back(message.info);
}

void EmitterNode::handle_get_published_messages(
  [[maybe_unused]] const std::shared_ptr<ros2_framework_perf_interfaces::srv::GetPublishedMessages::Request> request,
  std::shared_ptr<ros2_framework_perf_interfaces::srv::GetPublishedMessages::Response> response)
{
  RCLCPP_INFO(get_logger(), "Handling GetPublishedMessages service request");
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

  RCLCPP_INFO(get_logger(), "GetPublishedMessages service request handled. Found %zu published topics and %zu received topics",
    response->published_topic_names.size(),
    response->received_topic_names.size());
}

}  // namespace ros2_framework_perf

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_framework_perf::EmitterNode)
