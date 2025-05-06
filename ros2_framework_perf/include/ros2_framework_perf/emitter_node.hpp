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

#ifndef ROS2_FRAMEWORK_PERF__EMITTER_NODE_HPP_
#define ROS2_FRAMEWORK_PERF__EMITTER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_framework_perf_interfaces/msg/message_with_header.hpp"

namespace ros2_framework_perf
{

class EmitterNode : public rclcpp::Node
{
public:
  explicit EmitterNode(const rclcpp::NodeOptions & options);
  void timer_callback();

private:
  rclcpp::Publisher<ros2_framework_perf_interfaces::msg::MessageWithHeader>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string node_name_;
  double frequency_hz_ = 1.0;
  uint64_t sequence_number_ = 0;
};

}  // namespace ros2_framework_perf

#endif  // ROS2_FRAMEWORK_PERF__EMITTER_NODE_HPP_
