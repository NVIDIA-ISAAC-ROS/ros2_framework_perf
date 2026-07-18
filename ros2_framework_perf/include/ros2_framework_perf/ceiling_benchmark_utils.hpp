// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ROS2_FRAMEWORK_PERF__CEILING_BENCHMARK_UTILS_HPP_
#define ROS2_FRAMEWORK_PERF__CEILING_BENCHMARK_UTILS_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rcl/wait.h"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/waitable.hpp"

namespace ros2_framework_perf
{
namespace ceiling_benchmark
{

inline int64_t EnvInt(const char * name, const int64_t default_value)
{
  const char * value = std::getenv(name);
  if (value == nullptr || std::string(value).empty()) {
    return default_value;
  }
  return std::stoll(value);
}

inline uint64_t NowNs()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

inline std::string JsonEscape(const std::string & value)
{
  std::string result;
  result.reserve(value.size());
  for (const char character : value) {
    switch (character) {
      case '"':
        result += "\\\"";
        break;
      case '\\':
        result += "\\\\";
        break;
      case '\n':
        result += "\\n";
        break;
      case '\r':
        result += "\\r";
        break;
      case '\t':
        result += "\\t";
        break;
      default:
        result += character;
        break;
    }
  }
  return result;
}

struct WaitableTraceCounts
{
  uint64_t trigger{0};
  uint64_t on_ready_callback{0};
  uint64_t add_to_wait_set{0};
  uint64_t is_ready{0};
  uint64_t is_ready_true{0};
  uint64_t take_data{0};
  uint64_t take_data_by_entity_id{0};
  uint64_t execute{0};
};

class AlwaysReadyWaitable : public rclcpp::Waitable
{
public:
  explicit AlwaysReadyWaitable(std::function<void()> on_execute)
  : on_execute_(std::move(on_execute)),
    guard_condition_(std::make_shared<rclcpp::GuardCondition>())
  {
  }

  size_t get_number_of_ready_guard_conditions() override
  {
    return 1;
  }

  void add_to_wait_set(rcl_wait_set_t & wait_set) override
  {
    add_to_wait_set_count_.fetch_add(1, std::memory_order_relaxed);
    const rcl_ret_t result = rcl_wait_set_add_guard_condition(
      &wait_set, &guard_condition_->get_rcl_guard_condition(), &wait_set_guard_condition_index_);
    if (result != RCL_RET_OK) {
      throw std::runtime_error("Failed to add ceiling benchmark guard condition to wait set");
    }
  }

  bool is_ready(const rcl_wait_set_t & wait_set) override
  {
    is_ready_count_.fetch_add(1, std::memory_order_relaxed);
    const bool ready =
      wait_set.guard_conditions[wait_set_guard_condition_index_] != nullptr;
    if (ready) {
      is_ready_true_count_.fetch_add(1, std::memory_order_relaxed);
    }
    return ready;
  }

  std::shared_ptr<void> take_data() override
  {
    take_data_count_.fetch_add(1, std::memory_order_relaxed);
    return nullptr;
  }

  std::shared_ptr<void> take_data_by_entity_id(size_t) override
  {
    take_data_by_entity_id_count_.fetch_add(1, std::memory_order_relaxed);
    return nullptr;
  }

  std::vector<std::shared_ptr<rclcpp::TimerBase>> get_timers() const override
  {
    return {};
  }

  void execute(const std::shared_ptr<void> &) override
  {
    execute_count_.fetch_add(1, std::memory_order_relaxed);
    on_execute_();
  }

  void set_on_ready_callback(std::function<void(size_t, int)> callback) override
  {
    guard_condition_->set_on_trigger_callback(
      [this, callback](size_t number_of_events) {
        on_ready_callback_count_.fetch_add(1, std::memory_order_relaxed);
        callback(number_of_events, 0);
      });
  }

  void clear_on_ready_callback() override
  {
    guard_condition_->set_on_trigger_callback(nullptr);
  }

  void Trigger()
  {
    trigger_count_.fetch_add(1, std::memory_order_relaxed);
    guard_condition_->trigger();
  }

  WaitableTraceCounts TraceCounts() const
  {
    return WaitableTraceCounts{
      trigger_count_.load(std::memory_order_relaxed),
      on_ready_callback_count_.load(std::memory_order_relaxed),
      add_to_wait_set_count_.load(std::memory_order_relaxed),
      is_ready_count_.load(std::memory_order_relaxed),
      is_ready_true_count_.load(std::memory_order_relaxed),
      take_data_count_.load(std::memory_order_relaxed),
      take_data_by_entity_id_count_.load(std::memory_order_relaxed),
      execute_count_.load(std::memory_order_relaxed)};
  }

private:
  std::function<void()> on_execute_;
  std::shared_ptr<rclcpp::GuardCondition> guard_condition_;
  size_t wait_set_guard_condition_index_{0};
  std::atomic<uint64_t> trigger_count_{0};
  std::atomic<uint64_t> on_ready_callback_count_{0};
  std::atomic<uint64_t> add_to_wait_set_count_{0};
  std::atomic<uint64_t> is_ready_count_{0};
  std::atomic<uint64_t> is_ready_true_count_{0};
  std::atomic<uint64_t> take_data_count_{0};
  std::atomic<uint64_t> take_data_by_entity_id_count_{0};
  std::atomic<uint64_t> execute_count_{0};
};

inline void Accumulate(
  WaitableTraceCounts & total,
  const WaitableTraceCounts & value)
{
  total.trigger += value.trigger;
  total.on_ready_callback += value.on_ready_callback;
  total.add_to_wait_set += value.add_to_wait_set;
  total.is_ready += value.is_ready;
  total.is_ready_true += value.is_ready_true;
  total.take_data += value.take_data;
  total.take_data_by_entity_id += value.take_data_by_entity_id;
  total.execute += value.execute;
}

}  // namespace ceiling_benchmark
}  // namespace ros2_framework_perf

#endif  // ROS2_FRAMEWORK_PERF__CEILING_BENCHMARK_UTILS_HPP_
