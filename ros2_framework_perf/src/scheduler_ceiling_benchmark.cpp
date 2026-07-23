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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_framework_perf/ceiling_benchmark_utils.hpp"

#ifndef ROS2_FRAMEWORK_PERF_BUILD_TYPE
#define ROS2_FRAMEWORK_PERF_BUILD_TYPE "unknown"
#endif

namespace
{

using ros2_framework_perf::ceiling_benchmark::Accumulate;
using ros2_framework_perf::ceiling_benchmark::AlwaysReadyWaitable;
using ros2_framework_perf::ceiling_benchmark::EnvInt;
using ros2_framework_perf::ceiling_benchmark::JsonEscape;
using ros2_framework_perf::ceiling_benchmark::NowNs;
using ros2_framework_perf::ceiling_benchmark::WaitableTraceCounts;

class SchedulerNode : public rclcpp::Node
{
public:
  SchedulerNode(const size_t index, const int64_t operation_count)
  : Node("ceiling_scheduler_" + std::to_string(index)),
    operation_count_(operation_count)
  {
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    waitable_ = std::make_shared<AlwaysReadyWaitable>([this]() {ExecuteOnce();});
    get_node_waitables_interface()->add_waitable(waitable_, callback_group_);
  }

  void Kick()
  {
    waitable_->Trigger();
  }

  bool Complete() const
  {
    return complete_.load(std::memory_order_acquire);
  }

  uint64_t Count() const
  {
    return count_.load(std::memory_order_relaxed);
  }

  uint64_t FirstOperationNs() const
  {
    return first_operation_ns_.load(std::memory_order_relaxed);
  }

  uint64_t LastOperationNs() const
  {
    return last_operation_ns_.load(std::memory_order_relaxed);
  }

  WaitableTraceCounts TraceCounts() const
  {
    return waitable_->TraceCounts();
  }

private:
  void ExecuteOnce()
  {
    const uint64_t previous = count_.fetch_add(1, std::memory_order_acq_rel);
    if (previous == 0) {
      first_operation_ns_.store(NowNs(), std::memory_order_release);
    }

    if (previous + 1 >= static_cast<uint64_t>(operation_count_)) {
      last_operation_ns_.store(NowNs(), std::memory_order_release);
      complete_.store(true, std::memory_order_release);
      return;
    }

    waitable_->Trigger();
  }

  const int64_t operation_count_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<AlwaysReadyWaitable> waitable_;
  std::atomic<uint64_t> count_{0};
  std::atomic<uint64_t> first_operation_ns_{0};
  std::atomic<uint64_t> last_operation_ns_{0};
  std::atomic<bool> complete_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    const int64_t thread_count =
      EnvInt("ROS2_FRAMEWORK_PERF_CEILING_THREADS", 1);
    const int64_t operator_count =
      EnvInt("ROS2_FRAMEWORK_PERF_CEILING_OPERATORS", 1);
    const int64_t operations_per_operator =
      EnvInt("ROS2_FRAMEWORK_PERF_CEILING_OPERATIONS", 100000);
    const int64_t timeout_seconds =
      EnvInt("ROS2_FRAMEWORK_PERF_CEILING_TIMEOUT_SEC", 60);

    if (thread_count <= 0 || operator_count <= 0 ||
      operations_per_operator <= 0 || timeout_seconds <= 0)
    {
      throw std::runtime_error(
              "threads, operators, operations, and timeout must be greater than zero");
    }

    auto executor = std::make_shared<rclcpp::executors::EventsCBGExecutor>(
      rclcpp::ExecutorOptions{}, static_cast<size_t>(thread_count));
    std::vector<std::shared_ptr<SchedulerNode>> nodes;
    nodes.reserve(static_cast<size_t>(operator_count));

    for (int64_t index = 0; index < operator_count; ++index) {
      auto node = std::make_shared<SchedulerNode>(
        static_cast<size_t>(index), operations_per_operator);
      executor->add_node(node);
      nodes.push_back(std::move(node));
    }

    for (const auto & node : nodes) {
      node->Kick();
    }

    std::atomic<bool> stop_monitor{false};
    std::thread monitor(
      [&]() {
        const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::seconds(timeout_seconds);
        while (!stop_monitor.load(std::memory_order_relaxed) &&
        std::chrono::steady_clock::now() < deadline)
        {
          if (std::all_of(
              nodes.begin(), nodes.end(),
            [](const auto & node) {return node->Complete();}))
          {
            executor->cancel();
            return;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        executor->cancel();
      });

    try {
      executor->spin();
    } catch (...) {
      stop_monitor.store(true, std::memory_order_relaxed);
      executor->cancel();
      monitor.join();
      throw;
    }
    stop_monitor.store(true, std::memory_order_relaxed);
    executor->cancel();
    monitor.join();

    uint64_t total_operations = 0;
    uint64_t first_operation_ns = UINT64_MAX;
    uint64_t last_operation_ns = 0;
    bool complete = true;
    WaitableTraceCounts trace_counts;

    for (const auto & node : nodes) {
      total_operations += node->Count();
      if (node->FirstOperationNs() > 0) {
        first_operation_ns =
          std::min(first_operation_ns, node->FirstOperationNs());
      }
      last_operation_ns =
        std::max(last_operation_ns, node->LastOperationNs());
      complete = complete && node->Complete();
      Accumulate(trace_counts, node->TraceCounts());
    }

    const uint64_t expected_operations =
      static_cast<uint64_t>(operations_per_operator) *
      static_cast<uint64_t>(operator_count);
    const bool waitable_invariant =
      trace_counts.execute == total_operations;
    complete = complete && total_operations == expected_operations &&
      waitable_invariant;

    const double duration_seconds =
      first_operation_ns != UINT64_MAX &&
      last_operation_ns > first_operation_ns ?
      static_cast<double>(last_operation_ns - first_operation_ns) / 1e9 : 0.0;
    const double throughput =
      duration_seconds > 0.0 ?
      static_cast<double>(total_operations) / duration_seconds : 0.0;

    std::ostringstream output;
    output << "{"
           << "\"benchmark\":\"scheduler_dispatch\","
           << "\"ready_mode\":\"waitable\","
           << "\"executor\":\"events_cbg\","
           << "\"build_type\":\""
           << JsonEscape(ROS2_FRAMEWORK_PERF_BUILD_TYPE) << "\","
           << "\"threads\":" << thread_count << ","
           << "\"operators\":" << operator_count << ","
           << "\"operations_per_operator\":" << operations_per_operator << ","
           << "\"expected_operations\":" << expected_operations << ","
           << "\"total_operations\":" << total_operations << ","
           << "\"complete\":" << (complete ? "true" : "false") << ","
           << "\"waitable_invariant\":"
           << (waitable_invariant ? "true" : "false") << ","
           << "\"waitable_trigger_count\":" << trace_counts.trigger << ","
           << "\"waitable_on_ready_callback_count\":"
           << trace_counts.on_ready_callback << ","
           << "\"waitable_add_to_wait_set_count\":"
           << trace_counts.add_to_wait_set << ","
           << "\"waitable_is_ready_count\":" << trace_counts.is_ready << ","
           << "\"waitable_is_ready_true_count\":"
           << trace_counts.is_ready_true << ","
           << "\"waitable_take_data_count\":" << trace_counts.take_data << ","
           << "\"waitable_take_data_by_entity_id_count\":"
           << trace_counts.take_data_by_entity_id << ","
           << "\"waitable_execute_count\":" << trace_counts.execute << ","
           << "\"throughput_ops_s\":" << throughput << ","
           << "\"duration_s\":" << duration_seconds
           << "}";

    std::cout << output.str() << std::endl;
    rclcpp::shutdown();
    return complete ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << "{\"benchmark\":\"scheduler_dispatch\","
              << "\"complete\":false,\"error\":\""
              << JsonEscape(error.what()) << "\"}" << std::endl;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 1;
}
