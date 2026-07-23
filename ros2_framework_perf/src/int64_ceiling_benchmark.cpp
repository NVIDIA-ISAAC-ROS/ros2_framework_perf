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
#include "std_msgs/msg/int64.hpp"

#ifndef ROS2_FRAMEWORK_PERF_BUILD_TYPE
#define ROS2_FRAMEWORK_PERF_BUILD_TYPE "unknown"
#endif

namespace
{

using Int64 = std_msgs::msg::Int64;
using ros2_framework_perf::ceiling_benchmark::Accumulate;
using ros2_framework_perf::ceiling_benchmark::AlwaysReadyWaitable;
using ros2_framework_perf::ceiling_benchmark::EnvInt;
using ros2_framework_perf::ceiling_benchmark::JsonEscape;
using ros2_framework_perf::ceiling_benchmark::NowNs;
using ros2_framework_perf::ceiling_benchmark::WaitableTraceCounts;

class SourceNode : public rclcpp::Node
{
public:
  SourceNode(
    const size_t index,
    const int64_t message_count,
    const int64_t qos_depth)
  : Node(
      "ceiling_source_" + std::to_string(index),
      rclcpp::NodeOptions().use_intra_process_comms(true)),
    message_count_(message_count)
  {
    const auto qos = rclcpp::QoS(static_cast<size_t>(qos_depth));
    publisher_ = create_publisher<Int64>(
      "ceiling_flow_" + std::to_string(index), qos);
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    waitable_ = std::make_shared<AlwaysReadyWaitable>([this]() {PublishOnce();});
    get_node_waitables_interface()->add_waitable(waitable_, callback_group_);
  }

  void Kick()
  {
    waitable_->Trigger();
  }

  uint64_t Published() const
  {
    return published_.load(std::memory_order_relaxed);
  }

  uint64_t FirstPublishNs() const
  {
    return first_publish_ns_.load(std::memory_order_relaxed);
  }

  uint64_t LastPublishNs() const
  {
    return last_publish_ns_.load(std::memory_order_relaxed);
  }

  WaitableTraceCounts TraceCounts() const
  {
    return waitable_->TraceCounts();
  }

private:
  void PublishOnce()
  {
    const uint64_t previous = published_.load(std::memory_order_relaxed);
    if (previous >= static_cast<uint64_t>(message_count_)) {
      return;
    }

    const uint64_t publish_ns = NowNs();
    if (previous == 0) {
      first_publish_ns_.store(publish_ns, std::memory_order_release);
    }

    Int64 message;
    message.data = static_cast<int64_t>(publish_ns);
    publisher_->publish(message);
    last_publish_ns_.store(publish_ns, std::memory_order_release);

    const uint64_t published =
      published_.fetch_add(1, std::memory_order_acq_rel) + 1;
    if (published < static_cast<uint64_t>(message_count_)) {
      waitable_->Trigger();
    }
  }

  const int64_t message_count_;
  rclcpp::Publisher<Int64>::SharedPtr publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<AlwaysReadyWaitable> waitable_;
  std::atomic<uint64_t> published_{0};
  std::atomic<uint64_t> first_publish_ns_{0};
  std::atomic<uint64_t> last_publish_ns_{0};
};

class SinkNode : public rclcpp::Node
{
public:
  SinkNode(
    const size_t index,
    const int64_t message_count,
    const int64_t qos_depth)
  : Node(
      "ceiling_sink_" + std::to_string(index),
      rclcpp::NodeOptions().use_intra_process_comms(true)),
    message_count_(message_count)
  {
    const auto qos = rclcpp::QoS(static_cast<size_t>(qos_depth));
    subscription_ = create_subscription<Int64>(
      "ceiling_flow_" + std::to_string(index), qos,
      [this](const Int64 & message) {Receive(message);});
  }

  bool Complete() const
  {
    return complete_.load(std::memory_order_acquire);
  }

  uint64_t Received() const
  {
    return received_.load(std::memory_order_relaxed);
  }

  uint64_t FirstReceiveNs() const
  {
    return first_receive_ns_.load(std::memory_order_relaxed);
  }

  uint64_t LastReceiveNs() const
  {
    return last_receive_ns_.load(std::memory_order_relaxed);
  }

  uint64_t TotalLatencyNs() const
  {
    return total_latency_ns_.load(std::memory_order_relaxed);
  }

  uint64_t MinLatencyNs() const
  {
    return min_latency_ns_.load(std::memory_order_relaxed);
  }

  uint64_t MaxLatencyNs() const
  {
    return max_latency_ns_.load(std::memory_order_relaxed);
  }

private:
  static void UpdateMin(std::atomic<uint64_t> & target, const uint64_t value)
  {
    uint64_t current = target.load(std::memory_order_relaxed);
    while (value < current &&
      !target.compare_exchange_weak(current, value, std::memory_order_relaxed))
    {
    }
  }

  static void UpdateMax(std::atomic<uint64_t> & target, const uint64_t value)
  {
    uint64_t current = target.load(std::memory_order_relaxed);
    while (value > current &&
      !target.compare_exchange_weak(current, value, std::memory_order_relaxed))
    {
    }
  }

  void Receive(const Int64 & message)
  {
    const uint64_t receive_ns = NowNs();
    const uint64_t previous = received_.fetch_add(1, std::memory_order_acq_rel);
    if (previous == 0) {
      first_receive_ns_.store(receive_ns, std::memory_order_release);
    }
    last_receive_ns_.store(receive_ns, std::memory_order_release);

    const uint64_t latency_ns =
      receive_ns - static_cast<uint64_t>(message.data);
    total_latency_ns_.fetch_add(latency_ns, std::memory_order_relaxed);
    UpdateMin(min_latency_ns_, latency_ns);
    UpdateMax(max_latency_ns_, latency_ns);

    if (previous + 1 >= static_cast<uint64_t>(message_count_)) {
      complete_.store(true, std::memory_order_release);
    }
  }

  const int64_t message_count_;
  rclcpp::Subscription<Int64>::SharedPtr subscription_;
  std::atomic<uint64_t> received_{0};
  std::atomic<uint64_t> first_receive_ns_{0};
  std::atomic<uint64_t> last_receive_ns_{0};
  std::atomic<uint64_t> total_latency_ns_{0};
  std::atomic<uint64_t> min_latency_ns_{UINT64_MAX};
  std::atomic<uint64_t> max_latency_ns_{0};
  std::atomic<bool> complete_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    const int64_t thread_count =
      EnvInt("ROS2_FRAMEWORK_PERF_CEILING_THREADS", 1);
    const int64_t flow_count =
      EnvInt("ROS2_FRAMEWORK_PERF_CEILING_FLOWS", 1);
    const int64_t messages_per_flow =
      EnvInt("ROS2_FRAMEWORK_PERF_CEILING_MESSAGES", 100000);
    const int64_t qos_depth =
      EnvInt("ROS2_FRAMEWORK_PERF_CEILING_QOS_DEPTH", messages_per_flow);
    const int64_t timeout_seconds =
      EnvInt("ROS2_FRAMEWORK_PERF_CEILING_TIMEOUT_SEC", 60);

    if (thread_count <= 0 || flow_count <= 0 || messages_per_flow <= 0 ||
      qos_depth <= 0 || timeout_seconds <= 0)
    {
      throw std::runtime_error(
              "threads, flows, messages, QoS depth, and timeout must be greater than zero");
    }

    auto executor = std::make_shared<rclcpp::executors::EventsCBGExecutor>(
      rclcpp::ExecutorOptions{}, static_cast<size_t>(thread_count));
    std::vector<std::shared_ptr<SourceNode>> sources;
    std::vector<std::shared_ptr<SinkNode>> sinks;
    sources.reserve(static_cast<size_t>(flow_count));
    sinks.reserve(static_cast<size_t>(flow_count));

    for (int64_t index = 0; index < flow_count; ++index) {
      auto sink = std::make_shared<SinkNode>(
        static_cast<size_t>(index), messages_per_flow, qos_depth);
      auto source = std::make_shared<SourceNode>(
        static_cast<size_t>(index), messages_per_flow, qos_depth);
      executor->add_node(sink);
      executor->add_node(source);
      sinks.push_back(std::move(sink));
      sources.push_back(std::move(source));
    }

    for (const auto & source : sources) {
      source->Kick();
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
              sinks.begin(), sinks.end(),
            [](const auto & sink) {return sink->Complete();}))
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

    uint64_t published = 0;
    uint64_t received = 0;
    uint64_t first_publish_ns = UINT64_MAX;
    uint64_t last_publish_ns = 0;
    uint64_t last_receive_ns = 0;
    uint64_t total_latency_ns = 0;
    uint64_t min_latency_ns = UINT64_MAX;
    uint64_t max_latency_ns = 0;
    bool complete = true;
    WaitableTraceCounts trace_counts;

    for (const auto & source : sources) {
      published += source->Published();
      if (source->FirstPublishNs() > 0) {
        first_publish_ns = std::min(first_publish_ns, source->FirstPublishNs());
      }
      last_publish_ns = std::max(last_publish_ns, source->LastPublishNs());
      Accumulate(trace_counts, source->TraceCounts());
    }

    for (const auto & sink : sinks) {
      received += sink->Received();
      last_receive_ns = std::max(last_receive_ns, sink->LastReceiveNs());
      total_latency_ns += sink->TotalLatencyNs();
      min_latency_ns = std::min(min_latency_ns, sink->MinLatencyNs());
      max_latency_ns = std::max(max_latency_ns, sink->MaxLatencyNs());
      complete = complete && sink->Complete();
    }

    const double source_duration_seconds =
      first_publish_ns != UINT64_MAX && last_publish_ns > first_publish_ns ?
      static_cast<double>(last_publish_ns - first_publish_ns) / 1e9 : 0.0;
    const double end_to_end_duration_seconds =
      first_publish_ns != UINT64_MAX && last_receive_ns > first_publish_ns ?
      static_cast<double>(last_receive_ns - first_publish_ns) / 1e9 : 0.0;
    const double source_throughput =
      source_duration_seconds > 0.0 ?
      static_cast<double>(published) / source_duration_seconds : 0.0;
    const double throughput =
      end_to_end_duration_seconds > 0.0 ?
      static_cast<double>(received) / end_to_end_duration_seconds : 0.0;
    const double average_latency_ns =
      received > 0 ?
      static_cast<double>(total_latency_ns) / static_cast<double>(received) : 0.0;
    const double min_latency_us =
      min_latency_ns == UINT64_MAX ?
      0.0 : static_cast<double>(min_latency_ns) / 1e3;
    const double drain_lag_ms =
      last_receive_ns > last_publish_ns ?
      static_cast<double>(last_receive_ns - last_publish_ns) / 1e6 : 0.0;
    const uint64_t expected =
      static_cast<uint64_t>(messages_per_flow) *
      static_cast<uint64_t>(flow_count);
    const bool waitable_invariant =
      trace_counts.execute == published;
    complete = complete && published == expected && received == expected &&
      waitable_invariant;

    std::ostringstream output;
    output << "{"
           << "\"benchmark\":\"int64_message_passing\","
           << "\"ready_mode\":\"waitable\","
           << "\"executor\":\"events_cbg\","
           << "\"build_type\":\""
           << JsonEscape(ROS2_FRAMEWORK_PERF_BUILD_TYPE) << "\","
           << "\"threads\":" << thread_count << ","
           << "\"flows\":" << flow_count << ","
           << "\"messages_per_flow\":" << messages_per_flow << ","
           << "\"expected_messages\":" << expected << ","
           << "\"published_messages\":" << published << ","
           << "\"received_messages\":" << received << ","
           << "\"complete\":" << (complete ? "true" : "false") << ","
           << "\"waitable_invariant\":"
           << (waitable_invariant ? "true" : "false") << ","
           << "\"waitable_trigger_count\":" << trace_counts.trigger << ","
           << "\"waitable_on_ready_callback_count\":"
           << trace_counts.on_ready_callback << ","
           << "\"waitable_take_data_by_entity_id_count\":"
           << trace_counts.take_data_by_entity_id << ","
           << "\"waitable_execute_count\":" << trace_counts.execute << ","
           << "\"source_publish_msg_s\":" << source_throughput << ","
           << "\"throughput_msg_s\":" << throughput << ","
           << "\"avg_latency_us\":" << average_latency_ns / 1e3 << ","
           << "\"min_latency_us\":" << min_latency_us << ","
           << "\"max_latency_us\":"
           << static_cast<double>(max_latency_ns) / 1e3 << ","
           << "\"drain_lag_ms\":" << drain_lag_ms << ","
           << "\"source_duration_s\":" << source_duration_seconds << ","
           << "\"duration_s\":" << end_to_end_duration_seconds
           << "}";

    std::cout << output.str() << std::endl;
    rclcpp::shutdown();
    return complete ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << "{\"benchmark\":\"int64_message_passing\","
              << "\"complete\":false,\"error\":\""
              << JsonEscape(error.what()) << "\"}" << std::endl;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 1;
}
