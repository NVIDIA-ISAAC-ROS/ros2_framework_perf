# ROS 2 Framework-Ceiling Microbenchmarks

The framework-ceiling microbenchmarks complement the configurable application-graph benchmark.
They intentionally remove graph processing and rich message metadata to isolate two lower-level
limits:

1. Executor-dispatched, intra-process message throughput and callback latency using
   `std_msgs/msg/Int64`.
2. Executor dispatch throughput with no message transport or application work.

Both benchmarks use continuously ready, guard-condition-backed `rclcpp::Waitable` instances.

## Why a separate microbenchmark path?

The configurable `EmitterNode` graph benchmark measures application-like message journeys. Its
custom payload metadata, lifecycle nodes, composable container, graph stages, and instrumentation
are useful parts of that measurement.

The ceiling benchmarks answer a narrower question: when application work is nearly zero, how fast
can an executor dispatch ready work and move a tiny intra-process message? Keeping these paths
separate makes the source of a throughput limit easier to identify.

The benchmarks do not modify ROS 2. They use public `rclcpp` APIs, including
`EventsCBGExecutor`, `Waitable`, and `GuardCondition`.

## Benchmarks

### Int64 message passing

Each flow consists of one source node and one sink node in the same process:

```text
always-ready source waitable -> publish Int64 timestamp -> subscription callback
```

The source waitable publishes one message per executor dispatch and retriggers itself until the
configured message count is reached. The sink computes latency from the timestamp sampled
immediately before `publish()` to the beginning of the subscription callback.

The primary metrics are:

- `source_publish_msg_s`
- `throughput_msg_s`
- average, minimum, and maximum callback latency
- source-to-sink drain lag

### Scheduler dispatch

Each scheduler node owns one always-ready waitable. Every executor dispatch increments a counter
and retriggers the waitable until the configured operation count is reached. There is no publisher,
subscription, or payload.

The primary metric is `throughput_ops_s`.

### Executor-dispatch invariant

Every result includes waitable trace counters. A successful run requires:

```text
waitable_execute_count == published_messages
```

for message passing, or:

```text
waitable_execute_count == total_operations
```

for scheduler dispatch. This confirms the measured work ran from executor-dispatched
`Waitable::execute()` calls rather than directly from the initial trigger or guard-condition
callback.

## Run the matrix

Build the workspace as described in the repository README, then:

```bash
source /opt/ros/rolling/setup.bash
source install/setup.bash

python3 src/ros2_framework_perf/scripts/run_ceiling_benchmarks.py \
  --config src/ros2_framework_perf/config/framework_ceiling.yaml
```

The default YAML runs five repetitions of each matrix cell with `EventsCBGExecutor`. Edit or copy
the YAML to change thread counts, flow/operator counts, operation counts, or repetitions.

Validate a configuration without executing it:

```bash
python3 src/ros2_framework_perf/scripts/run_ceiling_benchmarks.py \
  --config src/ros2_framework_perf/config/framework_ceiling.yaml \
  --dry-run
```

Run only one benchmark family:

```bash
python3 src/ros2_framework_perf/scripts/run_ceiling_benchmarks.py \
  --config src/ros2_framework_perf/config/framework_ceiling.yaml \
  --benchmark message_passing
```

## Summarize results

The runner writes one JSON file per repetition and matrix cell. Summarize repeated runs and validate
result completeness and executor-dispatch invariants with:

```bash
python3 src/ros2_framework_perf/scripts/summarize_ceiling_results.py \
  ceiling_benchmark_results/<timestamp>
```

The summarizer writes `summary.json` and `summary.csv`. It exits nonzero if any run is incomplete,
an invariant fails, or a required metric is missing.

## Methodology and limitations

- These are saturation tests. Sources remain continuously ready and do not represent an
  application-selected publish rate.
- The message-passing benchmark uses an 8-byte `Int64` timestamp and intra-process communication.
  It does not measure DDS serialization, networking, cross-process IPC, large payload transfer, GPU
  inference, or application callback work.
- The scheduler benchmark measures a minimal counter operation. It is useful for executor
  comparison, not as an application throughput prediction.
- More threads do not guarantee higher throughput. Queue synchronization, cache contention, and
  competition between continuously ready source work and sink callbacks can dominate.
- Latency begins immediately before `publish()`. Source scheduling delay before that timestamp is
  intentionally outside the latency interval.
- Performance results depend on the ROS distribution, `rclcpp` version, build type, compiler,
  hardware, kernel, CPU configuration, and system load. Record these inputs and compare
  repeated-run medians rather than isolated runs.
- Rolling changes continuously. For reproducible published results, record the container image
  digest and package versions used for a run.
