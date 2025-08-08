# ros2_framework_perf

This package provides a benchmarking framework for measuring message processing performance
of ROS 2 application graphs. It launches configurable EmitterNode instances within
a composable node container to simulate application workloads and measure performance metrics.

This package is inspired by the [ros2_performance](https://github.com/irobot-ros/ros2-performance) package from iRobot. In this framework, we use launch files with composable nodes instead which facilitates measuring more complex pipelines including multi-process flows. Our measurements also include the end-to-end message journey include the executor latencies, middleware transport time, and more from initial publish through new messages published as a result to the final subscriber callback invocation.

## Key Features

- Launches multiple `EmitterNode` instances based on YAML configuration
- Manages node lifecycle transitions (configure, activate, deactivate, shutdown)
- Collects message publishing/receiving data with timestamps
- Monitors system resource usage (CPU, memory, page faults, context switches)
- Supports perf profiling for detailed performance analysis
- Generates JSON reports with raw message data

## What it Measures

The framework reads node configurations from `config/benchmark_graph.yaml` and measures:
- Message throughput and latency across the application graph
- System resource consumption during message processing
- Node lifecycle transition timing
- Inter-node communication patterns and bottlenecks

## Output Files

The benchmark generates:
- Raw message data with timestamps and metadata
- Resource usage statistics (initial, final, delta)
- Perf profiling reports (if enabled)
- Symlinks to latest results for easy access


# Running the benchmark
## Environment
Ubuntu 24.04 Noble with ROS 2 Rolling installed OR within the official Docker container launched as below:

**Launch container (optional)**
```bash
docker run -it --rm --cap-add=SYS_NICE ros:rolling bash
```

## Setup
**Install prerequisites**
```bash
apt-get update && apt-get install -y python3-tqdm python3-matplotlib
```

**Clone repository**
```bash
mkdir -p /workspaces/ros_ws/src && cd /workspaces/ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/ros2_framework_perf.git
cd /workspaces/ros_ws
```

**Prepare environment**
```bash
source /opt/ros/rolling/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
```

**Build**
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to-regex ros2_framework_perf* --event-handlers console_direct+ --symlink-install
source install/setup.bash
```
## Benchmark
Running the following script will read the configuration from `config/benchmark_graph.yaml`. Modify this file or change the symlink to another file to edit parameters.

**Launch**
```bash
launch_test src/ros2_framework_perf/scripts/benchmark_launch.py
```

## Analysis
The output file will be symlinked as `raw_message_data_latest.json` and contain all of the raw data. You can write your own scripts to parse this data or use the `analyze_message_tree.py` script which will generate useful graphs for you.

**Analyze results**
```bash
python3 src/ros2_framework_perf/scripts/analyze_message_tree.py -n tensor_inference_node -p /tensor_encoder_output -s raw_message_data_latest.json
```

Review `timestamp_deltas_tensor_inference_node__tensor_encoder_output.png` or any of the other outputs.


You can also visualize the launch graph itself as a DOT file from the benchmark results.

**View launch graph**
```bash
python3 src/ros2_framework_perf/scripts/generate_graph_viz.py raw_message_data_latest.json
```

# Troubleshooting
**Fix permissions for `chrt`**

If you see the following log message, you need to run with `root` privileges or launch the Docker container with `--cap-add=SYS_NICE`.

```bash
[chrt-1] chrt: failed to set pid 0's policy: Operation not permitted
```
