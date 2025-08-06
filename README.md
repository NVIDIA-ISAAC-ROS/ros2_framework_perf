# ros2_framework_perf

Measures performance of executor and transport in ROS 2 framework using synthetic data and tracing.

**Launch container**
```bash
docker run -it --rm --cap-add=SYS_NICE ros:rolling bash
```

**Install prerequisites**
```bash
apt-get update && apt-get install -y python3-tqdm python3-matplotlib
```

**Clone repository**
```bash
mkdir -p /workspaces/ros_ws/src && cd /workspaces/ros_ws/src
# git clone https://github.com/NVIDIA-ISAAC-ROS/ros2_framework_perf.git
git clone https://gitlab-master.nvidia.com/isaac_ros/ros2_framework_perf.git
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

**Launch**
```bash
launch_test src/ros2_framework_perf/scripts/benchmark_launch.py
```

**Analyze results**
```bash
python3 src/ros2_framework_perf/scripts/analyze_message_tree.py -n tensor_inference_node -p /tensor_encoder_output -s raw_message_data_latest.json
```

Review `timestamp_deltas_tensor_inference_node__tensor_encoder_output.png` or any of the other outputs.

# Troubleshooting
**Fix permissions for `chrt`**

If you see the following log message, you need to run with `root` privileges or launch the Docker container with `--cap-add=SYS_NICE`.

```bash
[chrt-1] chrt: failed to set pid 0's policy: Operation not permitted
```
