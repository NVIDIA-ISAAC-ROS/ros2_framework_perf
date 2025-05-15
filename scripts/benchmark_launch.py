import os
import time
import unittest
import json
import yaml
from datetime import datetime
from pathlib import Path
import resource
import psutil
import signal
import subprocess
import threading

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import pytest
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from launch_testing.actions import ReadyToTest
from ros2_framework_perf_interfaces.msg import MessageWithPayload
from ros2_framework_perf_interfaces.srv import GetPublishedMessages
from tqdm import tqdm


def load_node_configs():
    """Load node configurations from YAML file."""
    # Get the package directory
    package_dir = Path(__file__).parent.parent
    config_path = package_dir / 'config' / 'benchmark_graph.yaml'

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def create_composable_node(node_config, global_settings):
    """Create a ComposableNode from node configuration."""
    # Use node-specific setting if available, otherwise use global setting
    use_intra_process = node_config.get('use_intra_process', global_settings.get('use_intra_process', True))

    return ComposableNode(
        package="ros2_framework_perf",
        plugin="ros2_framework_perf::EmitterNode",
        name=node_config['name'],
        parameters=[node_config['config']],
        extra_arguments=[{
            'use_intra_process_comms': use_intra_process
        }]
    )


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description for testing EmitterNode."""
    # Load node configurations
    config = load_node_configs()
    node_configs = config['nodes']
    global_settings = config.get('node_settings', {})

    # Create composable nodes from configurations
    composable_nodes = []
    for node_name, node_config in node_configs.items():
        composable_node = create_composable_node(node_config, global_settings)
        composable_nodes.append(composable_node)
        print(f"{node_name} configuration:")
        print(node_config['config']['yaml_config'])

    # Create container with all nodes
    container_executable = config.get('container_executable', 'component_container')
    container = ComposableNodeContainer(
        name='emitter_container',
        namespace='',
        package='rclcpp_components',
        executable=container_executable,
        composable_node_descriptions=composable_nodes,
        output='screen',
        prefix='chrt -r 99'
    )

    return LaunchDescription([
        container,
        ReadyToTest()
    ])


class LifecycleTestNode(Node):
    def __init__(self, node_names):
        super().__init__('lifecycle_test_node')
        self.node_names = node_names
        self.lifecycle_clients = {}

        # Create lifecycle clients for each node
        for node_name in node_names:
            self.lifecycle_clients[node_name] = self.create_client(
                ChangeState,
                f'/{node_name}/change_state'
            )

    def _call_lifecycle_service(self, node_name, transition_id):
        """Helper method to call lifecycle service for a specific node."""
        if node_name not in self.lifecycle_clients:
            self.get_logger().error(f'No lifecycle client found for node: {node_name}')
            return False

        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.lifecycle_clients[node_name].call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def configure_nodes(self):
        """Configure all nodes."""
        results = []
        for node_name in self.node_names:
            result = self._call_lifecycle_service(node_name, Transition.TRANSITION_CONFIGURE)
            results.append(result)
        return all(results)

    def activate_nodes(self):
        """Activate all nodes."""
        results = []
        for node_name in self.node_names:
            print(f"Activating node: {node_name}")
            result = self._call_lifecycle_service(node_name, Transition.TRANSITION_ACTIVATE)
            print(f"Activation result for {node_name}: {result}")
            results.append(result)
        return all(results)

    def deactivate_nodes(self):
        """Deactivate all nodes."""
        results = []
        for node_name in self.node_names:
            result = self._call_lifecycle_service(node_name, Transition.TRANSITION_DEACTIVATE)
            results.append(result)
        return all(results)

    def shutdown_nodes(self):
        """Shutdown all nodes."""
        results = []
        for node_name in self.node_names:
            result = self._call_lifecycle_service(node_name, Transition.TRANSITION_INACTIVE_SHUTDOWN)
            results.append(result)
        return all(results)


class TestEmitterNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        print("\n=== Setting up Test Class ===")
        rclpy.init()
        cls.node = Node('test_emitter')

        # Load node names from configuration
        config = load_node_configs()
        node_configs = config['nodes']
        cls.node_names = list(node_configs.keys())
        cls.profiling_config = config['profiling_config']
        cls.perf_config = config.get('perf_config', {})
        cls.perf_process = None
        cls.perf_output_file = None

        cls.lifecycle_node = LifecycleTestNode(cls.node_names)
        cls.received_messages = []

        # Initialize container process tracking
        cls.container_pid = None
        cls.initial_rusage = None

        # Wait for the nodes to be ready
        time.sleep(cls.profiling_config['setup_wait_time'])

        # Configure the nodes
        print("Configuring nodes")
        cls.lifecycle_node.configure_nodes()
        print("Configured nodes")

    def start_perf_profiling(self, pid):
        """Start perf profiling for the given process ID."""
        if not self.perf_config.get('enabled', False):
            return

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.perf_output_file = f"perf_data_{timestamp}.data"

        # Build perf command with configured events
        events = self.perf_config.get('events', ['cpu-clock', 'page-faults', 'context-switches'])
        event_args = [f'-e {event}' for event in events]

        # Add sampling frequency if specified
        freq = self.perf_config.get('frequency', 1000)
        freq_args = [f'-F {freq}']

        # Add call graph depth if specified
        call_graph = self.perf_config.get('call_graph_depth', 0)
        call_graph_args = [f'--call-graph=dwarf,{call_graph}'] if call_graph > 0 else []

        # Build the full command
        cmd = [
            'sudo', 'perf', 'record',
            *event_args,
            *freq_args,
            *call_graph_args,
            '-p', str(pid),
            '-o', self.perf_output_file,
            '--no-buildid',  # Don't record build IDs
            '--no-call-graph' if call_graph == 0 else '',  # Disable call graph if depth is 0
            '--timestamp'  # Add timestamps to events
        ]

        # Remove empty strings from command
        cmd = [arg for arg in cmd if arg]

        print(f"\nStarting perf profiling with command: {' '.join(cmd)}")
        try:
            self.perf_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group
            )
            # Give perf a moment to start
            time.sleep(1)
            if self.perf_process.poll() is not None:
                # Process ended immediately
                stdout, stderr = self.perf_process.communicate()
                print(f"Perf process failed to start. stdout: {stdout.decode()}")
                print(f"stderr: {stderr.decode()}")
                self.perf_process = None
        except Exception as e:
            print(f"Error starting perf: {e}")
            self.perf_process = None

    def stop_perf_profiling(self):
        """Stop perf profiling and generate report."""
        if not self.perf_process:
            return

        print("\nStopping perf profiling...")
        try:
            # Send SIGINT to the process group
            os.killpg(os.getpgid(self.perf_process.pid), signal.SIGINT)

            # Wait for the process to finish
            try:
                self.perf_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("Perf process did not terminate gracefully, forcing...")
                os.killpg(os.getpgid(self.perf_process.pid), signal.SIGKILL)
                self.perf_process.wait()

            if self.perf_output_file and os.path.exists(self.perf_output_file):
                # Generate report
                report_file = f"{self.perf_output_file}.report"
                cmd = [
                    'sudo', 'perf', 'report',
                    '-i', self.perf_output_file,
                    '--stdio',
                    '--no-children'  # Don't show children in the report
                ]

                try:
                    result = subprocess.run(
                        cmd,
                        capture_output=True,
                        text=True,
                        check=True
                    )

                    # Save the report
                    with open(report_file, 'w') as f:
                        f.write(result.stdout)
                    print(f"Perf report saved to {report_file}")

                    # Create symlink to latest
                    latest_report = "perf_report_latest.txt"
                    if os.path.exists(latest_report):
                        os.remove(latest_report)
                    os.symlink(report_file, latest_report)

                except subprocess.CalledProcessError as e:
                    print(f"Error generating perf report: {e}")
                    print(f"stderr: {e.stderr}")
            else:
                print(f"Warning: Perf data file not found: {self.perf_output_file}")

        except Exception as e:
            print(f"Error stopping perf profiling: {e}")
        finally:
            self.perf_process = None

    def setUp(self):
        # Clear received messages
        self.received_messages = []

        # Load configuration
        config = load_node_configs()
        self.container_executable = config.get('container_executable', 'component_container')
        print(f"\nLooking for container process with executable: {self.container_executable}")

        # Wait for the nodes to be ready
        time.sleep(self.profiling_config['setup_wait_time'])

        # Find and verify container process before activation
        print("\nWaiting for container process to start...")
        max_wait_time = 10  # Maximum time to wait for container in seconds
        start_time = time.time()
        container_proc = None

        while time.time() - start_time < max_wait_time:
            container_proc = self.find_container_process()
            if container_proc:
                print(f"Found container process (PID: {container_proc.pid})")
                break
            time.sleep(0.5)

        if not container_proc:
            print(f"Warning: Could not find container process with executable: {self.container_executable}")
            return

        # Start perf profiling if enabled
        self.start_perf_profiling(container_proc.pid)

        # Get initial stats before activation
        self.initial_usage = self.get_container_resource_usage()
        if self.initial_usage:
            print("\nInitial container resource usage (before activation):")
            print(f"PID: {self.initial_usage['pid']}")
            print(f"Page faults (major/minor): {self.initial_usage['page_faults']['major']}/{self.initial_usage['page_faults']['minor']}")
            print(f"Memory RSS: {self.initial_usage['memory']['rss'] / 1024 / 1024:.2f} MB")
            print(f"Threads: {self.initial_usage['threads']}")

        # Activate the nodes
        print("Activating nodes...")
        activation_result = self.lifecycle_node.activate_nodes()
        print(f"Node activation result: {activation_result}")
        print("Activated nodes")

        # Create service clients for each node
        self.message_clients = {}
        service_name_by_node_name = {}

        for node_name in self.node_names:
            # Match exactly how the C++ code constructs the service name: "/" + get_name() + "/get_published_messages"
            service_name = f"/{node_name}/get_published_messages"  # No extra spaces, exact string construction
            print(f"\nTrying to connect to service: {service_name}")
            service_name_by_node_name[node_name] = service_name
            self.message_clients[node_name] = self.node.create_client(
                GetPublishedMessages,
                service_name
            )

        # Wait for all services to be available
        print("\nWaiting for services to be available...")
        start_time = time.time()
        services_ready = {node_name: False for node_name in self.node_names}

        while not all(services_ready.values()) and (time.time() - start_time) < self.profiling_config['service_wait_timeout']:
            # Check each service in round-robin fashion
            for node_name, client in self.message_clients.items():
                if not services_ready[node_name]:
                    try:
                        # Use a very short timeout to check service availability
                        services_ready[node_name] = client.wait_for_service(timeout_sec=0.1)
                        if services_ready[node_name]:
                            print(f"Service {service_name_by_node_name[node_name]} is ready!")
                    except Exception as e:
                        print(f"Error checking {node_name} service: {str(e)}")

            # Print status of services not yet ready
            not_ready = [node_name for node_name, ready in services_ready.items() if not ready]
            if not_ready:
                print(f"\nServices not ready after {time.time() - start_time:.1f}s:")
                for node_name in not_ready:
                    print(f"  {service_name_by_node_name[node_name]}")
                    # Print service type for debugging
                    for service, types in self.node.get_service_names_and_types():
                        if service == service_name_by_node_name[node_name]:
                            print(f"    Type: {types}")

            # Small sleep to prevent tight loop
            time.sleep(0.1)

        # Final status check
        if all(services_ready.values()):
            print("\nAll services are ready!")
        else:
            print("\nSome services failed to become ready:")
            for node_name, ready in services_ready.items():
                if not ready:
                    print(f"  {service_name_by_node_name[node_name]}")
                    # Print service type for debugging
                    for service, types in self.node.get_service_names_and_types():
                        if service == service_name_by_node_name[node_name]:
                            print(f"    Type: {types}")

    def tearDown(self):
        # Stop perf profiling
        self.stop_perf_profiling()

        # Destroy all message clients
        for client in self.message_clients.values():
            self.node.destroy_client(client)
        self.message_clients.clear()

        self.lifecycle_node.shutdown_nodes()
        print("Shutdown nodes")


    def collect_message_data(self, published_messages_by_node):
        """Collect message data from all nodes.

        Args:
            published_messages_by_node: Dictionary to store message data
        """
        # Collect messages from all nodes
        for node_name in self.node_names:
            print(f"Collecting messages from {node_name}...")
            self.assertTrue(
                self.message_clients[node_name].wait_for_service(timeout_sec=5.0),
                f"get_published_messages service not available for {node_name}"
            )

            # Call the service
            request = GetPublishedMessages.Request()
            future = self.message_clients[node_name].call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            # Get the response
            response = future.result()
            self.assertTrue(response.success, f"Service call failed for {node_name}")

            # Store published messages for this node
            published_messages_by_node[node_name] = {
                'published_messages': response.published_messages,
                'published_topics': response.published_topic_names,
                'received_messages': response.received_message_timestamps,
                'received_topics': response.received_topic_names,
                'lifecycle_transitions': response.lifecycle_transitions
            }

    def write_raw_message_data(self, published_messages_by_node, output_file, resource_usage=None):
        """Write raw message data to a JSON file for further processing.

        Args:
            published_messages_by_node: Dictionary containing message data
            output_file: File object to write JSON data to
            resource_usage: Dictionary containing resource usage data
        """
        # Convert the data to a serializable format
        serializable_data = {}

        for node_name, node_data in published_messages_by_node.items():
            serializable_data[node_name] = {
                'published_messages': [
                    {
                        'identifier': msg.identifier,
                        'publish_timestamp': {
                            'sec': msg.publish_timestamp.sec,
                            'nanosec': msg.publish_timestamp.nanosec
                        },
                        'parent_messages': list(msg.parent_messages),
                        'topic_name': msg.topic_name
                    }
                    for msg in node_data['published_messages']
                ],
                'published_topics': list(node_data['published_topics']),
                'received_messages': [
                    {
                        'topic': topic,
                        'messages': [
                            {
                                'identifier': msg_id,
                                'timestamp': {
                                    'sec': timestamps.timestamps[j].sec,
                                    'nanosec': timestamps.timestamps[j].nanosec
                                }
                            }
                            for j, msg_id in enumerate(timestamps.message_identifiers)
                        ]
                    }
                    for i, (topic, timestamps) in enumerate(zip(
                        node_data['received_topics'],
                        node_data['received_messages']
                    ))
                ],
                'lifecycle_transitions': [
                    {
                        'state_id': t.state_id,
                        'state_label': t.state_label,
                        'transition_time': {
                            'sec': t.transition_time.sec,
                            'nanosec': t.transition_time.nanosec
                        }
                    }
                    for t in node_data['lifecycle_transitions']
                ]
            }

        # Load benchmark configuration
        config = load_node_configs()

        # Add metadata
        metadata = {
            'timestamp': datetime.now().isoformat(),
            'node_names': self.node_names,
            'profiling_config': self.profiling_config,
            'benchmark_config': config  # Include the full benchmark configuration
        }

        # Combine all data into a single JSON object
        complete_data = {
            'metadata': metadata,
            'message_data': serializable_data
        }

        # Add resource usage if provided
        if resource_usage:
            complete_data['resource_usage'] = resource_usage

        # Write to file
        json.dump(complete_data, output_file, indent=2)

    def find_container_process(self):
        """Find the container process by looking for the configured executable name."""
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if self.container_executable in ' '.join(proc.info['cmdline'] or []):
                    return proc
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return None

    def get_container_resource_usage(self):
        """Get resource usage statistics for the container process."""
        container_proc = self.find_container_process()
        if not container_proc:
            print("Warning: Container process not found")
            return None

        try:
            # Get process stats
            with open(f"/proc/{container_proc.pid}/stat", 'r') as f:
                stat = f.read().split()

            # Get process status
            with open(f"/proc/{container_proc.pid}/status", 'r') as f:
                status = dict(line.split(':', 1) for line in f if ':' in line)

            # Get memory info
            mem_info = container_proc.memory_info()

            return {
                'pid': container_proc.pid,
                'page_faults': {
                    'major': int(stat[11]),  # majflt
                    'minor': int(stat[9]),   # minflt
                },
                'memory': {
                    'rss': mem_info.rss,
                    'vms': mem_info.vms,
                },
                'cpu': {
                    'user_time': float(stat[13]) / os.sysconf('SC_CLK_TCK'),
                    'system_time': float(stat[14]) / os.sysconf('SC_CLK_TCK'),
                    'percent': container_proc.cpu_percent(interval=0.1)
                },
                'threads': int(status.get('Threads', '0').strip()),
                'voluntary_ctxt_switches': int(status.get('voluntary_ctxt_switches', '0').strip()),
                'nonvoluntary_ctxt_switches': int(status.get('nonvoluntary_ctxt_switches', '0').strip())
            }
        except (psutil.NoSuchProcess, FileNotFoundError, PermissionError) as e:
            print(f"Error getting container stats: {e}")
            return None

    def test_get_published_messages_service(self):
        """Test the get_published_messages service and write raw data to file."""
        # Wait for configured time to let the nodes publish messages
        wait_time = self.profiling_config['data_collection_time']
        step = 0.1
        steps = int(wait_time / step)
        for _ in tqdm(range(steps), desc='Data collection wait', unit='0.1s'):
            time.sleep(step)
        remainder = wait_time - steps * step
        if remainder > 0:
            time.sleep(remainder)

        # Get final resource usage
        final_usage = self.get_container_resource_usage()
        if final_usage:
            print("\nFinal container resource usage:")
            print(f"Page faults (major/minor): {final_usage['page_faults']['major']}/{final_usage['page_faults']['minor']}")
            print(f"Memory RSS: {final_usage['memory']['rss'] / 1024 / 1024:.2f} MB")
            print(f"CPU User Time: {final_usage['cpu']['user_time']:.2f}s")
            print(f"CPU System Time: {final_usage['cpu']['system_time']:.2f}s")
            print(f"Threads: {final_usage['threads']}")
            print(f"Context Switches (vol/nonvol): {final_usage['voluntary_ctxt_switches']}/{final_usage['nonvoluntary_ctxt_switches']}")

        self.lifecycle_node.deactivate_nodes()
        print("Deactivated nodes")

        # Dictionary to store published messages by node
        published_messages_by_node = {}

        # Collect message data from all nodes
        self.collect_message_data(published_messages_by_node)

        # Add resource usage to the output data
        resource_usage = {
            'initial': self.initial_usage,  # Store initial usage from setUp
            'final': final_usage,
            'delta': None
        }

        if self.initial_usage and final_usage:
            resource_usage['delta'] = {
                'page_faults': {
                    'major': final_usage['page_faults']['major'] - self.initial_usage['page_faults']['major'],
                    'minor': final_usage['page_faults']['minor'] - self.initial_usage['page_faults']['minor']
                },
                'memory': {
                    'rss_delta': final_usage['memory']['rss'] - self.initial_usage['memory']['rss'],
                    'vms_delta': final_usage['memory']['vms'] - self.initial_usage['memory']['vms']
                },
                'cpu': {
                    'user_time_delta': final_usage['cpu']['user_time'] - self.initial_usage['cpu']['user_time'],
                    'system_time_delta': final_usage['cpu']['system_time'] - self.initial_usage['cpu']['system_time']
                },
                'context_switches': {
                    'voluntary_delta': final_usage['voluntary_ctxt_switches'] - self.initial_usage['voluntary_ctxt_switches'],
                    'nonvoluntary_delta': final_usage['nonvoluntary_ctxt_switches'] - self.initial_usage['nonvoluntary_ctxt_switches']
                }
            }

        # Write raw message data to JSON file
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        raw_data_filename = f"raw_message_data_{timestamp}.json"
        with open(raw_data_filename, 'w') as raw_data_file:
            self.write_raw_message_data(published_messages_by_node, raw_data_file, resource_usage)
        print(f"Raw message data has been written to {raw_data_filename}")

        # Update symlink to latest
        symlink_name = "raw_message_data_latest.json"
        try:
            if os.path.islink(symlink_name) or os.path.exists(symlink_name):
                os.remove(symlink_name)
        except Exception as e:
            print(f"Warning: Could not remove old symlink {symlink_name}: {e}")
        try:
            os.symlink(raw_data_filename, symlink_name)
            print(f"Symlink {symlink_name} -> {raw_data_filename} created.")
        except Exception as e:
            print(f"Warning: Could not create symlink {symlink_name}: {e}")

def main():
    unittest.main()


if __name__ == '__main__':
    main()