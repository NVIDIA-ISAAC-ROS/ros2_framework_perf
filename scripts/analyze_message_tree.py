#!/usr/bin/env python3

import json
import argparse
from datetime import datetime
import time
from tqdm import tqdm
import sys
import yaml
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np


class TeeOutput:
    """Class to write output to both file and screen."""
    def __init__(self, file, screen=False):
        self.file = file
        self.screen = screen
        self.buffer = []

    def write(self, text):
        self.file.write(text)
        if self.screen:
            self.buffer.append(text)

    def flush_to_screen(self):
        """Flush buffered output to screen."""
        if self.screen:
            print(''.join(self.buffer), end='')
            self.buffer = []


def calculate_timestamp_deltas(node_data, topic_name, output, node_name, expected_frequency=None):
    """Calculate and print statistics for timestamp deltas between received messages.

    Args:
        node_data: Dictionary containing node message data
        topic_name: Name of the topic to analyze
        output: TeeOutput object to write to
        node_name: Name of the node being analyzed
        expected_frequency: Expected frequency in Hz from configuration
    """
    topic_index = None

    # Find the topic index
    for i, topic in enumerate(node_data['received_messages']):
        if topic['topic'] == topic_name:
            topic_index = i
            break

    if topic_index is None:
        output.write(f"\nNo {topic_name} messages found in node\n")
        return

    # Get timestamps for messages
    timestamps = node_data['received_messages'][topic_index]['messages']

    if len(timestamps) < 2:
        output.write("\nNot enough messages to calculate timestamp deltas\n")
        return

    # Calculate deltas between consecutive timestamps
    deltas = []
    timestamps_sec = []
    for i in range(1, len(timestamps)):
        prev_time = timestamps[i-1]['timestamp']['sec'] + timestamps[i-1]['timestamp']['nanosec'] * 1e-9
        curr_time = timestamps[i]['timestamp']['sec'] + timestamps[i]['timestamp']['nanosec'] * 1e-9
        delta = curr_time - prev_time
        deltas.append(delta)
        timestamps_sec.append(curr_time)  # Store the current timestamp

    # Find indices of min and max deltas
    min_index = deltas.index(min(deltas))
    max_index = deltas.index(max(deltas))
    total_deltas = len(deltas)

    output.write(f"\nMin delta at index {min_index + 1} of {total_deltas}\n")
    output.write(f"Max delta at index {max_index + 1} of {total_deltas}\n")

    # Calculate statistics
    sorted_deltas = sorted(deltas)
    min_delta = sorted_deltas[0]
    max_delta = sorted_deltas[-1]
    mean_delta = sum(sorted_deltas) / len(sorted_deltas)
    median_delta = sorted_deltas[len(sorted_deltas) // 2] if len(sorted_deltas) % 2 == 1 else \
                  (sorted_deltas[len(sorted_deltas) // 2 - 1] + sorted_deltas[len(sorted_deltas) // 2]) / 2

    # Calculate standard deviation
    variance = sum((x - mean_delta) ** 2 for x in sorted_deltas) / len(sorted_deltas)
    stddev = variance ** 0.5

    # Write statistics to file
    output.write(f"\nTimestamp Delta Statistics for {node_name} receiving {topic_name} messages:\n")
    if expected_frequency is not None:
        expected_delta = 1.0 / expected_frequency
        output.write(f"  Expected delta: {expected_delta:.6f} seconds (from {expected_frequency} Hz)\n")
        output.write(f"  Minimum delta: {min_delta:.6f} seconds (delta: |{abs(min_delta - expected_delta):.6f}|s)\n")
        output.write(f"  Maximum delta: {max_delta:.6f} seconds (delta: |{abs(max_delta - expected_delta):.6f}|s)\n")
        output.write(f"  Mean delta: {mean_delta:.6f} seconds (delta: |{abs(mean_delta - expected_delta):.6f}|s)\n")
        output.write(f"  Median delta: {median_delta:.6f} seconds (delta: |{abs(median_delta - expected_delta):.6f}|s)\n")
        output.write(f"  Standard deviation: {stddev:.6f} seconds\n")
    else:
        output.write(f"  Minimum delta: {min_delta:.6f} seconds\n")
        output.write(f"  Maximum delta: {max_delta:.6f} seconds\n")
        output.write(f"  Mean delta: {mean_delta:.6f} seconds\n")
        output.write(f"  Median delta: {median_delta:.6f} seconds\n")
        output.write(f"  Standard deviation: {stddev:.6f} seconds\n")

    # Create plot
    plt.figure(figsize=(12, 6))
    # Convert deltas to milliseconds
    deltas_ms = [delta * 1000 for delta in deltas]
    # Use message indices as x-axis
    message_indices = range(1, len(deltas) + 1)
    plt.plot(message_indices, deltas_ms, 'b-', label='Timestamp Delta')

    if expected_frequency is not None:
        expected_delta = 1.0 / expected_frequency
        expected_delta_ms = expected_delta * 1000
        plt.axhline(y=expected_delta_ms, color='r', linestyle='--', label=f'Expected Delta ({expected_frequency} Hz)')

    plt.xlabel('Message Index')
    plt.ylabel('Timestamp Delta (milliseconds)')
    plt.title(f'Timestamp Deltas for {node_name} receiving {topic_name}')
    plt.grid(True)
    plt.legend()

    # Save plot
    plot_filename = f"timestamp_deltas_{node_name}_{topic_name.replace('/', '_')}.png"
    plt.savefig(plot_filename)
    plt.close()

    output.write(f"\nPlot saved as: {plot_filename}\n")


def print_parent_chain(msg_id, node_name, message_data, receive_time_sec, indent="", output=None):
    # Search for the message in any node's published messages
    msg = None
    found_node = None
    for candidate_node, node_data in message_data.items():
        candidate_msgs = node_data['published_messages']
        msg = next((m for m in candidate_msgs if m['identifier'] == msg_id), None)
        if msg:
            found_node = candidate_node
            break
    if not msg:
        output.write(f"{indent}{node_name}: {msg_id} (not found in any node)\n")
        return
    pub_time = msg['publish_timestamp']['sec'] + msg['publish_timestamp']['nanosec'] * 1e-9
    latency = receive_time_sec - pub_time

    output.write(f"{indent}{found_node}: {msg['identifier']} | Publish: {msg['publish_timestamp']['sec']}.{msg['publish_timestamp']['nanosec']} | Latency from Actuator: {latency:.6f}s\n")
    if msg['parent_messages']:
        for parent_id in msg['parent_messages']:
            print_parent_chain(parent_id, found_node, message_data, receive_time_sec, indent + "    └── ", output)


def analyze_message_flow(message_data, output):
    """Analyze and write message flow between nodes to file."""
    output.write("\nMessage Flow Analysis:\n")
    for node_name, data in tqdm(message_data.items(), desc="Analyzing message flow", leave=False):
        output.write(f"\n{node_name}:\n")
        output.write("Published Messages:\n")
        for msg in data['published_messages']:
            output.write(f"  ID: {msg['identifier']}\n")
            output.write(f"  Timestamp: {msg['publish_timestamp']['sec']}.{msg['publish_timestamp']['nanosec']}\n")

        output.write("\nReceived Messages:\n")
        for topic_data in data['received_messages']:
            output.write(f"  Topic: {topic_data['topic']}\n")
            for msg in topic_data['messages']:
                output.write(f"    Message: {msg['identifier']}\n")
                output.write(f"    Timestamp: {msg['timestamp']['sec']}.{msg['timestamp']['nanosec']}\n")


def load_benchmark_config():
    """Load the benchmark graph configuration."""
    # Get the package directory
    package_dir = Path(__file__).parent.parent
    config_path = package_dir / 'config' / 'benchmark_graph.yaml'

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def find_expected_frequency(node_config, target_topic, config, visited_nodes=None):
    """Find the expected frequency of a publisher for a given topic.

    Args:
        node_config: Dictionary containing node configuration
        target_topic: Name of the topic to find frequency for
        config: Full benchmark configuration for tracing dependencies
        visited_nodes: Set of nodes already visited to prevent cycles

    Returns:
        float: Expected frequency in Hz, or None if not found
    """
    if visited_nodes is None:
        visited_nodes = set()

    node_name = node_config['name']
    if node_name in visited_nodes:
        return None
    visited_nodes.add(node_name)

    yaml_config = yaml.safe_load(node_config['config']['yaml_config'])

    # Check publishers
    if 'publishers' in yaml_config:
        for publisher in yaml_config['publishers']:
            if publisher['topic_name'] == target_topic:
                trigger = publisher['trigger']
                if trigger['type'] == 'timer':
                    # For timer triggers, check both direct frequency and timer group
                    if 'frequency' in trigger:
                        return trigger['frequency']
                    elif 'timer_group' in trigger:
                        # Check timer group frequency
                        for timer_group in yaml_config['timer_groups']:
                            if timer_group['name'] == trigger['timer_group']:
                                return timer_group['frequency']
                elif trigger['type'] == 'message_received':
                    # For message_received triggers, find the direct publisher
                    for source_topic in trigger['topics']:
                        # Find the node that publishes to this topic
                        for source_node_name, source_node_config in config['nodes'].items():
                            source_yaml = yaml.safe_load(source_node_config['config']['yaml_config'])
                            if 'publishers' in source_yaml:
                                for source_pub in source_yaml['publishers']:
                                    if source_pub['topic_name'] == source_topic:
                                        # This is a direct publisher, get its frequency
                                        source_trigger = source_pub['trigger']
                                        if source_trigger['type'] == 'timer':
                                            if 'frequency' in source_trigger:
                                                return source_trigger['frequency']
                                            elif 'timer_group' in source_trigger:
                                                for timer_group in source_yaml['timer_groups']:
                                                    if timer_group['name'] == source_trigger['timer_group']:
                                                        return timer_group['frequency']
                                        # If it's not a timer trigger, continue searching
                                        return find_expected_frequency(
                                            source_node_config,
                                            source_topic,
                                            config,
                                            visited_nodes.copy()
                                        )

    return None


def find_topic_publishers(config, target_topic):
    """Find all nodes that publish to the target topic and their frequencies.

    Args:
        config: Dictionary containing benchmark configuration
        target_topic: Name of the topic to find publishers for

    Returns:
        dict: Dictionary mapping node names to their expected frequencies
    """
    publishers = {}

    for node_name, node_config in config['nodes'].items():
        frequency = find_expected_frequency(node_config, target_topic, config)
        if frequency is not None:
            publishers[node_name] = frequency

    return publishers


def analyze_message_trees(node_data, target_node, target_topic, output, message_data):
    """Analyze and write message tree section to output.

    Args:
        node_data: Dictionary containing node message data
        target_node: Name of the node being analyzed
        target_topic: Name of the topic being analyzed
        output: TeeOutput object to write to
        message_data: Dictionary containing all nodes' message data
    """
    output.write("\nLatency Analysis (End-to-End Message Chain):\n")

    # Get target node's received messages
    topic_index = None
    for i, topic_data in enumerate(node_data['received_messages']):
        if topic_data['topic'] == target_topic:
            topic_index = i
            break

    if topic_index is not None:
        messages = node_data['received_messages'][topic_index]['messages']
        latencies = []

        output.write(f"\n{target_node} Message Trees for {target_topic}:\n")
        for msg in tqdm(messages, desc="Processing message chains", leave=False):
            receive_time_sec = msg['timestamp']['sec'] + msg['timestamp']['nanosec'] * 1e-9
            output.write(f"\nMessage Tree for Message: {msg['identifier']}\n")
            output.write(f"  Receive Time: {msg['timestamp']['sec']}.{msg['timestamp']['nanosec']}\n")
            print_parent_chain(msg['identifier'], target_node, message_data, receive_time_sec, indent="└── ", output=output)

        if latencies:
            latencies.sort()
            min_latency = latencies[0]
            max_latency = latencies[-1]
            mean_latency = sum(latencies) / len(latencies)
            median_latency = latencies[len(latencies) // 2] if len(latencies) % 2 == 1 else \
                           (latencies[len(latencies) // 2 - 1] + latencies[len(latencies) // 2]) / 2

            output.write("\nLatency Statistics:\n")
            output.write(f"  Number of Messages: {len(latencies)}\n")
            output.write(f"  Minimum Latency: {min_latency:.6f} seconds\n")
            output.write(f"  Maximum Latency: {max_latency:.6f} seconds\n")
            output.write(f"  Mean Latency: {mean_latency:.6f} seconds\n")
            output.write(f"  Median Latency: {median_latency:.6f} seconds\n")
        else:
            output.write("\nNo complete message chains found\n")
    else:
        output.write(f"\nNo messages found for topic {target_topic} in node {target_node}\n")


def analyze_raw_data(raw_data_file, output_file, include_message_flow=False, include_message_tree=False, target_node=None, target_topic=None, screen_output=False):
    """Analyze and write message tree to file.

    Args:
        raw_data_file: Path to the raw data JSON file
        output_file: File object to write analysis to
        include_message_flow: Whether to include detailed message flow analysis
        include_message_tree: Whether to include message tree analysis
        target_node: Name of the node to analyze
        target_topic: Name of the topic to analyze
        screen_output: Whether to also output to screen
    """
    print("Loading raw data file...")
    # Load the raw data
    with open(raw_data_file, 'r') as f:
        data = json.load(f)

    message_data = data['message_data']
    metadata = data['metadata']
    benchmark_config = metadata['benchmark_config']

    # Create output object
    output = TeeOutput(output_file, screen_output)

    # Validate target node
    if not target_node:
        print("Error: Target node must be specified")
        print(f"Available nodes: {', '.join(message_data.keys())}")
        return

    if target_node not in message_data:
        print(f"Error: Target node '{target_node}' not found in data")
        print(f"Available nodes: {', '.join(message_data.keys())}")
        return

    # Validate target topic
    if not target_topic:
        print("Error: Target topic must be specified")
        node_data = message_data[target_node]
        print(f"Available topics for {target_node}: {[topic_data['topic'] for topic_data in node_data['received_messages']]}")
        return

    # Find expected frequencies for the target topic
    publishers = find_topic_publishers(benchmark_config, target_topic)

    # Filter to just the target node
    node_data = message_data[target_node]

    print("\nWriting analysis to file...")
    output.write("Message Tree Analysis\n")
    output.write("===================\n\n")
    output.write(f"Analysis of data collected at: {metadata['timestamp']}\n")
    output.write(f"Node analyzed: {target_node}\n")
    output.write(f"Topic analyzed: {target_topic}\n")

    # Write expected frequencies
    if publishers:
        output.write("\nExpected Publisher Frequencies:\n")
        for node_name, frequency in publishers.items():
            output.write(f"  {node_name}: {frequency} Hz\n")
    else:
        output.write("\nNo direct publishers found for this topic in configuration\n")

    output.write("\n")

    # Write summary for the node
    output.write(f"\n{target_node} Summary:\n")
    output.write(f"Published Topics: {node_data['published_topics']}\n")
    output.write(f"Received Topics: {[topic_data['topic'] for topic_data in node_data['received_messages']]}\n")
    output.write(f"Number of Published Messages: {len(node_data['published_messages'])}\n")
    for topic_data in node_data['received_messages']:
        if topic_data['topic'] == target_topic:
            output.write(f"Number of Messages Received on {topic_data['topic']}: {len(topic_data['messages'])}\n")

    # Get expected frequency for this topic from all publishers
    expected_frequency = None
    if publishers:
        # Use the minimum frequency since the node can't receive faster than the slowest publisher
        expected_frequency = min(publishers.values())

    # Calculate timestamp delta statistics
    print("\nCalculating timestamp deltas...")
    calculate_timestamp_deltas(node_data, target_topic, output, target_node, expected_frequency)

    # Analyze message flow if requested
    if include_message_flow:
        print("\nAnalyzing message flow...")
        analyze_message_flow(message_data, output)

    # Analyze message tree if requested
    if include_message_tree:
        print("\nAnalyzing message chains and latency...")
        analyze_message_trees(node_data, target_node, target_topic, output, message_data)

    # Flush buffered output to screen if requested
    output.flush_to_screen()


def main():
    parser = argparse.ArgumentParser(description='Analyze message tree from raw data file')
    parser.add_argument('raw_data_file', help='Path to the raw data JSON file')
    parser.add_argument('--output', '-o', help='Output file path (default: message_tree_<timestamp>.txt)')
    parser.add_argument('--message-flow', '-m', action='store_true', help='Include detailed message flow analysis')
    parser.add_argument('--message-tree', '-t', action='store_true', help='Include message tree analysis')
    parser.add_argument('--node', '-n', required=True, help='Name of the node to analyze')
    parser.add_argument('--topic', '-p', required=True, help='Name of the topic to analyze')
    parser.add_argument('--screen', '-s', action='store_true', help='Also output analysis to screen')
    args = parser.parse_args()

    # Generate output filename if not provided
    if not args.output:
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        args.output = f"message_tree_{timestamp}.txt"

    print(f"\nAnalyzing message tree from: {args.raw_data_file}")
    print(f"Output will be written to: {args.output}")
    print(f"Analyzing node: {args.node}")
    print(f"Analyzing topic: {args.topic}")
    if args.message_flow:
        print("Detailed message flow analysis will be included")
    if args.message_tree:
        print("Message tree analysis will be included")
    if args.screen:
        print("Analysis will also be output to screen")
    print()

    with open(args.output, 'w') as output_file:
        analyze_raw_data(args.raw_data_file, output_file, args.message_flow, args.message_tree, args.node, args.topic, args.screen)

    print(f"\nAnalysis complete! Results written to {args.output}")


if __name__ == '__main__':
    main()