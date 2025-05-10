#!/usr/bin/env python3

import json
import argparse
from datetime import datetime
import time
from tqdm import tqdm
import sys
import yaml
from pathlib import Path


def calculate_timestamp_deltas(node_data, topic_name, output_file, node_name):
    """Calculate and print statistics for timestamp deltas between received messages.

    Args:
        node_data: Dictionary containing node message data
        topic_name: Name of the topic to analyze
        output_file: File object to write analysis to
        node_name: Name of the node being analyzed
    """
    topic_index = None

    # Find the topic index
    for i, topic in enumerate(node_data['received_messages']):
        if topic['topic'] == topic_name:
            topic_index = i
            break

    if topic_index is None:
        output_file.write(f"\nNo {topic_name} messages found in node\n")
        return

    # Get timestamps for messages
    timestamps = node_data['received_messages'][topic_index]['messages']

    if len(timestamps) < 2:
        output_file.write("\nNot enough messages to calculate timestamp deltas\n")
        return

    # Calculate deltas between consecutive timestamps
    deltas = []
    for i in range(1, len(timestamps)):
        prev_time = timestamps[i-1]['timestamp']['sec'] + timestamps[i-1]['timestamp']['nanosec'] * 1e-9
        curr_time = timestamps[i]['timestamp']['sec'] + timestamps[i]['timestamp']['nanosec'] * 1e-9
        delta = curr_time - prev_time
        deltas.append(delta)

    # Calculate statistics
    deltas.sort()
    min_delta = deltas[0]
    max_delta = deltas[-1]
    mean_delta = sum(deltas) / len(deltas)
    median_delta = deltas[len(deltas) // 2] if len(deltas) % 2 == 1 else \
                  (deltas[len(deltas) // 2 - 1] + deltas[len(deltas) // 2]) / 2

    # Calculate standard deviation
    variance = sum((x - mean_delta) ** 2 for x in deltas) / len(deltas)
    stddev = variance ** 0.5

    # Calculate frequency statistics (inverse of time deltas)
    frequencies = [1.0/delta for delta in deltas]
    frequencies.sort()
    min_freq = frequencies[0]
    max_freq = frequencies[-1]
    mean_freq = sum(frequencies) / len(frequencies)
    median_freq = frequencies[len(frequencies) // 2] if len(frequencies) % 2 == 1 else \
                 (frequencies[len(frequencies) // 2 - 1] + frequencies[len(frequencies) // 2]) / 2

    # Write statistics to file
    output_file.write(f"\nTimestamp Delta Statistics for {node_name} receiving {topic_name} messages:\n")
    output_file.write(f"  Minimum delta: {min_delta:.6f} seconds\n")
    output_file.write(f"  Maximum delta: {max_delta:.6f} seconds\n")
    output_file.write(f"  Mean delta: {mean_delta:.6f} seconds\n")
    output_file.write(f"  Median delta: {median_delta:.6f} seconds\n")
    output_file.write(f"  Standard deviation: {stddev:.6f} seconds\n")

    # Write frequency statistics
    output_file.write(f"\nProjected Frequency Statistics:\n")
    output_file.write(f"  Minimum frequency: {min_freq:.2f} Hz\n")
    output_file.write(f"  Maximum frequency: {max_freq:.2f} Hz\n")
    output_file.write(f"  Mean frequency: {mean_freq:.2f} Hz\n")
    output_file.write(f"  Median frequency: {median_freq:.2f} Hz\n")


def print_parent_chain(msg_id, node_name, message_data, receive_time_sec, indent="", output_file=None):
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
        output_file.write(f"{indent}{node_name}: {msg_id} (not found in any node)\n")
        return
    pub_time = msg['publish_timestamp']['sec'] + msg['publish_timestamp']['nanosec'] * 1e-9
    latency = receive_time_sec - pub_time

    output_file.write(f"{indent}{found_node}: {msg['identifier']} | Publish: {msg['publish_timestamp']['sec']}.{msg['publish_timestamp']['nanosec']} | Latency from Actuator: {latency:.6f}s\n")
    if msg['parent_messages']:
        for parent_id in msg['parent_messages']:
            print_parent_chain(parent_id, found_node, message_data, receive_time_sec, indent + "    └── ", output_file)


def analyze_message_flow(message_data, output_file):
    """Analyze and write message flow between nodes to file."""
    output_file.write("\nMessage Flow Analysis:\n")
    for node_name, data in tqdm(message_data.items(), desc="Analyzing message flow", leave=False):
        output_file.write(f"\n{node_name}:\n")
        output_file.write("Published Messages:\n")
        for msg in data['published_messages']:
            output_file.write(f"  ID: {msg['identifier']}\n")
            output_file.write(f"  Timestamp: {msg['publish_timestamp']['sec']}.{msg['publish_timestamp']['nanosec']}\n")

        output_file.write("\nReceived Messages:\n")
        for topic_data in data['received_messages']:
            output_file.write(f"  Topic: {topic_data['topic']}\n")
            for msg in topic_data['messages']:
                output_file.write(f"    Message: {msg['identifier']}\n")
                output_file.write(f"    Timestamp: {msg['timestamp']['sec']}.{msg['timestamp']['nanosec']}\n")


def load_benchmark_config():
    """Load the benchmark graph configuration."""
    # Get the package directory
    package_dir = Path(__file__).parent.parent
    config_path = package_dir / 'config' / 'benchmark_graph.yaml'

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def find_publisher_frequency(node_config, target_topic, config, visited_nodes=None):
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

    # Check timer groups first
    if 'timer_groups' in yaml_config:
        for timer_group in yaml_config['timer_groups']:
            if 'frequency' in timer_group:
                return timer_group['frequency']

    # Check publishers
    if 'publishers' in yaml_config:
        for publisher in yaml_config['publishers']:
            if publisher['topic_name'] == target_topic:
                trigger = publisher['trigger']
                if trigger['type'] == 'timer' and 'frequency' in trigger:
                    return trigger['frequency']
                elif trigger['type'] == 'message_received':
                    # For message_received triggers, trace back to the source topics
                    frequencies = []
                    for source_topic in trigger['topics']:
                        # Find the node that publishes to this topic
                        for source_node_name, source_node_config in config['nodes'].items():
                            source_yaml = yaml.safe_load(source_node_config['config']['yaml_config'])
                            if 'publishers' in source_yaml:
                                for source_pub in source_yaml['publishers']:
                                    if source_pub['topic_name'] == source_topic:
                                        # Recursively find the frequency of this source
                                        source_freq = find_publisher_frequency(
                                            source_node_config,
                                            source_topic,
                                            config,
                                            visited_nodes.copy()
                                        )
                                        if source_freq is not None:
                                            frequencies.append(source_freq)

                    # If we found frequencies from all sources, return the minimum
                    # (as the node can't publish faster than its slowest input)
                    if frequencies:
                        return min(frequencies)

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
        frequency = find_publisher_frequency(node_config, target_topic, config)
        if frequency is not None:
            publishers[node_name] = frequency

    return publishers


def analyze_message_tree(raw_data_file, output_file, include_message_flow=False, include_message_tree=False, target_node=None, target_topic=None):
    """Analyze and write message tree to file.

    Args:
        raw_data_file: Path to the raw data JSON file
        output_file: File object to write analysis to
        include_message_flow: Whether to include detailed message flow analysis
        include_message_tree: Whether to include message tree analysis
        target_node: Name of the node to analyze
        target_topic: Name of the topic to analyze
    """
    print("Loading raw data file...")
    # Load the raw data
    with open(raw_data_file, 'r') as f:
        data = json.load(f)

    message_data = data['message_data']
    metadata = data['metadata']
    benchmark_config = metadata['benchmark_config']

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
    message_data = {target_node: message_data[target_node]}

    print("\nWriting analysis to file...")
    output_file.write("Message Tree Analysis\n")
    output_file.write("===================\n\n")
    output_file.write(f"Analysis of data collected at: {metadata['timestamp']}\n")
    output_file.write(f"Node analyzed: {target_node}\n")
    output_file.write(f"Topic analyzed: {target_topic}\n")

    # Write expected frequencies
    if publishers:
        output_file.write("\nExpected Publisher Frequencies:\n")
        for node_name, frequency in publishers.items():
            output_file.write(f"  {node_name}: {frequency} Hz\n")
    else:
        output_file.write("\nNo direct publishers found for this topic in configuration\n")

    output_file.write("\n")

    # Write summary for the node
    node_data = message_data[target_node]
    output_file.write(f"\n{target_node} Summary:\n")
    output_file.write(f"Published Topics: {node_data['published_topics']}\n")
    output_file.write(f"Received Topics: {[topic_data['topic'] for topic_data in node_data['received_messages']]}\n")
    output_file.write(f"Number of Published Messages: {len(node_data['published_messages'])}\n")
    for topic_data in node_data['received_messages']:
        if topic_data['topic'] == target_topic:
            output_file.write(f"Number of Messages Received on {topic_data['topic']}: {len(topic_data['messages'])}\n")

    # Calculate timestamp delta statistics
    print("\nCalculating timestamp deltas...")
    calculate_timestamp_deltas(node_data, target_topic, output_file, target_node)

    # Analyze message flow if requested
    if include_message_flow:
        print("\nAnalyzing message flow...")
        analyze_message_flow(message_data, output_file)

    # Analyze message tree if requested
    if include_message_tree:
        print("\nAnalyzing message chains and latency...")
        output_file.write("\nLatency Analysis (End-to-End Message Chain):\n")

        # Get target node's received messages
        topic_index = None
        for i, topic_data in enumerate(node_data['received_messages']):
            if topic_data['topic'] == target_topic:
                topic_index = i
                break

        if topic_index is not None:
            messages = node_data['received_messages'][topic_index]['messages']
            latencies = []

            output_file.write(f"\n{target_node} Message Trees for {target_topic}:\n")
            for msg in tqdm(messages, desc="Processing message chains", leave=False):
                receive_time_sec = msg['timestamp']['sec'] + msg['timestamp']['nanosec'] * 1e-9
                output_file.write(f"\nMessage Tree for Message: {msg['identifier']}\n")
                output_file.write(f"  Receive Time: {msg['timestamp']['sec']}.{msg['timestamp']['nanosec']}\n")
                print_parent_chain(msg['identifier'], target_node, message_data, receive_time_sec, indent="└── ", output_file=output_file)

            if latencies:
                latencies.sort()
                min_latency = latencies[0]
                max_latency = latencies[-1]
                mean_latency = sum(latencies) / len(latencies)
                median_latency = latencies[len(latencies) // 2] if len(latencies) % 2 == 1 else \
                               (latencies[len(latencies) // 2 - 1] + latencies[len(latencies) // 2]) / 2

                output_file.write("\nLatency Statistics:\n")
                output_file.write(f"  Number of Messages: {len(latencies)}\n")
                output_file.write(f"  Minimum Latency: {min_latency:.6f} seconds\n")
                output_file.write(f"  Maximum Latency: {max_latency:.6f} seconds\n")
                output_file.write(f"  Mean Latency: {mean_latency:.6f} seconds\n")
                output_file.write(f"  Median Latency: {median_latency:.6f} seconds\n")
            else:
                output_file.write("\nNo complete message chains found\n")
        else:
            output_file.write(f"\nNo messages found for topic {target_topic} in node {target_node}\n")


def main():
    parser = argparse.ArgumentParser(description='Analyze message tree from raw data file')
    parser.add_argument('raw_data_file', help='Path to the raw data JSON file')
    parser.add_argument('--output', '-o', help='Output file path (default: message_tree_<timestamp>.txt)')
    parser.add_argument('--message-flow', '-m', action='store_true', help='Include detailed message flow analysis')
    parser.add_argument('--message-tree', '-t', action='store_true', help='Include message tree analysis')
    parser.add_argument('--node', '-n', required=True, help='Name of the node to analyze')
    parser.add_argument('--topic', '-p', required=True, help='Name of the topic to analyze')
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
    print()

    with open(args.output, 'w') as output_file:
        analyze_message_tree(args.raw_data_file, output_file, args.message_flow, args.message_tree, args.node, args.topic)

    print(f"\nAnalysis complete! Results written to {args.output}")


if __name__ == '__main__':
    main()