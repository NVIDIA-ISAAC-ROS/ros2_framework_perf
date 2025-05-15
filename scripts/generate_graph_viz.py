#!/usr/bin/env python3

import yaml
from pathlib import Path
import subprocess
import os
import json
import argparse
import re
from datetime import datetime

def load_node_configs(json_file):
    """Load node configurations from JSON file."""
    with open(json_file, 'r') as f:
        data = json.load(f)

    # Extract the benchmark_config from the metadata field
    if 'metadata' not in data or 'benchmark_config' not in data['metadata']:
        raise ValueError("No benchmark_config found in metadata field of the JSON file")

    return data['metadata']['benchmark_config']

def find_subscribers_for_topic(topic_name, config):
    """Find all nodes that subscribe to a given topic and whether they trigger publishers."""
    subscribers = []
    for node_name, node_config in config['nodes'].items():
        yaml_config = yaml.safe_load(node_config['config']['yaml_config'])

        # Check if this node has any publishers triggered by this topic
        triggers_publisher = False
        if 'publishers' in yaml_config:
            for pub in yaml_config['publishers']:
                trigger = pub['trigger']
                if trigger['type'] == 'message_received':
                    if 'topics' in trigger and topic_name in trigger['topics']:
                        triggers_publisher = True
                        break
                    if 'subscription_topics' in pub:
                        for sub_topic in pub['subscription_topics']:
                            if sub_topic['topic_name'] == topic_name:
                                triggers_publisher = True
                                break
                elif trigger['type'] == 'timer' and 'subscription_topics' in pub:
                    for sub_topic in pub['subscription_topics']:
                        if sub_topic['topic_name'] == topic_name:
                            subscribers.append((node_name, False))  # Timer-based capture
                            break

        # Check if this node has a direct subscription to this topic
        has_subscription = False
        if 'subscriptions' in yaml_config:
            for sub in yaml_config['subscriptions']:
                if sub['topic_name'] == topic_name:
                    has_subscription = True
                    break

        if triggers_publisher or has_subscription:
            subscribers.append((node_name, triggers_publisher))

    return subscribers

def find_timer_publisher_subscriptions(config):
    """Find all subscription topics used by timer-triggered publishers."""
    timer_subscriptions = {}  # node_name -> list of (topic_name, window_time)

    for node_name, node_config in config['nodes'].items():
        yaml_config = yaml.safe_load(node_config['config']['yaml_config'])

        if 'publishers' in yaml_config:
            for pub in yaml_config['publishers']:
                trigger = pub['trigger']
                if trigger['type'] == 'timer':
                    if node_name not in timer_subscriptions:
                        timer_subscriptions[node_name] = []

                    # Add subscription topics from the publisher's configuration
                    if 'subscription_topics' in trigger:
                        for sub_topic in trigger['subscription_topics']:
                            timer_subscriptions[node_name].append((
                                sub_topic['topic_name'],
                                sub_topic.get('window_time', None)
                            ))

    return timer_subscriptions

def generate_dot_graph(config):
    """Generate DOT format graph visualization."""
    dot_lines = [
        'digraph G {',
        '    rankdir=LR;',
        '    node [fontsize=12];',
        '    edge [fontsize=10];',
        '',
        '    // Node style definitions',
        '    node [shape=oval, style=filled, fillcolor="#76B900"];  // Default node style',
        '',
        '    // Edge style definitions',
        '    edge [fontname="Arial"];',
        '    edge [fontcolor="#333333"];',
        '',
        '    // Node definitions',
    ]

    # First pass: collect all topic information
    topic_info = {}
    for node_name, node_config in config['nodes'].items():
        yaml_config = yaml.safe_load(node_config['config']['yaml_config'])

        # Add publishers
        if 'publishers' in yaml_config:
            for pub in yaml_config['publishers']:
                topic_name = pub['topic_name']
                if topic_name not in topic_info:
                    topic_info[topic_name] = {
                        'publisher': node_name,
                        'frequency': None,
                        'subscribers': [],
                        'window_captures': [],  # Track which topics are captured with windows
                        'window_capture_info': {}  # Store detailed window capture info
                    }

                # Set frequency if timer-based
                trigger = pub['trigger']
                if trigger['type'] == 'timer':
                    if 'frequency' in trigger:
                        topic_info[topic_name]['frequency'] = trigger['frequency']
                    elif 'timer_group' in trigger:
                        for timer_group in yaml_config.get('timer_groups', []):
                            if timer_group['name'] == trigger['timer_group']:
                                topic_info[topic_name]['frequency'] = timer_group['frequency']
                                break

                    # Track window captures with detailed info
                    if 'subscription_topics' in trigger:
                        for sub_topic in trigger['subscription_topics']:
                            if sub_topic.get('mode') == 'window':
                                captured_topic = sub_topic['topic_name']
                                window_time = sub_topic.get('window_time', 0.0)
                                topic_info[topic_name]['window_captures'].append((
                                    captured_topic,
                                    window_time
                                ))
                                topic_info[topic_name]['window_capture_info'][captured_topic] = {
                                    'window_time': window_time,
                                    'mode': 'window'
                                }

    # Second pass: find all subscribers for each topic
    for topic_name, info in topic_info.items():
        info['subscribers'] = find_subscribers_for_topic(topic_name, config)

    # Find timer publisher subscriptions
    timer_subscriptions = find_timer_publisher_subscriptions(config)

    # Third pass: generate DOT graph
    # First add all nodes
    for node_name, node_config in config['nodes'].items():
        yaml_config = yaml.safe_load(node_config['config']['yaml_config'])

        # Create node label with publishers
        label_parts = [f"{node_name}"]

        # Add publishers
        if 'publishers' in yaml_config:
            for pub in yaml_config['publishers']:
                topic_name = pub['topic_name']
                info = topic_info[topic_name]

                # Add publisher info
                if info['frequency']:
                    label_parts.append(f"\\n{pub['topic_name']} ({info['frequency']} Hz)")
                else:
                    label_parts.append(f"\\n{pub['topic_name']} (msg-triggered)")

        # Add node with its label
        label = '\\n'.join(label_parts)
        dot_lines.append(f'    "{node_name}" [label="{label}"];')

    # Add topic nodes and their connections
    for topic_name, info in topic_info.items():
        # Add topic node
        topic_label = f"{topic_name}"
        if info['frequency']:
            topic_label += f"\\n({info['frequency']} Hz)"
        dot_lines.append(f'    "{topic_name}" [label="{topic_label}", shape=box, fillcolor="#5d1682", fontcolor=white];')

        # Add edge from publisher to topic
        dot_lines.append(f'    "{info["publisher"]}" -> "{topic_name}" [label="pub"];')

        # Add edges from topic to subscribers
        for sub_node, triggers in info['subscribers']:
            if triggers:
                dot_lines.append(f'    "{topic_name}" -> "{sub_node}" [label="triggers", color=red];')
            else:
                dot_lines.append(f'    "{topic_name}" -> "{sub_node}" [label="sub"];')

    # Add window capture edges with enhanced visualization
    for node_name, subscriptions in timer_subscriptions.items():
        for topic_name, window_time in subscriptions:
            if window_time:
                label = f"window capture\\n({window_time}s)"
                dot_lines.append(f'    "{topic_name}" -> "{node_name}" [label="{label}", style=dashed, color=blue, penwidth=2];')

    # Close the graph
    dot_lines.append('}')

    return '\n'.join(dot_lines)

def extract_date_from_filename(filename):
    """Extract date from filename in format raw_messages_data_YYYYMMDD_HHMMSS.json"""
    match = re.search(r'raw_messages_data_(\d{8}_\d{6})\.json', filename)
    if match:
        return match.group(1)
    return None

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Generate graph visualization from benchmark data')
    parser.add_argument('json_file', help='Path to the raw_messages_data JSON file')
    args = parser.parse_args()

    # Get resolved symlink filename without extension
    resolved_path = os.path.realpath(args.json_file)
    base_filename = os.path.splitext(os.path.basename(resolved_path))[0]

    # Load configuration from JSON file
    config = load_node_configs(args.json_file)

    # Generate DOT graph
    dot_graph = generate_dot_graph(config)

    # Write DOT file with resolved input filename
    dot_file = f'{base_filename}_graph.dot'
    with open(dot_file, 'w') as f:
        f.write(dot_graph)

    # Generate PNG using Graphviz
    try:
        png_file = f'{base_filename}_graph.png'
        subprocess.run(['dot', '-Tpng', dot_file, '-o', png_file], check=True)
        print(f"Graph visualization generated: {png_file}")
    except subprocess.CalledProcessError as e:
        print(f"Error generating PNG: {e}")
    except FileNotFoundError:
        print("Error: Graphviz 'dot' command not found. Please install Graphviz.")

if __name__ == '__main__':
    main()