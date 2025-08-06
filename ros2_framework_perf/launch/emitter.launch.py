# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import EmitEvent
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    """Generate launch description for EmitterNode."""
    # Declare the launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='CameraImager01',
        description='Name of the node'
    )

    yaml_config_arg = DeclareLaunchArgument(
        'yaml_config',
        default_value='',
        description='YAML configuration for the emitter node'
    )

    # Get the package share directory
    pkg_share = get_package_share_directory('ros2_framework_perf')

    # Create the node
    emitter_node = Node(
        package='ros2_framework_perf',
        executable='emitter_node',
        name=LaunchConfiguration('node_name'),
        parameters=[{
            'node_name': LaunchConfiguration('node_name'),
            'yaml_config': LaunchConfiguration('yaml_config')
        }],
        output='screen'
    )

    # Create EmitterNode
    emitter_node_composable = ComposableNode(
        package='ros2_framework_perf',
        plugin='ros2_framework_perf::EmitterNode',
        name='emitter_node',
        parameters=[{
            'node_name': LaunchConfiguration('node_name'),
            'yaml_config': LaunchConfiguration('yaml_config')
        }],
        output='screen'
    )

    # Create container with EmitterNode
    container = ComposableNodeContainer(
        name='emitter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[emitter_node_composable],
        output='screen'
    )

    return LaunchDescription([
        node_name_arg,
        yaml_config_arg,
        emitter_node,
        container
    ])
