# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
# SPDX-Generated-By: Cursor

cmake_minimum_required(VERSION 3.8)
project(ros2_framework_perf)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to Release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_python_install_package(${PROJECT_NAME})

# Create library for EmitterNode
ament_auto_add_library(emitter_node SHARED
  src/emitter_node.cpp
  include/ros2_framework_perf/emitter_node.hpp
)
target_include_directories(emitter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(emitter_node
  yaml-cpp
)
rclcpp_components_register_nodes(emitter_node "ros2_framework_perf::EmitterNode")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(launch_testing_ament_cmake REQUIRED)

  # Add launch test
  add_launch_test(
    test/test_emitter_launch.py
    TARGET test_emitter_launch
    ENV
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test:$ENV{PYTHONPATH}
  )

endif()

ament_auto_package(INSTALL_TO_SHARE ../scripts launch)