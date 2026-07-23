#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Run a small ceiling benchmark and validate its JSON contract."""

import json
import os
import subprocess
import sys


def main():
    """Run the selected executable with a 100-operation workload."""
    if len(sys.argv) != 3:
        print(
            'usage: verify_ceiling_executable.py EXECUTABLE '
            '{message_passing|scheduler}',
            file=sys.stderr)
        return 2

    executable, benchmark = sys.argv[1:]
    environment = os.environ.copy()
    environment.update({
        'ROS2_FRAMEWORK_PERF_CEILING_THREADS': '1',
        'ROS2_FRAMEWORK_PERF_CEILING_TIMEOUT_SEC': '10',
    })
    if benchmark == 'message_passing':
        environment.update({
            'ROS2_FRAMEWORK_PERF_CEILING_FLOWS': '1',
            'ROS2_FRAMEWORK_PERF_CEILING_MESSAGES': '100',
        })
        expected_benchmark = 'int64_message_passing'
    elif benchmark == 'scheduler':
        environment.update({
            'ROS2_FRAMEWORK_PERF_CEILING_OPERATORS': '1',
            'ROS2_FRAMEWORK_PERF_CEILING_OPERATIONS': '100',
        })
        expected_benchmark = 'scheduler_dispatch'
    else:
        print(f'unknown benchmark: {benchmark}', file=sys.stderr)
        return 2

    process = subprocess.run(
        [executable],
        env=environment,
        text=True,
        capture_output=True,
        timeout=15,
        check=False,
    )
    if process.returncode != 0:
        print(process.stdout, file=sys.stderr)
        print(process.stderr, file=sys.stderr)
        return 1

    result = None
    for line in reversed(process.stdout.splitlines()):
        candidate = line.strip()
        if candidate.startswith('{') and candidate.endswith('}'):
            result = json.loads(candidate)
            break
    assert result is not None
    assert result['benchmark'] == expected_benchmark
    assert result['complete'] is True
    assert result['waitable_invariant'] is True
    return 0


if __name__ == '__main__':
    sys.exit(main())
