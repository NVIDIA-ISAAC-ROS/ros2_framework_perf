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

"""Run YAML-defined ROS 2 framework-ceiling benchmark matrices."""

import argparse
import copy
import json
import os
import shlex
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import yaml


BENCHMARKS = {
    'message_passing': {
        'executable': 'int64_ceiling_benchmark',
        'result_benchmark': 'int64_message_passing',
        'count_key': 'messages_per_flow',
        'dimension_key': 'flows',
        'count_env': 'ROS2_FRAMEWORK_PERF_CEILING_MESSAGES',
        'dimension_env': 'ROS2_FRAMEWORK_PERF_CEILING_FLOWS',
    },
    'scheduler': {
        'executable': 'scheduler_ceiling_benchmark',
        'result_benchmark': 'scheduler_dispatch',
        'count_key': 'operations_per_operator',
        'dimension_key': 'operators',
        'count_env': 'ROS2_FRAMEWORK_PERF_CEILING_OPERATIONS',
        'dimension_env': 'ROS2_FRAMEWORK_PERF_CEILING_OPERATORS',
    },
}


def load_config(path):
    """Load and minimally validate a ceiling benchmark configuration."""
    with path.open('r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)

    if not isinstance(config, dict) or config.get('schema_version') != 1:
        raise ValueError('framework ceiling config must use schema_version: 1')
    if config.get('executor') != 'events_cbg':
        raise ValueError(
            'framework ceiling config executor must be events_cbg')
    if int(config.get('repetitions', 0)) <= 0:
        raise ValueError(
            'framework ceiling repetitions must be greater than zero')

    for benchmark_name, definition in BENCHMARKS.items():
        benchmark_config = config.get(benchmark_name, {})
        if not benchmark_config.get('enabled', False):
            continue
        if int(benchmark_config.get(definition['count_key'], 0)) <= 0:
            raise ValueError(
                f'{benchmark_name}.{definition["count_key"]} must be greater '
                'than zero')
        if not benchmark_config.get('matrix'):
            raise ValueError(f'{benchmark_name}.matrix must not be empty')
        for cell in benchmark_config['matrix']:
            if int(cell.get('threads', 0)) <= 0:
                raise ValueError(
                    f'{benchmark_name} matrix threads must be greater than '
                    'zero')
            if int(cell.get(definition['dimension_key'], 0)) <= 0:
                raise ValueError(
                    f'{benchmark_name} matrix {definition["dimension_key"]} '
                    'must be greater than zero')
    return config


def parse_result(output):
    """Return the final JSON object printed by a benchmark executable."""
    for line in reversed(output.splitlines()):
        candidate = line.strip()
        if candidate.startswith('{') and candidate.endswith('}'):
            return json.loads(candidate)
    raise ValueError('benchmark did not print a JSON result')


def timeout_output_text(output):
    """Normalize TimeoutExpired output for JSON serialization."""
    if isinstance(output, bytes):
        return output.decode(errors='replace')
    return output or ''


def build_runs(config, selected_benchmark, repetitions):
    """Yield benchmark command metadata in interleaved repetition order."""
    benchmark_names = (
        [selected_benchmark]
        if selected_benchmark != 'all'
        else list(BENCHMARKS))
    for repetition in range(1, repetitions + 1):
        for benchmark_name in benchmark_names:
            benchmark_config = config.get(benchmark_name, {})
            if not benchmark_config.get('enabled', False):
                continue
            definition = BENCHMARKS[benchmark_name]
            for cell in benchmark_config['matrix']:
                yield (
                    repetition, benchmark_name, benchmark_config,
                    definition, cell, config['executor'])


def run_one(run, output_directory, dry_run):
    """Run one matrix cell and write its JSON result."""
    (
        repetition, benchmark_name, benchmark_config, definition, cell,
        executor,
    ) = run
    threads = int(cell['threads'])
    dimension = int(cell[definition['dimension_key']])
    timeout_seconds = int(benchmark_config.get('timeout_seconds', 60))

    environment = os.environ.copy()
    environment.update({
        'ROS2_FRAMEWORK_PERF_CEILING_THREADS': str(threads),
        definition['count_env']: str(
            benchmark_config[definition['count_key']]),
        definition['dimension_env']: str(dimension),
        'ROS2_FRAMEWORK_PERF_CEILING_TIMEOUT_SEC': str(timeout_seconds),
    })
    command = [
        'ros2', 'run', 'ros2_framework_perf', definition['executable'],
    ]
    result_path = output_directory / (
        f'{benchmark_name}_{executor}_threads-{threads}_'
        f'{definition["dimension_key"]}-{dimension}_run-{repetition}.json')

    print(
        f'[{repetition}] {benchmark_name}: executor={executor} '
        f'threads={threads} {definition["dimension_key"]}={dimension}')
    if dry_run:
        print(
            '  env ' +
            ' '.join(
                f'{key}={environment[key]}' for key in environment
                if key.startswith('ROS2_FRAMEWORK_PERF_CEILING_')) +
            ' ' + ' '.join(command))
        return True

    try:
        process = subprocess.run(
            command,
            env=environment,
            text=True,
            capture_output=True,
            timeout=timeout_seconds + 15,
            check=False,
        )
        return_code = process.returncode
        try:
            result = parse_result(process.stdout + '\n' + process.stderr)
        except (ValueError, json.JSONDecodeError) as error:
            result = {
                'benchmark': definition['result_benchmark'],
                'complete': False,
                'error': str(error),
                'stdout': process.stdout,
                'stderr': process.stderr,
            }
    except subprocess.TimeoutExpired as error:
        return_code = None
        result = {
            'benchmark': definition['result_benchmark'],
            'complete': False,
            'error': f'process timed out after {error.timeout} seconds',
            'stdout': timeout_output_text(error.stdout),
            'stderr': timeout_output_text(error.stderr),
        }

    result.setdefault('executor', executor)
    result.setdefault('threads', threads)
    result.setdefault(definition['dimension_key'], dimension)
    result.setdefault('waitable_invariant', False)
    result['run_index'] = repetition
    result['process_return_code'] = return_code
    result_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')

    valid = (
        return_code == 0 and
        result.get('complete') is True and
        result.get('waitable_invariant') is True
    )
    if not valid:
        print(f'  FAILED: {result_path}', file=sys.stderr)
    return valid


def main():
    """Run the configured benchmark matrices."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config',
        type=Path,
        default=Path('config/framework_ceiling.yaml'),
        help='YAML matrix configuration')
    parser.add_argument(
        '--benchmark',
        choices=['all', *BENCHMARKS],
        default='all',
        help='run one benchmark family or all enabled families')
    parser.add_argument(
        '--repetitions',
        type=int,
        help='override repetitions from YAML')
    parser.add_argument(
        '--output-directory',
        type=Path,
        help=(
            'write results to this directory instead of a timestamped '
            'directory'))
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='validate and print commands without executing them')
    args = parser.parse_args()

    config = load_config(args.config)
    repetitions = (
        args.repetitions
        if args.repetitions is not None
        else int(config['repetitions'])
    )
    if repetitions <= 0:
        parser.error('--repetitions must be greater than zero')

    runs = list(build_runs(config, args.benchmark, repetitions))
    if not runs:
        parser.error('configuration selects no enabled benchmark runs')

    if args.output_directory:
        output_directory = args.output_directory
    else:
        timestamp = datetime.now().strftime('%Y%m%d-%H%M%S')
        output_directory = Path(config['output_directory']) / timestamp

    if not args.dry_run:
        output_directory.mkdir(parents=True, exist_ok=False)
        effective_config = copy.deepcopy(config)
        effective_config['repetitions'] = repetitions
        if args.benchmark != 'all':
            for benchmark_name in BENCHMARKS:
                if benchmark_name != args.benchmark:
                    benchmark_config = effective_config.setdefault(
                        benchmark_name, {})
                    benchmark_config['enabled'] = False
        (output_directory / 'framework_ceiling.yaml').write_text(
            yaml.safe_dump(effective_config, sort_keys=False),
            encoding='utf-8')

    success = True
    for run in runs:
        success = run_one(run, output_directory, args.dry_run) and success

    if not args.dry_run:
        print(f'Results written to {output_directory}')
        summarizer = Path(__file__).with_name(
            'summarize_ceiling_results.py')
        print(
            'Summarize with: '
            f'python3 {shlex.quote(str(summarizer))} '
            f'{shlex.quote(str(output_directory))}')
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
