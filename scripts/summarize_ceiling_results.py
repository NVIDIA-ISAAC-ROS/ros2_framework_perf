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

"""Validate ceiling benchmark results and calculate repeated-run medians."""

import argparse
import csv
import json
import sys
from pathlib import Path
from statistics import median


BENCHMARK_FIELDS = {
    'int64_message_passing': {
        'dimension': 'flows',
        'metrics': [
            'source_publish_msg_s',
            'throughput_msg_s',
            'avg_latency_us',
            'min_latency_us',
            'max_latency_us',
            'drain_lag_ms',
            'duration_s',
        ],
    },
    'scheduler_dispatch': {
        'dimension': 'operators',
        'metrics': [
            'throughput_ops_s',
            'duration_s',
        ],
    },
}


def load_results(directory):
    """Load individual benchmark result files from a directory."""
    results = []
    for path in sorted(directory.glob('*.json')):
        if path.name == 'summary.json':
            continue
        with path.open('r', encoding='utf-8') as result_file:
            result = json.load(result_file)
        if result.get('benchmark') in BENCHMARK_FIELDS:
            result['_path'] = str(path)
            results.append(result)
    if not results:
        raise ValueError(
            f'no ceiling benchmark result JSON files found in {directory}')
    return results


def summarize(results):
    """Group runs by matrix cell and calculate medians."""
    grouped = {}
    validation_errors = []

    for result in results:
        benchmark = result['benchmark']
        definition = BENCHMARK_FIELDS[benchmark]
        dimension_key = definition['dimension']
        required = ['executor', 'threads', dimension_key]
        missing = [field for field in required if field not in result]
        if missing:
            validation_errors.append(
                f'{result["_path"]}: missing fields {", ".join(missing)}')
            continue

        key = (
            benchmark,
            result['executor'],
            int(result['threads']),
            int(result[dimension_key]),
        )
        grouped.setdefault(key, []).append(result)

        if result.get('complete') is not True:
            validation_errors.append(
                f'{result["_path"]}: complete is not true')
        if result.get('waitable_invariant') is not True:
            validation_errors.append(
                f'{result["_path"]}: waitable invariant failed')

    rows = []
    for key, group in sorted(grouped.items()):
        benchmark, executor, threads, dimension = key
        definition = BENCHMARK_FIELDS[benchmark]
        row = {
            'benchmark': benchmark,
            'executor': executor,
            'threads': threads,
            definition['dimension']: dimension,
            'run_count': len(group),
            'all_complete': all(
                item.get('complete') is True for item in group),
            'all_waitable_invariants': all(
                item.get('waitable_invariant') is True for item in group),
        }
        for metric in definition['metrics']:
            values = [
                float(item[metric]) for item in group if metric in item
            ]
            if len(values) != len(group):
                validation_errors.append(
                    f'{benchmark}/{executor}/{threads}/{dimension}: '
                    f'missing metric {metric}')
                continue
            row[f'median_{metric}'] = median(values)
        rows.append(row)

    return rows, validation_errors


def write_csv(path, rows):
    """Write summary rows with the union of their columns."""
    fieldnames = []
    for row in rows:
        for field in row:
            if field not in fieldnames:
                fieldnames.append(field)
    with path.open('w', encoding='utf-8', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def print_summary(rows):
    """Print a compact human-readable summary."""
    for row in rows:
        dimension_key = BENCHMARK_FIELDS[row['benchmark']]['dimension']
        if row['benchmark'] == 'int64_message_passing':
            primary = (
                f'{row["median_throughput_msg_s"]:,.0f} msg/s, '
                f'{row["median_avg_latency_us"]:.3f} us avg')
        else:
            primary = f'{row["median_throughput_ops_s"]:,.0f} ops/s'
        print(
            f'{row["benchmark"]}: executor={row["executor"]} '
            f'threads={row["threads"]} {dimension_key}={row[dimension_key]} '
            f'runs={row["run_count"]} median={primary}')


def main():
    """Validate and summarize a result directory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result_directory', type=Path)
    args = parser.parse_args()

    try:
        results = load_results(args.result_directory)
        rows, validation_errors = summarize(results)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 1

    summary = {
        'result_directory': str(args.result_directory),
        'result_count': len(results),
        'validation_passed': not validation_errors,
        'validation_errors': validation_errors,
        'groups': rows,
    }
    summary_json_path = args.result_directory / 'summary.json'
    summary_csv_path = args.result_directory / 'summary.csv'
    summary_json_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    write_csv(summary_csv_path, rows)
    print_summary(rows)
    print(f'JSON summary: {summary_json_path}')
    print(f'CSV summary: {summary_csv_path}')

    if validation_errors:
        for error in validation_errors:
            print(f'validation error: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
