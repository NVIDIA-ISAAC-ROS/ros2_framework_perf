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

"""Tests for the framework-ceiling matrix and summary tools."""

import importlib.util
import json
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).parents[2]


def load_script(name):
    """Import a repository script by filename."""
    path = REPOSITORY_ROOT / 'scripts' / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


runner = load_script('run_ceiling_benchmarks.py')
summarizer = load_script('summarize_ceiling_results.py')


def test_default_config_builds_expected_runs():
    config = runner.load_config(
        REPOSITORY_ROOT / 'config' / 'framework_ceiling.yaml')
    runs = list(runner.build_runs(config, 'all', repetitions=1))

    expected_cells = (
        len(config['message_passing']['matrix']) +
        len(config['scheduler']['matrix'])
    )
    assert len(runs) == expected_cells


def test_parse_result_uses_final_json_line():
    result = runner.parse_result(
        'diagnostic output\n{"complete":true,"waitable_invariant":true}\n')
    assert result['complete'] is True
    assert result['waitable_invariant'] is True


def test_summary_calculates_medians_and_validates_invariant(tmp_path):
    base = {
        'benchmark': 'scheduler_dispatch',
        'executor': 'events_cbg',
        'threads': 1,
        'operators': 1,
        'complete': True,
        'waitable_invariant': True,
        'duration_s': 1.0,
    }
    throughputs = [100.0, 300.0, 200.0]
    for index, throughput in enumerate(throughputs):
        result = {**base, 'throughput_ops_s': throughput}
        (tmp_path / f'run-{index}.json').write_text(
            json.dumps(result), encoding='utf-8')

    rows, errors = summarizer.summarize(
        summarizer.load_results(tmp_path))

    assert errors == []
    assert len(rows) == 1
    assert rows[0]['run_count'] == 3
    assert rows[0]['median_throughput_ops_s'] == 200.0
    assert rows[0]['all_waitable_invariants'] is True


def test_summary_reports_failed_invariant(tmp_path):
    result = {
        'benchmark': 'int64_message_passing',
        'executor': 'events_cbg',
        'threads': 1,
        'flows': 1,
        'complete': True,
        'waitable_invariant': False,
        'source_publish_msg_s': 100.0,
        'throughput_msg_s': 100.0,
        'avg_latency_us': 1.0,
        'min_latency_us': 0.5,
        'max_latency_us': 2.0,
        'drain_lag_ms': 0.0,
        'duration_s': 1.0,
    }
    (tmp_path / 'run.json').write_text(
        json.dumps(result), encoding='utf-8')

    rows, errors = summarizer.summarize(
        summarizer.load_results(tmp_path))

    assert rows[0]['all_waitable_invariants'] is False
    assert any('waitable invariant failed' in error for error in errors)
