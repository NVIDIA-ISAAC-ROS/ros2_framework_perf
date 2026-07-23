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
import sys
from pathlib import Path

import pytest

import yaml


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


def test_load_results_rejects_unknown_benchmark(tmp_path):
    result_path = tmp_path / 'unknown.json'
    result_path.write_text(
        json.dumps({'benchmark': 'unknown_benchmark'}),
        encoding='utf-8')

    with pytest.raises(ValueError, match='unknown ceiling benchmark'):
        summarizer.load_results(tmp_path)


def test_summary_calculates_medians_and_validates_invariant(tmp_path):
    base = {
        'benchmark': 'scheduler_dispatch',
        'executor': 'events_cbg',
        'threads': 1,
        'operators': 1,
        'run_index': 1,
        'complete': True,
        'waitable_invariant': True,
        'duration_s': 1.0,
    }
    throughputs = [100.0, 300.0, 200.0]
    for index, throughput in enumerate(throughputs):
        result = {
            **base,
            'run_index': index + 1,
            'throughput_ops_s': throughput,
        }
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
        'run_index': 1,
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
    assert 'median_throughput_msg_s' not in rows[0]
    assert any('waitable invariant failed' in error for error in errors)
    summarizer.print_summary(rows)


def test_timeout_output_text_decodes_bytes():
    assert runner.timeout_output_text(b'partial output') == 'partial output'
    assert runner.timeout_output_text(None) == ''


def test_expected_result_validation_reports_missing_run(tmp_path):
    config = {
        'schema_version': 1,
        'executor': 'events_cbg',
        'repetitions': 2,
        'message_passing': {
            'enabled': True,
            'messages_per_flow': 100,
            'matrix': [{'threads': 1, 'flows': 1}],
        },
        'scheduler': {'enabled': False},
    }
    (tmp_path / 'framework_ceiling.yaml').write_text(
        yaml.safe_dump(config), encoding='utf-8')
    result = {
        'benchmark': 'int64_message_passing',
        'executor': 'events_cbg',
        'threads': 1,
        'flows': 1,
        'run_index': 1,
    }
    result_path = tmp_path / 'run-1.json'
    result_path.write_text(json.dumps(result), encoding='utf-8')
    result['_path'] = str(result_path)

    errors = summarizer.validate_expected_results(tmp_path, [result])

    assert any('missing configured run' in error for error in errors)


@pytest.mark.parametrize('result_overrides, result_count', [
    ({}, 2),
    ({'threads': 2}, 1),
])
def test_expected_result_validation_reports_duplicate_or_unexpected_run(
        tmp_path, result_overrides, result_count):
    config = {
        'schema_version': 1,
        'executor': 'events_cbg',
        'repetitions': 1,
        'message_passing': {
            'enabled': True,
            'messages_per_flow': 100,
            'matrix': [{'threads': 1, 'flows': 1}],
        },
        'scheduler': {'enabled': False},
    }
    (tmp_path / 'framework_ceiling.yaml').write_text(
        yaml.safe_dump(config), encoding='utf-8')
    result = {
        'benchmark': 'int64_message_passing',
        'executor': 'events_cbg',
        'threads': 1,
        'flows': 1,
        'run_index': 1,
        '_path': str(tmp_path / 'run.json'),
        **result_overrides,
    }

    errors = summarizer.validate_expected_results(
        tmp_path, [result.copy() for _ in range(result_count)])

    assert any(
        'unexpected or duplicate run' in error for error in errors)


def test_missing_metric_prints_unavailable_instead_of_raising(capsys):
    rows = [{
        'benchmark': 'scheduler_dispatch',
        'executor': 'events_cbg',
        'threads': 1,
        'operators': 1,
        'run_count': 1,
    }]

    summarizer.print_summary(rows)

    assert 'metrics unavailable' in capsys.readouterr().out


def test_invalid_result_set_suppresses_medians():
    rows = [{
        'benchmark': 'scheduler_dispatch',
        'median_throughput_ops_s': 100.0,
        'median_duration_s': 1.0,
    }]

    summarizer.suppress_medians(rows)

    assert 'median_throughput_ops_s' not in rows[0]
    assert 'median_duration_s' not in rows[0]


def test_main_writes_effective_selected_config(
        tmp_path, monkeypatch, capsys):
    config = {
        'schema_version': 1,
        'output_directory': str(tmp_path / 'unused'),
        'repetitions': 5,
        'executor': 'events_cbg',
        'message_passing': {
            'enabled': True,
            'messages_per_flow': 100,
            'matrix': [{'threads': 1, 'flows': 1}],
        },
        'scheduler': {
            'enabled': True,
            'operations_per_operator': 100,
            'matrix': [{'threads': 1, 'operators': 1}],
        },
    }
    config_path = tmp_path / 'config.yaml'
    config_path.write_text(yaml.safe_dump(config), encoding='utf-8')
    output_directory = tmp_path / 'results'
    monkeypatch.setattr(runner, 'run_one', lambda *args: True)
    monkeypatch.setattr(
        sys, 'argv',
        [
            'run_ceiling_benchmarks.py',
            '--config', str(config_path),
            '--benchmark', 'message_passing',
            '--repetitions', '2',
            '--output-directory', str(output_directory),
        ])

    assert runner.main() == 0

    effective_config = yaml.safe_load(
        (output_directory / 'framework_ceiling.yaml').read_text(
            encoding='utf-8'))
    assert effective_config['repetitions'] == 2
    assert effective_config['message_passing']['enabled'] is True
    assert effective_config['scheduler']['enabled'] is False
    assert 'summarize_ceiling_results.py' in capsys.readouterr().out


def test_main_rejects_configuration_with_no_runs(
        tmp_path, monkeypatch):
    config = {
        'schema_version': 1,
        'output_directory': str(tmp_path / 'unused'),
        'repetitions': 1,
        'executor': 'events_cbg',
        'message_passing': {'enabled': False},
        'scheduler': {'enabled': False},
    }
    config_path = tmp_path / 'config.yaml'
    config_path.write_text(yaml.safe_dump(config), encoding='utf-8')
    output_directory = tmp_path / 'results'
    monkeypatch.setattr(
        sys, 'argv',
        [
            'run_ceiling_benchmarks.py',
            '--config', str(config_path),
            '--output-directory', str(output_directory),
        ])

    with pytest.raises(SystemExit) as error:
        runner.main()

    assert error.value.code == 2
    assert not output_directory.exists()
