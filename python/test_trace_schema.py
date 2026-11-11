# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom

"""Criterion 20: validate trace.jsonl schema from CI-generated traces."""

from __future__ import annotations

import json
import pathlib
import re

import pytest

REPO = pathlib.Path(__file__).resolve().parents[1]
TRACE_DIRS = [
    REPO / "traces" / "ci",
    REPO / "traces" / "web_gen",
]

STATUS_ENUM = frozenset({"IDLE", "RUNNING", "SUCCESS", "FAILURE"})

SAMPLE_KEYS = frozenset(
    {"k", "tick", "pose", "battery", "docked", "brushes", "cov"}
)
TRANSITION_KEYS = frozenset({"k", "tick", "seq", "uid", "from", "to"})

HEADER_KEYS = frozenset(
    {"schema_version", "scenario", "dt_ms", "tree_sha256", "nodes", "map"}
)


def discover_traces() -> list[pathlib.Path]:
    paths: list[pathlib.Path] = []
    for base in TRACE_DIRS:
        if not base.is_dir():
            continue
        for child in sorted(base.iterdir()):
            p = child / "trace.jsonl" if child.is_dir() else base / "trace.jsonl"
            if child.is_dir() and p.is_file():
                paths.append(p)
        flat = base / "trace.jsonl"
        if flat.is_file():
            paths.append(flat)
    return paths


def parse_jsonl(path: pathlib.Path) -> tuple[dict, list[dict], list[dict]]:
    lines = path.read_text(encoding="utf-8").splitlines()
    assert lines, f"empty trace: {path}"
    header = json.loads(lines[0])
    samples: list[dict] = []
    transitions: list[dict] = []
    for line in lines[1:]:
        if not line.strip():
            continue
        row = json.loads(line)
        if row.get("k") == "s":
            samples.append(row)
        elif row.get("k") == "t":
            transitions.append(row)
        else:
            raise AssertionError(f"unknown record kind in {path}: {row}")
    return header, samples, transitions


def validate_header(header: dict, path: pathlib.Path) -> None:
    missing = HEADER_KEYS - header.keys()
    assert not missing, f"{path}: header missing keys {missing}"
    assert header["schema_version"] == 1
    assert header["dt_ms"] == 100
    assert isinstance(header["scenario"], str) and header["scenario"]
    assert isinstance(header["nodes"], list) and header["nodes"]


def validate_samples(samples: list[dict], path: pathlib.Path) -> None:
    prev_tick = -1
    for s in samples:
        missing = SAMPLE_KEYS - s.keys()
        assert not missing, f"{path}: sample missing keys {sorted(missing)}"
        assert "battery" in s, f"{path}: sample missing battery"
        tick = s["tick"]
        assert isinstance(tick, int)
        assert tick >= prev_tick, f"{path}: ticks not monotonic ({prev_tick} -> {tick})"
        prev_tick = tick
        assert len(s["pose"]) == 2
        assert isinstance(s["cov"], list) and len(s["cov"]) == 3


def validate_transitions(transitions: list[dict], path: pathlib.Path) -> None:
    for t in transitions:
        missing = TRANSITION_KEYS - t.keys()
        assert not missing, f"{path}: transition missing keys {missing}"
        assert t["from"] in STATUS_ENUM, f"{path}: bad from status {t['from']!r}"
        assert t["to"] in STATUS_ENUM, f"{path}: bad to status {t['to']!r}"


@pytest.mark.parametrize("trace_path", discover_traces(), ids=lambda p: str(p.relative_to(REPO)))
def test_trace_schema(trace_path: pathlib.Path) -> None:
  header, samples, transitions = parse_jsonl(trace_path)
  validate_header(header, trace_path)
  assert samples, f"{trace_path}: no samples"
  validate_samples(samples, trace_path)
  validate_transitions(transitions, trace_path)


def test_rejects_sample_missing_battery(tmp_path: pathlib.Path) -> None:
    bad = tmp_path / "bad.jsonl"
    bad.write_text(
        '{"schema_version":1,"scenario":"x","dt_ms":100,"tree_sha256":"00",'
        '"nodes":[],"map":{"dock":[0,0]}}\n'
        '{"k":"s","tick":0,"pose":[0,0],"docked":false,"brushes":false,"cov":[0,0,0]}\n',
        encoding="utf-8",
    )
    _, samples, _ = parse_jsonl(bad)
    with pytest.raises(AssertionError, match="battery"):
        validate_samples(samples, bad)


ALL_SCENARIOS = frozenset(
    {
        "nominal",
        "low_battery",
        "critical_battery",
        "blocked_corridor",
        "phantom_obstacle",
        "stuck",
        "unreachable_zone",
        "lane_obstacle",
        "lane_blocked_perm",
    }
)


def test_discover_at_least_one_trace() -> None:
    paths = discover_traces()
    assert paths, "no trace.jsonl found under traces/ci or traces/web_gen"
    found = {p.parent.name for p in paths if p.parent.is_dir()}
    missing = ALL_SCENARIOS - found
    assert not missing, f"missing scenarios in traces/ci or traces/web_gen: {sorted(missing)}"
