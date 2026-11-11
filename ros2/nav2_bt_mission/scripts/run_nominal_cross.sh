#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: run_nominal_cross.sh MOCK_BIN MISSION_BIN" >&2
  exit 2
fi

MOCK_BIN="$1"
MISSION_BIN="$2"

export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

mock_pid=""
on_exit() {
  if [[ -n "$mock_pid" ]] && kill -0 "$mock_pid" 2>/dev/null; then
    kill -TERM "$mock_pid" 2>/dev/null || true
    wait "$mock_pid" 2>/dev/null || true
  fi
}
trap on_exit EXIT

"$MOCK_BIN" &
mock_pid=$!

for _ in $(seq 1 20); do
  if ! kill -0 "$mock_pid" 2>/dev/null; then
    echo "mock_nav2_servers exited before mission start" >&2
    exit 1
  fi
  sleep 1
done

set +e
"$MISSION_BIN" --scenario nominal
mission_ec=$?
set -e

if ! kill -0 "$mock_pid" 2>/dev/null; then
  echo "mock_nav2_servers died during mission (exit implied non-zero)" >&2
  exit 1
fi

kill -TERM "$mock_pid" 2>/dev/null || true
wait "$mock_pid" 2>/dev/null || true
mock_ec=$?
if [[ "$mock_ec" -ne 0 && "$mock_ec" -ne 143 ]]; then
  echo "mock_nav2_servers exit code $mock_ec" >&2
  exit "$mock_ec"
fi

exit "$mission_ec"
