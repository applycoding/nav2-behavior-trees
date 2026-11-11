#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

trace_count=0
metrics_count=0
while read -r hash path; do
  [ -z "${hash:-}" ] && continue
  case "$path" in
    traces/*/trace.jsonl)
      pub="$root/web/public/$path"
      if [ ! -f "$pub" ]; then
        echo "missing public trace: $pub" >&2
        exit 1
      fi
      echo "$hash  $pub" | sha256sum -c -
      trace_count=$((trace_count + 1))
      ;;
    traces/*/metrics.json)
      pub="$root/web/public/$path"
      if [ ! -f "$pub" ]; then
        echo "missing public metrics: $pub" >&2
        exit 1
      fi
      if ! cmp -s "$root/$path" "$pub"; then
        echo "public metrics byte mismatch: $path" >&2
        exit 1
      fi
      echo "$hash  $root/$path" | sha256sum -c -
      metrics_count=$((metrics_count + 1))
      ;;
    web/public/traces/manifest.json)
      if [ ! -f "$root/$path" ]; then
        echo "missing $path" >&2
        exit 1
      fi
      echo "$hash  $root/$path" | sha256sum -c -
      ;;
    *) continue ;;
  esac
done < "$root/traces/golden.sha256"

if [ "$trace_count" -ne 9 ] || [ "$metrics_count" -ne 9 ]; then
  echo "expected 9 public trace and 9 metrics comparisons, got $trace_count and $metrics_count" >&2
  exit 1
fi
echo "Public traces check OK ($trace_count scenarios, metrics + manifest)"
