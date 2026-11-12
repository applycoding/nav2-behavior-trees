#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
readme="$root/README.md"
test -f "$readme"

extract_paths() {
  grep -Eo '`[^`]+`' "$readme" | tr -d '`' | while read -r p; do
    case "$p" in
      trees/*|web/*|python/*|PLAN.md|TASKS.md|DEVIATIONS.md|LICENSE|NOTICE|ros2/*)
        echo "$p"
        ;;
    esac
  done
}

missing=0
while read -r rel; do
  [ -z "$rel" ] && continue
  if [ ! -e "$root/$rel" ]; then
    echo "README references missing path: $rel" >&2
    missing=1
  fi
done < <(extract_paths)

test "$missing" -eq 0
echo "README path check OK"
