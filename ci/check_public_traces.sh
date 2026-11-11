#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

count=0
while read -r hash path; do
  [ -z "${hash:-}" ] && continue
  case "$path" in
    traces/*/trace.jsonl) ;;
    *) continue ;;
  esac
  pub="$root/web/public/$path"
  if [ ! -f "$pub" ]; then
    echo "missing public trace: $pub" >&2
    exit 1
  fi
  echo "$hash  $pub" | sha256sum -c -
  count=$((count + 1))
done < "$root/traces/golden.sha256"

if [ "$count" -ne 9 ]; then
  echo "expected 9 public trace comparisons, got $count" >&2
  exit 1
fi
echo "Public traces check OK ($count scenarios)"
