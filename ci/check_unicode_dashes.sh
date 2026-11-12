#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom
set -euo pipefail
export LC_ALL=C.UTF-8
root="$(cd "$(dirname "$0")/.." && pwd)"
search_root="${1:-$root}"
cd "$search_root"

# LICENSE is upstream Apache text; package-lock.json is generated.
# build/ and web/node_modules/ are not project-authored (only when scanning repo root).
grep_args=(
  -RP '[\x{2013}\x{2014}\x{2212}]'
  --exclude=LICENSE
  --exclude=package-lock.json
  --exclude-dir=build
  --exclude-dir=.git
  --exclude-dir=node_modules
  .
)

hits=()
ec=0
mapfile -t hits < <(grep "${grep_args[@]}" 2>/dev/null) || ec=$?
if [ "$ec" -gt 1 ]; then
  echo "Unicode dash grep failed (exit $ec)" >&2
  exit 1
fi

if [ "${#hits[@]}" -gt 0 ]; then
  printf '%s\n' "${hits[@]}"
  echo "Unicode dash check failed (${#hits[@]} hits)" >&2
  exit 1
fi
echo "Unicode dash check OK"
