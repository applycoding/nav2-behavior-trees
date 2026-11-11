#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

# LICENSE is upstream Apache text; package-lock.json is generated.
# build/ and web/node_modules/ are not project-authored.
mapfile -t hits < <(
  grep -RP '[\x{2013}\x{2014}\x{2212}]' \
    --exclude=LICENSE \
    --exclude=package-lock.json \
    --exclude-dir=build \
    --exclude-dir=.git \
    --exclude-dir=node_modules \
    . 2>/dev/null || true
)

if ((${#hits[@]} > 0)); then
  printf '%s\n' "${hits[@]}"
  echo "Unicode dash check failed (${#hits[@]} hits)" >&2
  exit 1
fi
echo "Unicode dash check OK"
