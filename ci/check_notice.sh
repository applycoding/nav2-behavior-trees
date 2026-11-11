#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
notice="$root/NOTICE"
test -f "$notice"

require() {
  if ! grep -Fq "$1" "$notice"; then
    echo "NOTICE missing required line: $1" >&2
    exit 1
  fi
}

require "REFERENCES.md"
require "BehaviorTree.CPP 4.11.0"
require "8dad25dce26c3fd0787ff5c8790714bc6159dabe"
require "GoogleTest 1.18.0"
require "063de7e9578f82b369302001269680b4b1553359"
require "OpenSSL"
require "Node.js 26"
require "TypeScript 7.0.2"
require "Vite 8.3.7"
require "Vitest 5.1.0"
require "@playwright/test 1.65.0"
require "@types/node 22.20.5"
require "pytest 9.2.0"
require "iniconfig 2.3.1"
require "packaging 26.3"
require "pluggy 1.7.0"
require "pygments 2.21.0"
require "actions/checkout v7.0.1"
require "actions/setup-node v7.1.0"
require "actions/setup-python v7.1.0"
require "actions/configure-pages v6.0.0"
require "actions/upload-pages-artifact v5.0.0"
require "actions/deploy-pages v5.0.1"
require "actions/upload-artifact v7.0.2"

if grep -P '[\x{2013}\x{2014}\x{2212}]' "$notice"; then
  echo "NOTICE contains long dash characters" >&2
  exit 1
fi

echo "NOTICE check OK"
