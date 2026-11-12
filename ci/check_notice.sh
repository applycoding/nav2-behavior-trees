#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom
set -euo pipefail
export LC_ALL=C.UTF-8
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
require "3rdparty/tinyxml2"
require "3rdparty/minitrace"
require "3rdparty/minicoro"
require "3rdparty/flatbuffers"
require "json.hpp"
require "magic_enum.hpp"
require "any.hpp"
require "expected.hpp"
require "GoogleTest 1.18.0"
require "063de7e9578f82b369302001269680b4b1553359"
require "OpenSSL 3.x"
require "libssl-dev 3.0.13"
require "Apache-2.0"
require "Node.js 26.13.0"
require "TypeScript 7.0.2: Apache-2.0"
require "Vite 8.3.7: MIT"
require "Vitest 5.1.0: MIT"
require "@playwright/test 1.65.0: Apache-2.0"
require "@types/node 22.20.5: MIT"
require "Chrome for Testing"
require "158.0.8127.18"
require "ffmpeg"
require "pytest 9.2.0: MIT"
require "iniconfig 2.3.1: MIT"
require "packaging 26.3: Apache-2.0"
require "pluggy 1.7.0: MIT"
require "pygments 2.21.0: BSD-2-Clause"
require "actions/checkout v7.0.1 (3d3c42e5aac5ba805825da76410c181273ba90b1)"
require "actions/setup-node v7.1.0 (949feb2413d6458794dcd2491c4babbbce0c15c1)"
require "actions/setup-python v7.1.0 (6d4fd22b20465ccc0f7354476588e8df7c9ca0e0)"
require "actions/configure-pages v6.0.0 (45bfe0192ca1faeb007ade9deae92b16b8254a0d)"
require "actions/upload-pages-artifact v5.0.0 (fc324d3547104276b827a68afc52ff2a11cc49c9)"
require "actions/deploy-pages v5.0.1 (368f82528645a54fb793d4d04e342629a3f51346)"
require "actions/upload-artifact v7.0.2 (cf430e030ddbb5b0abf93d22962f4752f3646cd9)"
require "sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855"
require "nav2_msgs 1.1.x"
require "rclcpp, rclcpp_action"
require "/opt/btcpp-4.11"
require "libtinyxml2.a"

dash_grep() {
  local target="$1"
  local ec=0
  grep -P '[\x{2013}\x{2014}\x{2212}]' "$target" >/dev/null 2>&1 || ec=$?
  if [ "$ec" -eq 0 ]; then
    echo "NOTICE contains long dash characters" >&2
    exit 1
  fi
  if [ "$ec" -gt 1 ]; then
    echo "NOTICE dash grep failed (exit $ec)" >&2
    exit 1
  fi
}

dash_grep "$notice"
echo "NOTICE check OK"
