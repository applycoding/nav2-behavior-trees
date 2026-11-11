#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Arpan Bom
set -euo pipefail

BT_TAG=8dad25dce26c3fd0787ff5c8790714bc6159dabe
INSTALL_PREFIX=/opt/btcpp-4.11

apt_ver=$(dpkg-query -W -f='${Version}' ros-humble-behaviortree-cpp 2>/dev/null || true)
if [ -n "$apt_ver" ] && [[ "$apt_ver" == 4.11.0* ]]; then
  echo "Using apt ros-humble-behaviortree-cpp version $apt_ver"
  exit 0
fi

echo "Apt BT.CPP not 4.11.0 (got: ${apt_ver:-none}); building FetchContent pin $BT_TAG"

if [ ! -d /tmp/btcpp-src ]; then
  git clone --depth 1 https://github.com/BehaviorTree/BehaviorTree.CPP.git /tmp/btcpp-src
  git -C /tmp/btcpp-src fetch --depth 1 origin "$BT_TAG"
  git -C /tmp/btcpp-src checkout "$BT_TAG"
fi

cmake -S /tmp/btcpp-src -B /tmp/btcpp-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
  -DBTCPP_GROOT_INTERFACE=OFF \
  -DBTCPP_SQLITE_LOGGING=OFF \
  -DBTCPP_EXAMPLES=OFF \
  -DBTCPP_BUILD_TOOLS=OFF \
  -DBUILD_TESTING=OFF

cmake --build /tmp/btcpp-build --parallel
cmake --install /tmp/btcpp-build

ver=$("$INSTALL_PREFIX/bin/btcpp_version" 2>/dev/null | head -1 || true)
if [ -z "$ver" ]; then
  ver=$(grep -E '^[0-9]+\.[0-9]+\.[0-9]+' "$INSTALL_PREFIX/include/behaviortree_cpp/bt_version.h" | head -1 || true)
fi
echo "Installed BT.CPP: $ver at $INSTALL_PREFIX"

if [[ "$ver" != 4.11.0* ]]; then
  echo "BehaviorTree.CPP version is not 4.11.0: $ver" >&2
  exit 1
fi

echo "FetchContent install ready at $INSTALL_PREFIX"
