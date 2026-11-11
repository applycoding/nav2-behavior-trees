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
  git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git /tmp/btcpp-src
fi
git -C /tmp/btcpp-src fetch --depth 1 origin "$BT_TAG"
git -C /tmp/btcpp-src checkout "$BT_TAG"

built_head=$(git -C /tmp/btcpp-src rev-parse HEAD)
if [ "$built_head" != "$BT_TAG" ]; then
  echo "BT.CPP checkout is $built_head, expected $BT_TAG" >&2
  exit 1
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

cfg="$INSTALL_PREFIX/lib/cmake/behaviortree_cpp/behaviortree_cppConfig.cmake"
if [ ! -f "$cfg" ]; then
  echo "Missing installed cmake config at $cfg" >&2
  exit 1
fi

echo "FetchContent install ready at $INSTALL_PREFIX (commit $built_head == pin 4.11.0)"
echo "CI using FetchContent install at $INSTALL_PREFIX"
