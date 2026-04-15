#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
REPOS_FILE="$REPO_DIR/repos/jazzy-pios-trixie.repos"
WS=/ros2_ws
INSTALL_BASE=/opt/ros/jazzy

mkdir -p "$WS/src"
vcs import "$WS/src" < "$REPOS_FILE"

export PATH="/usr/lib/ccache:$PATH"

cd "$WS"
colcon build \
  --merge-install \
  --install-base "$INSTALL_BASE" \
  --packages-skip rosbag2_examples_cpp rosbag2_examples_py rosbag2_performance_benchmarking \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
  --parallel-workers "$(nproc)"

ccache -s

tar czf /tmp/ros-jazzy-base-pios-arm64.tar.gz -C / opt/ros/jazzy
