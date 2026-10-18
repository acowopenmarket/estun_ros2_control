#!/usr/bin/env bash
# 在 Ubuntu 22.04 / WSL Ubuntu 22.04 中构建；不启动通信、不发送运动。
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
mkdir -p logs
cd ros2_ws
colcon build --symlink-install --event-handlers console_direct+ 2>&1 | tee ../logs/colcon_build.txt
