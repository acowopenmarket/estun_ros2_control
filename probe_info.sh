#!/usr/bin/env bash
# 只接收真实反馈，禁止转发运动。
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 tests/probe_robot_info.py "$@"
