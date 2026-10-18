#!/usr/bin/env bash
# 仅本机回环测试，固定使用 127.0.0.1；不与真实控制器通信。
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
mkdir -p logs
python3 -m unittest discover -s tests -p test_motion.py -v 2>&1 | tee logs/wsl_phase_unit_tests.txt
python3 tests/ros_loopback.py 2>&1 | tee logs/ros_loopback_phase_summary.txt
