"""只读现场探测：bridge 禁止转发运动；监听最多 10 秒真实 RobotInfo。"""
import os
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
env = {**os.environ, 'ROS_DOMAIN_ID': '189', 'ROS_LOCALHOST_ONLY': '1', 'PYTHONUNBUFFERED': '1'}
folder = ROOT/'logs'
folder.mkdir(exist_ok=True)
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--feedback-source-ip')
parser.add_argument('--preview', action='store_true', help='用真实起点预览 1°、Z 2mm、XY 各 2mm；仍禁止运动')
options = parser.parse_args()
name = 'real_pose_preview' if options.preview else 'real_info_probe'
with open(folder/'real_info_bridge.txt', 'w', encoding='utf8') as stream:
    bridge_args = [sys.executable, '-m', 'estun_segmented.bridge', '--receive-only']
    if options.feedback_source_ip:
        bridge_args += ['--feedback-source-ip', options.feedback_source_ip]
    bridge = subprocess.Popen(bridge_args,
                              env=env, stdout=stream, stderr=subprocess.STDOUT)
    try:
        move_args = [sys.executable, '-m', 'estun_segmented.runner']
        if options.preview:
            move_args += ['--delta','0.002','0.002','0.002','0','0','0.01745329252']
        else:
            move_args += ['--read']
        move_args += ['--log', str(folder/(name+'.jsonl'))]
        result = subprocess.run(move_args,
                                env=env, text=True, capture_output=True, timeout=20)
        (folder/(name+'.txt')).write_text(result.stdout+result.stderr, encoding='utf8')
        print(result.stdout+result.stderr)
    finally:
        bridge.terminate()
        try:
            bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bridge.kill()
            bridge.wait(timeout=5)
sys.exit(result.returncode)
