"""WSL ROS2 集成测试：真实 ROS2 节点 + 本机模拟 UDP 控制器；不连接实机。"""
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
from estun_segmented.core import Settings, command, plan
from estun_segmented.wire import validate_command

START = [0.4, 0.1, 0.3, math.pi, 0, 0]
TARGET = [0.445, 0.14, 0.345, math.pi, 0, math.radians(17)]


class MockController(threading.Thread):
    def __init__(self, scenario):
        super().__init__(daemon=True)
        self.scenario = scenario
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(('127.0.0.1', 9005))
        self.rx.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if scenario == 'relayed_feedback':
            self.tx.bind(('127.0.0.2', 0))
        self.stop_event = threading.Event()
        self.commands, self.failures = [], []
        self.pose, self.state, self.fault = list(START), 'AutoReady', 0
        self.until, self.pending, self.ready_at = None, None, 0
        self.started, self.pending_points = None, []
        self.error = None

    def run(self):
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                try:
                    data, _ = self.rx.recvfrom(65535)
                    obj = json.loads(data)
                    validate_command(obj)
                    if self.state != 'AutoReady' or self.until is not None:
                        self.failures.append('前一阶段还在运行就发送下一阶段')
                    if self.commands and now-self.ready_at < 0.19:
                        self.failures.append('上一段尚未完成稳定到位等待')
                    self.commands.append(obj)
                    self.pending_points = [p['positions'] for p in obj['points']]
                    self.pending = self.pending_points[-1]
                    if self.scenario != 'no_ack':
                        self.started = now
                        self.state, self.until = 'AutoRunning', now+0.12*len(self.pending_points)+0.1
                except BlockingIOError:
                    pass
                if self.until is not None and now < self.until:
                    elapsed = now-self.started
                    reached = min(len(self.pending_points)-1, int(elapsed/0.12))
                    if reached > 0 and self.scenario != 'wrong_pose':
                        self.pose = self.pending_points[reached-1]
                    self.state = 'AutoRunning'
                if self.until is not None and now >= self.until:
                    if self.scenario == 'fault' or (self.scenario == 'fault_z' and len(self.commands) == 2):
                        self.fault = 1
                    elif self.scenario != 'wrong_pose':
                        self.pose = self.pending
                    self.state, self.until, self.ready_at = 'AutoReady', None, now
                if not (self.scenario == 'dropout' and self.commands):
                    if self.scenario == 'malformed' and self.commands:
                        payload = b'{bad json'
                    else:
                        payload = json.dumps({'joint_positions': [0]*6,
                                              'end_positions': self.pose,
                                              'state': self.state,
                                              'fault_flag': self.fault}).encode()
                    self.tx.sendto(payload, ('127.0.0.1', 9006))
                self.stop_event.wait(0.02)
        except Exception as exc:
            self.error = exc

    def close(self):
        self.stop_event.set()
        self.join(timeout=2)
        self.rx.close()
        self.tx.close()


def stop_process(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def run_scenario(scenario, domain):
    folder = ROOT/'logs'/'ros_loopback_phase'
    folder.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, 'ROS_LOCALHOST_ONLY': '1', 'ROS_DOMAIN_ID': str(domain),
           'PYTHONUNBUFFERED': '1'}
    config = folder/(scenario+'_config.json')
    config.write_text(json.dumps({'target': TARGET, 'accept_timeout': 1.0,
                                 'segment_timeout': 2.5}), encoding='utf8')
    controller = MockController(scenario)
    bridge_log = open(folder/(scenario+'_bridge.txt'), 'w', encoding='utf8')
    bridge = None
    try:
        bridge_args = [sys.executable, '-m', 'estun_segmented.bridge',
                       '--controller-ip', '127.0.0.1', '--bind-ip', '127.0.0.1']
        if scenario == 'receive_only':
            bridge_args.append('--receive-only')
        if scenario == 'relayed_feedback':
            bridge_args += ['--feedback-source-ip', '127.0.0.2']
        bridge = subprocess.Popen(bridge_args,
                                  env=env, stdout=bridge_log, stderr=subprocess.STDOUT)
        controller.start()
        # 以日志中的启动标志等待 bridge 绑定端口，不假定固定启动耗时。
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            if bridge.poll() is not None:
                raise AssertionError('bridge 提前退出')
            if '9006' in (folder/(scenario+'_bridge.txt')).read_text(encoding='utf8'):
                break
            time.sleep(0.05)
        args = [sys.executable, '-m', 'estun_segmented.runner', '--config', str(config),
                '--log', str(folder/(scenario+'_motion.jsonl'))]
        single_stage = scenario in ('orientation', 'z', 'xy')
        if single_stage:
            delta = {'orientation': [0,0,0,0,0,math.radians(1)],
                     'z': [0,0,0.002,0,0,0], 'xy': [0.002,0.002,0,0,0,0]}[scenario]
            args = [sys.executable, '-m', 'estun_segmented.runner', '--delta',
                    *map(str, delta), '--stage', scenario,
                    '--log', str(folder/(scenario+'_motion.jsonl'))]
        if scenario != 'preview':
            args.append('--execute')
        result = subprocess.run(args, env=env, capture_output=True, text=True, timeout=25)
        (folder/(scenario+'_runner.txt')).write_text(result.stdout+result.stderr, encoding='utf8')
        if controller.error:
            raise controller.error
        assert not controller.failures, controller.failures
        records = [json.loads(line) for line in result.stdout.splitlines() if line.startswith('{')]
        if scenario in ('success', 'relayed_feedback') or single_stage:
            settings = Settings()
            target = [a+b for a, b in zip(START, delta)] if single_stage else TARGET
            expected = [command(point, settings) for point in
                        plan(START, target, settings, scenario if single_stage else 'all')]
            assert result.returncode == 0, result.stdout+result.stderr
            assert controller.commands == expected, 'UDP 实际命令与分段计划不同'
            assert len([r for r in records if r['event']=='ARRIVED']) == len(expected)
            assert [r['stage'] for r in records if r['event']=='SEND'] == (
                [scenario] if single_stage else ['orientation', 'z', 'xy'])
            assert [r['point_count'] for r in records if r['event']=='SEND'] == [len(c['points']) for c in expected]
            assert records[-1]['event'] == 'COMPLETE'
        elif scenario == 'preview':
            assert result.returncode == 0, result.stdout+result.stderr
            assert controller.commands == [], '预览模式发送了 UDP 运动指令'
        elif scenario == 'receive_only':
            assert result.returncode == 1, result.stdout+result.stderr
            assert controller.commands == [], '只读 bridge 转发了运动指令'
            assert records[-1]['event'] == 'ABORT', result.stdout
        else:
            assert result.returncode == 1, result.stdout+result.stderr
            assert len(controller.commands) == (2 if scenario == 'fault_z' else 1), '异常后发生继续发送或重发'
            assert records[-1]['event'] == 'ABORT', result.stdout
        summary = {'scenario': scenario, 'result': 'PASS', 'runner_exit': result.returncode,
                   'udp_command_count': len(controller.commands), 'simulated_controller': True}
        print(json.dumps(summary), flush=True)
        return summary
    finally:
        controller.close()
        if bridge:
            stop_process(bridge)
        bridge_log.close()


def main():
    summaries = []
    for i, scenario in enumerate(('preview', 'success', 'fault', 'no_ack', 'dropout',
                                  'wrong_pose', 'malformed', 'orientation', 'z', 'xy',
                                  'receive_only', 'relayed_feedback', 'fault_z')):
        summaries.append(run_scenario(scenario, 150+i))
    (ROOT/'logs/ros_loopback_phase/results.json').write_text(json.dumps(summaries, indent=2), encoding='utf8')


if __name__ == '__main__':
    main()
