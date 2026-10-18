"""纯 Python 本地测试；所有反馈均为测试数据，不代表真实机械臂结果。"""
from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path
import random
import socket
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/estun_segmented'))
from estun_segmented.core import (Completion, Feedback, MotionError, Segment,
                                 Settings, command, errors, euler_near, plan,
                                 quaternion, rotation_distance)
from estun_segmented.wire import decode_feedback, encode_command, to_dict


class PlanTests(unittest.TestCase):
    def setUp(self):
        self.start = (0.4, 0.1, 0.3, math.pi, 0, 0)
        self.target = (0.445, 0.14, 0.345, math.pi, 0, math.radians(17))
        self.s = Settings()

    def test_order_and_axis_invariants(self):
        segments = plan(self.start, self.target, self.s)
        self.assertEqual([s.stage for s in segments], ['orientation','z','xy'])
        previous = self.start
        for s in segments:
            if s.stage == 'orientation':
                self.assertEqual(s.pose[:3], self.start[:3])
                self.assertLess(rotation_distance(s.pose[3:], self.target[3:]), 1e-10)
            else:
                self.assertLess(rotation_distance(s.pose[3:], self.target[3:]), 1e-10)
                if s.stage == 'z':
                    self.assertEqual(s.pose[:2], self.start[:2])
                else:
                    self.assertEqual(s.pose[2], self.target[2])
                    # XY 点与起终点共线。
                    dx, dy = s.pose[0]-self.start[0], s.pose[1]-self.start[1]
                    self.assertAlmostEqual(dx*0.04-dy*0.045, 0)
            previous = s.pose
        self.assertLess(errors(segments[-1].pose, self.target)[0], 1e-12)

    def test_no_motion_and_negative_z(self):
        self.assertEqual(plan(self.start, self.start), [])
        target = self.start[:2]+(0.255,)+self.start[3:]
        segments = plan(self.start, target)
        self.assertEqual([s.stage for s in segments], ['z'])
        self.assertAlmostEqual(segments[-1].pose[2], 0.255)

    def test_each_phase_has_only_one_endpoint(self):
        segments = plan(self.start, self.target)
        for segment in segments:
            obj = command(segment, self.s)
            self.assertEqual(len(obj['points']), 1)
            self.assertEqual(obj['points'][0]['positions'], list(segment.pose))
            self.assertEqual(json.loads(encode_command(obj)), obj)

    def test_shortest_rotation_across_pi(self):
        a = (0, 0, 0, 0, 0, math.radians(179))
        b = (0, 0, 0, 0, 0, math.radians(-179))
        segments = plan(a, b)
        self.assertEqual(len(segments), 1)
        self.assertAlmostEqual(segments[-1].pose[5], math.radians(181))
        self.assertLess(rotation_distance(segments[-1].pose[3:], b[3:]), 1e-12)

    def test_large_moves_do_not_create_small_steps(self):
        target = (0.402, 0.102, 0.302, math.pi, 0, math.radians(1))
        self.assertEqual([s.stage for s in plan(self.start, target)], ['orientation','z','xy'])
        target = tuple(self.start[i]+0.1 for i in range(3))+(math.pi, math.pi/4, 0)
        segments = plan(self.start, target)
        self.assertEqual(len(segments), 3)
        self.assertEqual(segments[1].pose[2], target[2])
        self.assertEqual(segments[-1].pose[:3], target[:3])

    def test_general_rotation_and_gimbal_lock(self):
        rng = random.Random(2026)
        cases = [(rng.uniform(-6, 6), rng.uniform(-6, 6), rng.uniform(-6, 6)) for _ in range(1000)]
        cases += [(1.2, p, -2.4) for p in (math.pi/2, -math.pi/2)]
        for rpy in cases:
            rebuilt = euler_near(quaternion(rpy), (0.1, 0.2, 0.3))
            self.assertLess(rotation_distance(rebuilt, rpy), 3e-6)
        for b in [(0.8, 1.7, -2.2), (2.9, -1.8, 2.7)]:
            a = (0, 0, 0, 1.2, 1.3, 2.1)
            segments = plan(a, (0, 0, 0)+b)
            previous = a
            total = 0
            for s in segments:
                delta = rotation_distance(previous[3:], s.pose[3:])
                total += delta
                previous = s.pose
            self.assertAlmostEqual(total, rotation_distance(a[3:], b), places=7)

    def test_official_fixed_xyz_example(self):
        # 官方手册 20°,30°,45° 的 Rz Ry Rx 矩阵；独立构造矩阵核对四元数。
        r, p, y = map(math.radians, (20, 30, 45))
        w, x, qy, z = quaternion((r, p, y))
        self.assertAlmostEqual(1-2*(qy*qy+z*z), math.cos(y)*math.cos(p))
        self.assertAlmostEqual(2*(x*qy+w*z), math.sin(y)*math.cos(p))
        self.assertAlmostEqual(2*(x*z-w*qy), -math.sin(p))

    def test_invalid_input_and_single_stage(self):
        for target in [(0,)*5, (math.nan,)*6, (math.inf,)*6]:
            with self.assertRaises(ValueError): plan(self.start, target)
        for stage in ('orientation', 'z', 'xy'):
            with self.assertRaises(ValueError): plan(self.start, self.target, stage=stage)
        with self.assertRaises(ValueError): Settings(settle_samples=1)
        with self.assertRaises(ValueError): Settings(linear_speed=math.nan)


class CompletionTests(unittest.TestCase):
    def setUp(self):
        self.s = Settings()
        self.seg = Segment('z', (0.4, 0.1, 0.302, math.pi, 0, 0))
        self.m = Completion(self.seg, 0, self.s)

    def fb(self, t, state='AutoReady', pose=None, fault=0):
        return Feedback(pose or self.seg.pose, (0,)*6, state, fault, t)

    def test_running_ready_and_stable_pose_required(self):
        self.assertFalse(self.m.feed(self.fb(0.04)))
        self.assertFalse(self.m.feed(self.fb(0.08, 'AutoRunning')))
        self.assertFalse(self.m.feed(self.fb(0.12)))
        self.assertFalse(self.m.feed(self.fb(0.24)))
        self.assertTrue(self.m.feed(self.fb(0.36)))

    def test_ready_without_running_never_completes(self):
        for i in range(1, 75):
            self.assertFalse(self.m.feed(self.fb(i*0.04)))
        with self.assertRaisesRegex(MotionError, '未观察到'):
            self.m.tick(3.01)

    def test_running_at_target_never_completes(self):
        for t in (0.04, 0.08, 0.2, 0.4):
            self.assertFalse(self.m.feed(self.fb(t, 'AutoRunning')))

    def test_intermediate_ready_does_not_complete_phase(self):
        self.m.feed(self.fb(0.04, 'AutoRunning'))
        midpoint = list(self.seg.pose); midpoint[2] -= 0.001
        for t in (0.08, 0.2, 0.32):
            self.assertFalse(self.m.feed(self.fb(t, pose=tuple(midpoint))))
        self.assertFalse(self.m.feed(self.fb(0.4, 'AutoRunning')))
        self.assertFalse(self.m.feed(self.fb(0.44)))
        self.assertFalse(self.m.feed(self.fb(0.56)))
        self.assertTrue(self.m.feed(self.fb(0.68)))

    def test_pose_error_resets_stability(self):
        self.m.feed(self.fb(0.04, 'AutoRunning'))
        self.m.feed(self.fb(0.08))
        wrong = list(self.seg.pose); wrong[0] += 0.002
        self.assertFalse(self.m.feed(self.fb(0.16, pose=tuple(wrong))))
        self.assertFalse(self.m.feed(self.fb(0.2)))
        self.assertFalse(self.m.feed(self.fb(0.28)))
        self.assertTrue(self.m.feed(self.fb(0.44)))

    def test_fault_and_mode_changes_abort(self):
        for state, fault in [('AutoRunning', 1), ('Idle', 0), ('PowerOff', 0), ('Unknown', 0)]:
            with self.assertRaises(MotionError): self.m.feed(self.fb(0.04, state, fault=fault))

    def test_stale_gap_and_late_frame_abort(self):
        with self.assertRaisesRegex(MotionError, '反馈中断'): self.m.tick(0.51)
        with self.assertRaisesRegex(MotionError, '反馈中断'):
            self.m.feed(self.fb(0.6, 'AutoRunning'))

    def test_segment_timeout(self):
        m = Completion(self.seg, 0, replace(self.s, segment_timeout=4))
        for i in range(1, 101): m.feed(self.fb(i*0.04, 'AutoRunning'))
        with self.assertRaisesRegex(MotionError, '到位超时'): m.tick(4.1)

    def test_duplicate_feedback_and_bad_arrays(self):
        self.m.feed(self.fb(0.04, 'AutoRunning'))
        self.assertFalse(self.m.feed(self.fb(0.04)))
        with self.assertRaises(MotionError):
            self.m.feed(Feedback((0,)*5, (0,)*6, 'AutoReady', 0, 0.08))


class WireTests(unittest.TestCase):
    def test_official_json_exact_fields_and_shapes(self):
        from types import SimpleNamespace as NS
        obj = command(Segment('z', (0.4, 0, 0.3, math.pi, 0, 0)), Settings())
        msg = NS(joint_names=obj['joint_names'], points=[NS(**obj['points'][0])])
        self.assertEqual(to_dict(msg), obj)
        # 不导入 ROS：从官方文件 AST 提取未改动的 to_json 方法来比较序列化。
        import ast
        source = (ROOT/'official/codroid_bridge/codroid_node/codroid_io.py').read_text(encoding='utf8')
        tree = ast.parse(source)
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'to_json')
        namespace = {'json': json}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<official>', 'exec'), namespace)
        self.assertEqual(encode_command(obj), namespace['to_json'](None, msg).encode())
        self.assertEqual(len(obj['points']), 1)
        self.assertEqual(len(obj['points'][0]['velocities']), 2)

    def test_udp_loopback_wire_roundtrip(self):
        # 仅本机随机端口，不联系真实控制器。
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rx, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
            rx.bind(('127.0.0.1', 0)); rx.settimeout(1)
            obj = command(Segment('xy', (0.4, 0, 0.3, 0, 0, 0)), Settings())
            tx.sendto(encode_command(obj), rx.getsockname())
            self.assertEqual(json.loads(rx.recvfrom(65535)[0]), obj)
            feedback = {'joint_positions': [0]*6, 'end_positions': [0.4,0,0.3,0,0,0],
                        'state': 'AutoReady', 'fault_flag': 0}
            tx.sendto(json.dumps(feedback).encode(), rx.getsockname())
            self.assertEqual(decode_feedback(rx.recvfrom(65535)[0]), feedback)

    def test_feedback_relay_only_forwards_feedback_bytes(self):
        # 不接触现场地址；127.0.0.2 模拟 Windows，127.0.0.1 模拟 WSL。
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rx, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
            rx.bind(('127.0.0.1', 9006)); rx.settimeout(0.1)
            process = subprocess.Popen([sys.executable, '-u', str(ROOT/'windows_feedback_relay.py'),
                                        '--listen-ip','127.0.0.2','--wsl-ip','127.0.0.1',
                                        '--controller-ip','127.0.0.1'],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            try:
                data = json.dumps({'joint_positions':[0]*6, 'end_positions':[0]*6,
                                   'state':'AutoReady', 'fault_flag':0}).encode()
                import time
                deadline = time.monotonic()+5
                received = None
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        self.fail(process.stderr.read().decode())
                    tx.sendto(data, ('127.0.0.2',9006))
                    try:
                        received, _ = rx.recvfrom(65535)
                        break
                    except socket.timeout:
                        continue
                self.assertEqual(received, data)
            finally:
                process.terminate()
                process.wait(timeout=5)
                process.stderr.close()

    def test_bad_udp_and_command(self):
        for data in (b'no json', b'{}', b'[]', b'{"end_positions": [NaN]}'):
            with self.assertRaises((ValueError, KeyError, TypeError)): decode_feedback(data)
        obj = command(Segment('z', (0,)*6), Settings())
        obj['joint_names'][0] = 'Joint1'
        with self.assertRaises(ValueError): encode_command(obj)
        obj = command(Segment('z', (0,)*6), Settings())
        obj['points'] = obj['points']*2
        with self.assertRaisesRegex(ValueError, '每阶段'): encode_command(obj)
        obj = command(Segment('z', (0,)*6), Settings())
        obj['points'][0]['positions'][2] = math.nan
        with self.assertRaises(ValueError): encode_command(obj)
        obj = command(Segment('z', (0,)*6), Settings())
        obj['points'][0]['velocities'] = [0.01]*6
        with self.assertRaises(ValueError): encode_command(obj)

    def test_offline_cli_and_execute_guard(self):
        base = [sys.executable, '-m', 'estun_segmented.runner', '--start',
                '0.4','0.1','0.3','3.141592653589793','0','0', '--delta',
                '0.002','0.002','0.002','0','0','0.017453292519943295',
                '--log', str(ROOT/'logs/cli_test.jsonl')]
        import os
        env = {**os.environ, 'PYTHONPATH': str(ROOT/'ros2_ws/src/estun_segmented'), 'PYTHONIOENCODING': 'utf8'}
        result = subprocess.run(base, env=env, capture_output=True, text=True, encoding='utf8')
        self.assertEqual(result.returncode, 0, result.stderr+result.stdout)
        records = [json.loads(l) for l in result.stdout.splitlines()]
        self.assertEqual([r['stage'] for r in records if r['event']=='PREVIEW'], ['orientation','z','xy'])
        self.assertNotIn('SEND', [r['event'] for r in records])
        result = subprocess.run(base+['--execute'], env=env, capture_output=True)
        self.assertEqual(result.returncode, 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
