"""默认只预览；--execute 明确执行，--read 只读取真实反馈。"""
import argparse
from dataclasses import fields
import json
import math
from pathlib import Path
import sys
import time

from .core import (Completion, Feedback, MotionError, Settings, command, errors,
                   finite, plan)
from .wire import encode_command


class Journal:
    def __init__(self, path):
        self.stream = None
        if path:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            self.stream = open(path, 'a', encoding='utf-8')

    def write(self, event, **data):
        record = {'time': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                  'monotonic': time.monotonic(), 'event': event, **data}
        line = json.dumps(record, ensure_ascii=False, allow_nan=False)
        print(line, flush=True)
        if self.stream:
            self.stream.write(line+'\n')
            self.stream.flush()

    def close(self):
        if self.stream:
            self.stream.close()


def make_node(journal, settings):
    # 离线预览不需要导入 ROS，现场执行使用真实 rclpy 和官方消息包。
    from rclpy.node import Node
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from codroid_msgs.msg import RobotInfo
    from .bridge import qos

    class MotionNode(Node):
        def __init__(self):
            super().__init__('estun_segmented_move')
            self.feedback, self.bad, self.monitor = None, None, None
            self.done = False
            self.status_time = None
            self.publisher = self.create_publisher(JointTrajectory, '/RobotMove', qos())
            self.subscription = self.create_subscription(RobotInfo, '/RobotInfo', self.on_info, qos())
            self.status_sub = self.create_subscription(String, '/estun_bridge/status', self.on_status, qos())
            self.last_state = None

        def on_status(self, msg):
            try:
                data = json.loads(msg.data)
                self.status_time = time.monotonic()
                if data['error']:
                    self.bad = MotionError('通信 bridge 异常：'+str(data['error']))
                age = data['feedback_age']
                if age is not None and (not math.isfinite(age) or age > settings.feedback_timeout):
                    self.bad = MotionError('bridge 报告控制器反馈过期')
            except (ValueError, KeyError, TypeError) as exc:
                self.bad = MotionError(f'bridge 状态消息异常：{exc}')

        def on_info(self, msg):
            try:
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
                age = self.get_clock().now().nanoseconds*1e-9-stamp
                if not -0.05 <= age <= settings.feedback_timeout:
                    raise MotionError('RobotInfo 时间戳过期/异常，请使用配套 bridge 并检查时钟')
                feedback = Feedback(tuple(msg.end_positions), tuple(msg.joint_positions),
                                    msg.state, msg.fault_flag, time.monotonic())
                if not finite(feedback.pose, 6) or not finite(feedback.joints, 6):
                    raise MotionError('RobotInfo 数组非法')
                self.feedback = feedback
                if self.last_state != (feedback.state, feedback.fault):
                    journal.write('STATE', state=feedback.state, fault_flag=feedback.fault)
                    self.last_state = feedback.state, feedback.fault
                if self.monitor is not None and not self.done:
                    self.done = self.monitor.feed(feedback)
            except (MotionError, ValueError, TypeError) as exc:
                self.bad = MotionError(str(exc))

        def spin(self):
            import rclpy
            from rclpy.executors import ExternalShutdownException
            try:
                rclpy.spin_once(self, timeout_sec=0.02)
            except ExternalShutdownException as exc:
                raise MotionError('ROS 上下文被外部关闭') from exc
            if self.bad:
                raise self.bad
            if not rclpy.ok():
                raise MotionError('ROS 已关闭')

        def current(self):
            now = time.monotonic()
            if self.feedback is None or now-self.feedback.received > settings.feedback_timeout:
                raise MotionError('RobotInfo 缺失或过期')
            if self.status_time is None or now-self.status_time > settings.feedback_timeout:
                raise MotionError('配套 bridge 状态缺失或过期')
            return self.feedback

        def connected(self):
            return (self.publisher.get_subscription_count() == 1 and
                    self.count_publishers('/RobotInfo') == 1 and
                    self.count_publishers('/estun_bridge/status') == 1 and
                    self.count_publishers('/RobotMove') == 1)

        def read_start(self, require_ready=True, timeout=10.0):
            deadline, since, samples, anchor = time.monotonic()+timeout, None, 0, None
            while time.monotonic() < deadline:
                old = self.feedback
                self.spin()
                if self.feedback is None or self.feedback is old or self.status_time is None:
                    continue
                fb = self.current()
                if not require_ready:
                    return fb
                fb.check()
                if fb.state != 'AutoReady' or not self.connected():
                    since, samples, anchor = None, 0, None
                    continue
                if anchor is None or not self.close_pose(fb.pose, anchor):
                    since, samples, anchor = fb.received, 0, fb.pose
                samples += 1
                if samples >= settings.settle_samples and fb.received-since >= settings.settle_seconds:
                    return fb
            if self.feedback is None:
                raise MotionError('启动等待超时：没有收到 RobotInfo，请检查控制器 ROS 反馈 IP/9006 和网络')
            raise MotionError('启动等待超时：需要稳定 AutoReady、唯一 bridge 和新鲜 RobotInfo')

        def close_pose(self, a, b):
            p, r = errors(a, b)
            return p <= settings.position_tolerance and r <= settings.angle_tolerance

        def move_to(self, target=None, stage='all', execute=False, delta=None):
            """视觉接入点：target 为与 RobotInfo 同坐标系的 [x,y,z,rx,ry,rz]。"""
            start = self.read_start()
            if delta is not None:
                # 相对目标与分段起点必须使用同一帧，避免两次读取间的微小反馈变化。
                target = [a+b for a, b in zip(start.pose, delta)]
            journal.write('START', pose=start.pose, joints=start.joints,
                          state=start.state, target=target, settings=vars(settings))
            segments = plan(start.pose, target, settings, stage)
            preview(journal, segments, settings)
            if not execute:
                journal.write('PREVIEW_ONLY', segments=len(segments))
                return
            expected = start.pose
            for i, segment in enumerate(segments, 1):
                # 排空已经就绪的回调，随后再次核对起点；绝不把预览时的旧起点直接执行。
                self.spin()
                fb = self.current()
                fb.check()
                if fb.state != 'AutoReady' or not self.connected():
                    raise MotionError('发送前状态/ROS 通信连接发生变化')
                if not self.close_pose(fb.pose, expected):
                    raise MotionError('实际起点偏离上一段终点/预览起点，请重新规划')
                obj = command(segment, settings)
                msg = JointTrajectory()
                msg.joint_names = obj['joint_names']
                for values in obj['points']:
                    point = JointTrajectoryPoint()
                    for name, data in values.items():
                        setattr(point, name, data)
                    msg.points.append(point)
                self.done = False
                self.monitor = Completion(segment, time.monotonic(), settings)
                journal.write('SEND', index=i, stage=segment.stage, point_count=1, command=obj)
                self.publisher.publish(msg)  # 当前阶段只发一个终点，控制器负责整段插补。
                try:
                    while not self.done:
                        self.spin()
                        self.monitor.tick(time.monotonic())
                        self.current()
                        if not self.connected():
                            raise MotionError('运动中 ROS 连接断开或出现其他运动发布者')
                    p, a = errors(self.feedback.pose, segment.pose)
                    journal.write('ARRIVED', index=i, stage=segment.stage, point_count=1,
                                  actual=self.feedback.pose, joints=self.feedback.joints,
                                  position_error_m=p, angle_error_rad=a)
                finally:
                    self.monitor = None
                expected = segment.pose
            journal.write('COMPLETE', phases=len(segments), segments=len(segments), actual=self.current().pose)

    return MotionNode()


def preview(journal, segments, settings):
    commands = [(s, command(s, settings)) for s in segments]
    for _, obj in commands:
        encode_command(obj)
    for stage in ('orientation', 'z', 'xy'):
        journal.write('PLAN_STAGE', stage=stage,
                      segments=sum(s.stage == stage for s in segments))
    for i, (segment, obj) in enumerate(commands, 1):
        journal.write('PREVIEW', index=i, stage=segment.stage, point_count=1, command=obj)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--target', nargs=6, type=float, metavar=('X','Y','Z','RX','RY','RZ'))
    source.add_argument('--config', help='JSON 配置文件（target、速度、阶段超时等）')
    source.add_argument('--delta', nargs=6, type=float,
                        help='相对当前反馈的测试增量（m、rad）；姿态为单个欧拉分量增量')
    source.add_argument('--read', action='store_true', help='只读取一次真实 RobotInfo')
    parser.add_argument('--start', nargs=6, type=float, help='仅离线预览使用的起点（m、rad）')
    parser.add_argument('--stage', choices=['all','orientation','z','xy'], default='all')
    parser.add_argument('--linear-speed', type=float, help='m/s')
    parser.add_argument('--angular-speed', type=float, help='rad/s')
    parser.add_argument('--phase-timeout', '--segment-timeout', dest='segment_timeout', type=float,
                        help='整个阶段到位超时（秒），默认 120；兼容旧参数名')
    parser.add_argument('--execute', action='store_true', help='明确发送真实运动指令')
    parser.add_argument('--log', default='logs/motion.jsonl', help='JSONL 日志路径，追加写入')
    options = parser.parse_args(args)
    if options.execute and (options.start is not None or options.read):
        parser.error('--execute 不可与 --start/--read 同用')
    if options.read and options.start is not None:
        parser.error('--read 不可与 --start 同用')
    if not options.read and options.target is None and not options.config and options.delta is None:
        parser.error('需要 --target、--config、--delta 或 --read')
    journal = Journal(options.log)
    node, rclpy = None, None
    try:
        config = {}
        if options.config:
            with open(options.config, encoding='utf-8') as f:
                config = json.load(f)
            if not isinstance(config, dict):
                raise ValueError('配置必须为 JSON 对象')
        target = options.target if options.target is not None else config.get('target')
        values = {k: v for k, v in config.items() if k != 'target'}
        allowed = {f.name for f in fields(Settings)}
        if set(values)-allowed:
            raise ValueError(f'未知配置参数：{set(values)-allowed}')
        for name in ('linear_speed', 'angular_speed', 'segment_timeout'):
            if getattr(options, name) is not None:
                values[name] = getattr(options, name)
        settings = Settings(**values)
        if target is not None and not finite(target, 6):
            raise ValueError('target 必须为六个有限数值（m、rad）')
        if options.delta is not None:
            if not finite(options.delta, 6):
                raise ValueError('delta 必须为六个有限数值')
            if sum(v != 0 for v in options.delta[3:]) > 1:
                raise ValueError('相对姿态测试仅允许一个欧拉分量增量；一般目标请使用 --target')
        if options.start is not None:
            start = options.start
            if options.delta is not None:
                target = [a+b for a, b in zip(start, options.delta)]
            journal.write('OFFLINE_START', pose=start, target=target, settings=vars(settings))
            segments = plan(start, target, settings, options.stage)
            preview(journal, segments, settings)
            journal.write('PREVIEW_ONLY', segments=len(segments), source='offline')
        else:
            import rclpy
            rclpy.init(args=[])
            node = make_node(journal, settings)
            if options.read:
                fb = node.read_start(require_ready=False)
                journal.write('ROBOT_INFO', pose=fb.pose, joints=fb.joints,
                              state=fb.state, fault_flag=fb.fault)
            else:
                node.move_to(target, stage=options.stage, execute=options.execute, delta=options.delta)
    except (MotionError, ValueError, TypeError, KeyError, OSError, ImportError, OverflowError) as exc:
        journal.write('ABORT', reason=str(exc), note='已停止后续发送；不自动重发；本接口没有急停/取消指令')
        return 1
    except KeyboardInterrupt:
        journal.write('ABORT', reason='人工中断', note='只停止后续发送；已接收的当前段可能继续，请使用硬件停止/急停')
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        journal.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
