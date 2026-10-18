"""与 ROS 解耦的分段和反馈判定；长度为 m，角度为 rad。"""
from dataclasses import dataclass
import math


class MotionError(RuntimeError):
    pass


def finite(values, length):
    return len(values) == length and all(
        isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
        for v in values)


def quaternion(rpy):
    # 官方 X-Y-Z 固定角：R = Rz(rz) Ry(ry) Rx(rx)，四元数顺序为 w,x,y,z。
    cr, cp, cy = [math.cos(v / 2) for v in rpy]
    sr, sp, sy = [math.sin(v / 2) for v in rpy]
    return (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)


def rotation_distance(a, b):
    qa, qb = quaternion(a), quaternion(b)
    # 相同旋转的 q 和 -q 等价，atan2 在小角度下比 acos 更稳定。
    dot = sum(x*y for x, y in zip(qa, qb))
    sign = 1 if dot >= 0 else -1
    chord = math.sqrt(sum((x-sign*y)**2 for x, y in zip(qa, qb)))
    return 4 * math.asin(min(1.0, chord / 2))


def euler_near(q, previous):
    """选择与上一目标最接近的等价欧拉角，避免跨 ±π 时数值跳变。"""
    w, x, y, z = q
    sin_pitch = max(-1, min(1, 2*(w*y-z*x)))
    if abs(sin_pitch) > 1 - 1e-12:
        # 万向节锁处固定上一个 roll，解出 yaw；不把三轴分别线性插值。
        pitch = math.copysign(math.pi/2, sin_pitch)
        roll = previous[0]
        delta = math.atan2(-2*(x*y-w*z), 1-2*(x*x+z*z))
        yaw = delta + math.copysign(1, sin_pitch)*roll
    else:
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(sin_pitch)
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    candidates = [(roll, pitch, yaw), (roll+math.pi, math.pi-pitch, yaw+math.pi)]
    def near(values):
        return tuple(v + 2*math.pi*round((p-v)/(2*math.pi))
                     for v, p in zip(values, previous))
    return min(map(near, candidates),
               key=lambda r: sum((a-b)**2 for a, b in zip(r, previous)))


@dataclass(frozen=True)
class Settings:
    linear_speed: float = 0.005
    angular_speed: float = math.radians(2)
    linear_accel: float = 0.02
    angular_accel: float = math.radians(5)
    position_tolerance: float = 0.0005
    angle_tolerance: float = math.radians(0.5)
    feedback_timeout: float = 0.5
    accept_timeout: float = 3.0
    segment_timeout: float = 120.0  # 当前整个阶段的超时；保留旧配置字段名。
    settle_seconds: float = 0.2
    settle_samples: int = 3

    def __post_init__(self):
        for name, value in vars(self).items():
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} 必须是有限正数')
        if not isinstance(self.settle_samples, int) or self.settle_samples < 2:
            raise ValueError('settle_samples 必须是至少 2 的整数')
        if self.accept_timeout >= self.segment_timeout:
            raise ValueError('accept_timeout 必须小于 segment_timeout')


@dataclass(frozen=True)
class Segment:
    stage: str
    pose: tuple


def errors(actual, target):
    return math.dist(actual[:3], target[:3]), rotation_distance(actual[3:], target[3:])


def plan(start, target, settings=Settings(), stage='all'):
    """视觉系统可直接调用此接口传入同一坐标系的六维目标位姿。"""
    if not finite(start, 6) or not finite(target, 6):
        raise ValueError('起点和目标必须是六个有限数值（m、rad）')
    if stage not in ('all', 'orientation', 'z', 'xy'):
        raise ValueError('未知阶段')
    # 单阶段模式不能静默丢弃其他方向上的目标变化。
    pos_error, angle = errors(start, target)
    if stage == 'orientation' and pos_error > settings.position_tolerance:
        raise ValueError('姿态单阶段要求目标 XYZ 与起点相同')
    if stage in ('z', 'xy') and angle > settings.angle_tolerance:
        raise ValueError('平移单阶段要求目标姿态与起点相同')
    if stage == 'z' and math.dist(start[:2], target[:2]) > settings.position_tolerance:
        raise ValueError('Z 单阶段要求目标 XY 与起点相同')
    if stage == 'xy' and abs(start[2]-target[2]) > settings.position_tolerance:
        raise ValueError('XY 单阶段要求目标 Z 与起点相同')
    # 只生成三个大阶段的终点，不再按距离或角度拆成小步。
    result, current = [], tuple(start)
    if stage in ('all', 'orientation') and angle > settings.angle_tolerance:
        rpy = euler_near(quaternion(target[3:]), start[3:])
        result.append(Segment('orientation', tuple(start[:3])+rpy))
        current = result[-1].pose
    if stage in ('all', 'z') and abs(target[2]-current[2]) > settings.position_tolerance:
        result.append(Segment('z', current[:2]+(target[2],)+current[3:]))
        current = result[-1].pose
    if stage in ('all', 'xy') and math.dist(current[:2], target[:2]) > settings.position_tolerance:
        result.append(Segment('xy', tuple(target[:2])+current[2:]))
    return result


def command(segment, settings):
    # 官方协议：笛卡尔速度/加速度各为两个数，不是六个数。
    return {'joint_names': ['x', 'y', 'z', 'rx', 'ry', 'rz'], 'points': [{
        'positions': list(segment.pose),
        'velocities': [settings.linear_speed, settings.angular_speed],
        'accelerations': [settings.linear_accel, settings.angular_accel]}]}


@dataclass(frozen=True)
class Feedback:
    pose: tuple
    joints: tuple
    state: str
    fault: int
    received: float

    def check(self):
        if not finite(self.pose, 6) or not finite(self.joints, 6):
            raise MotionError('RobotInfo 位姿/关节数组异常')
        if self.fault != 0:
            raise MotionError(f'控制器故障 fault_flag={self.fault}，请查看示教器错误详情')
        if self.state not in ('AutoReady', 'AutoRunning'):
            raise MotionError(f'控制器状态不允许运动：{self.state}')


class Completion:
    """只确认当前大阶段的终点，控制器负责阶段内部插补。"""
    def __init__(self, segment, sent_at, settings):
        self.segment, self.sent_at, self.s = segment, sent_at, settings
        self.running = False
        self.last_received = sent_at
        self.stable_since = None
        self.samples = 0

    def tick(self, now):
        if now-self.last_received > self.s.feedback_timeout:
            raise MotionError('RobotInfo 反馈中断；停止后续指令')
        if not self.running and now-self.sent_at > self.s.accept_timeout:
            raise MotionError('未观察到 AutoRunning：指令可能丢失、被拒绝、逆解/规划失败或运动过短；不重发')
        if now-self.sent_at > self.s.segment_timeout:
            raise MotionError('当前阶段运动/到位超时；不重发')

    def feed(self, feedback):
        feedback.check()
        if feedback.received <= self.last_received:
            return False
        # 先检验与上一帧间隔，不能让迟来的帧掩盖反馈中断。
        self.tick(feedback.received)
        self.last_received = feedback.received
        if feedback.state == 'AutoRunning':
            self.running = True
        pos, ang = errors(feedback.pose, self.segment.pose)
        if self.running and feedback.state == 'AutoReady' and (
                pos <= self.s.position_tolerance and ang <= self.s.angle_tolerance):
            if self.stable_since is None:
                self.stable_since = feedback.received
            self.samples += 1
            return self.samples >= self.s.settle_samples and (
                feedback.received-self.stable_since >= self.s.settle_seconds)
        self.stable_since, self.samples = None, 0
        return False
