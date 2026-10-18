"""复用 codroid_io.py 的 JSON 字段；增加有限数值检查。"""
import json
from .core import finite


def decode_feedback(data):
    obj = json.loads(data.decode('utf-8'))
    if not isinstance(obj, dict):
        raise ValueError('UDP 顶层必须是 JSON 对象')
    if not finite(obj['joint_positions'], 6) or not finite(obj['end_positions'], 6):
        raise ValueError('UDP 关节/末端必须为六个有限数值')
    if not isinstance(obj['state'], str) or type(obj['fault_flag']) is not int:
        raise ValueError('UDP 状态/故障字段类型错误')
    if not -128 <= obj['fault_flag'] <= 127:
        raise ValueError('fault_flag 超出官方 int8 范围')
    return obj


def to_dict(msg):
    # 字段和官方 CodroidIO.to_json 完全一致；不发送 header、effort、时间。
    return {'joint_names': list(msg.joint_names), 'points': [
        {'positions': list(p.positions), 'velocities': list(p.velocities),
         'accelerations': list(p.accelerations)} for p in msg.points]}


def validate_command(obj):
    if obj['joint_names'] != ['x', 'y', 'z', 'rx', 'ry', 'rz'] or len(obj['points']) != 1:
        raise ValueError('本程序每阶段仅允许一个末端目标点')
    for p in obj['points']:
        if not finite(p['positions'], 6):
            raise ValueError('末端目标必须是六个有限数值')
        for key in ('velocities', 'accelerations'):
            if not finite(p[key], 2) or any(v <= 0 for v in p[key]):
                raise ValueError(f'{key} 必须是两个有限正数（平移、旋转）')


def encode_command(obj):
    validate_command(obj)
    payload = json.dumps(obj, allow_nan=False).encode('utf-8')
    if len(payload) > 65507:
        raise ValueError('运动指令超过 IPv4 UDP 单包上限，本次不发送')
    return payload
