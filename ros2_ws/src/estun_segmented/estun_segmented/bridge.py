"""按官方 codroid_io.py 转发；不做底层规划、不重发 UDP 指令。"""
import argparse
import json
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from codroid_msgs.msg import RobotInfo

from .wire import decode_feedback, encode_command, to_dict


def qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


class CodroidIO(Node):
    def __init__(self, ip, bind_ip, timeout, receive_only=False, feedback_source_ip=None):
        super().__init__('estun_bridge')
        self.address = (socket.gethostbyname(ip), 9005)
        self.feedback_source_ip = socket.gethostbyname(feedback_source_ip or ip)
        self.timeout = timeout
        self.receive_only = receive_only
        self.rx_time, self.latest = None, None
        self.bad = None
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.server_socket.bind((bind_ip, 9006))
        except OSError:
            self.client_socket.close()
            self.server_socket.close()
            raise
        self.server_socket.setblocking(False)
        self.publisher = self.create_publisher(RobotInfo, '/RobotInfo', qos())
        self.status = self.create_publisher(String, '/estun_bridge/status', qos())
        self.subscription = self.create_subscription(
            JointTrajectory, '/RobotMove', self.listener_callback, qos())
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.status_timer = self.create_timer(0.1, self.publish_status)
        self.get_logger().info(f'官方 UDP: 控制器 {self.address}，本机 {bind_ip}:9006')

    def fail(self, reason):
        # 通信异常锁定本次 bridge，避免后续正常包掩盖异常；人工排查后重启。
        if self.bad is None:
            self.bad = reason
            self.get_logger().error(reason)
        self.publish_status()

    def publish_status(self):
        age = None if self.rx_time is None else time.monotonic()-self.rx_time
        msg = String()
        msg.data = json.dumps({'error': self.bad, 'feedback_age': age})
        self.status.publish(msg)

    def timer_callback(self):
        latest = None
        try:
            # 取最新包，避免官方每次只读一包导致接收队列积压。
            for _ in range(256):
                try:
                    data, source = self.server_socket.recvfrom(65535)
                except BlockingIOError:
                    break
                if source[0] != self.feedback_source_ip:
                    continue
                latest = decode_feedback(data)
                if latest['fault_flag'] != 0:
                    self.fail(f"控制器故障 fault_flag={latest['fault_flag']}，请查看示教器并人工处理后重启 bridge")
            else:
                self.fail('UDP 接收队列过载，不能保证反馈新鲜度')
                return
            if latest is not None:
                self.latest, self.rx_time = latest, time.monotonic()
                msg = RobotInfo()
                # 官方消息中的 header 原示例未填写；本程序填写本机收包时间供接收者查延迟。
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.joint_positions = [float(v) for v in latest['joint_positions']]
                msg.end_positions = [float(v) for v in latest['end_positions']]
                msg.state, msg.fault_flag = latest['state'], latest['fault_flag']
                self.publisher.publish(msg)
        except (OSError, ValueError, TypeError, KeyError, OverflowError) as exc:
            self.fail(f'RobotInfo UDP 解析/接收失败：{exc}')

    def listener_callback(self, msg):
        try:
            if self.receive_only:
                raise ValueError('当前为只读通信验证模式')
            if self.bad:
                raise ValueError(self.bad)
            if self.rx_time is None or time.monotonic()-self.rx_time > self.timeout:
                raise ValueError('RobotInfo 缺失或过期')
            if self.latest['state'] != 'AutoReady' or self.latest['fault_flag'] != 0:
                raise ValueError('控制器不是无故障 AutoReady')
            payload = encode_command(to_dict(msg))
            self.client_socket.sendto(payload, self.address)  # 每段只发送这一次。
            self.get_logger().info('UDP_SEND '+payload.decode())
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.fail(f'拒绝/发送 RobotMove 失败：{exc}')

    def destroy_node(self):
        self.server_socket.close()
        self.client_socket.close()
        return super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller-ip', default='192.168.101.100')
    parser.add_argument('--bind-ip', default='0.0.0.0')
    parser.add_argument('--feedback-timeout', type=float, default=0.5)
    parser.add_argument('--receive-only', action='store_true', help='只接收 RobotInfo，拒绝转发运动')
    parser.add_argument('--feedback-source-ip', help='仅用于 NAT 反馈转发时指定 Windows 网关 IP；默认控制器 IP')
    options, ros_args = parser.parse_known_args(args)
    if not 0 < options.feedback_timeout < float('inf'):
        parser.error('feedback-timeout 必须为有限正数')
    rclpy.init(args=ros_args)
    node = None
    try:
        node = CodroidIO(options.controller_ip, options.bind_ip, options.feedback_timeout,
                         options.receive_only, options.feedback_source_ip)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
