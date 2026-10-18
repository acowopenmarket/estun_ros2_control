"""WSL NAT 专用：Windows UDP 9006 → WSL UDP 9006，只转发机器人反馈。"""
import argparse
import json
import socket


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wsl-ip', required=True, help='WSL 内 hostname -I 查到的 eth0 IPv4')
    parser.add_argument('--listen-ip', default='192.168.101.151')
    parser.add_argument('--controller-ip', default='192.168.101.100')
    args = parser.parse_args()
    # 只使用数据报套接字，不存在向控制器 9005 发送运动的路径。
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rx, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
        rx.bind((args.listen_ip, 9006))
        rx.settimeout(5)
        print(f'只转发反馈 {args.controller_ip} → {args.listen_ip}:9006 → {args.wsl_ip}:9006', flush=True)
        count = 0
        while True:
            try:
                data, source = rx.recvfrom(65535)
            except socket.timeout:
                print('5 秒未收到 Windows 侧 UDP 反馈；检查 ROS 开关、目标 IP 和防火墙', flush=True)
                continue
            if source[0] != args.controller_ip:
                continue
            # 解析字段后仍原样转发字节，不改变官方协议。
            try:
                obj = json.loads(data)
                if not all(k in obj for k in ('joint_positions','end_positions','state','fault_flag')):
                    raise ValueError('缺少 RobotInfo 字段')
            except (ValueError, TypeError) as exc:
                print(f'反馈异常，退出：{exc}', flush=True)
                return 1
            tx.sendto(data, (args.wsl_ip, 9006))
            count += 1
            if count == 1 or count % 250 == 0:
                print(f'已转发 {count} 帧，state={obj["state"]}，fault_flag={obj["fault_flag"]}', flush=True)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
