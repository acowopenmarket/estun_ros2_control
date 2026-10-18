# 埃斯顿 ROS2 三阶段机械臂控制

适用于埃斯顿 S3-60 Pro、Ubuntu 22.04 和 ROS2 Humble。使用官方 RobotMove/RobotInfo 接口，按以下顺序执行末端笛卡尔直线运动：

1. **姿态**：保持当前 XYZ，调整至目标姿态。
2. **Z**：保持 XY 和姿态，移动至目标 Z。
3. **XY**：保持 Z 和姿态，沿直线到目标 XY。

**每个大阶段只发一个终点，共最多三条运动指令；没有 20 mm、5° 等小步细分，也不批量下发小步目标。**阶段末确认到位后才进入下一阶段，无需移动的阶段自动跳过。运动规划和插补交给控制器，不使用关节 PTP。

默认只预览，添加 `--execute` 才会发送真实运动指令。

## 目录

```text
ros2_ws/src/codroid_msgs/       官方 RobotInfo 消息包
ros2_ws/src/estun_segmented/    控制程序
  estun_segmented/core.py      三阶段目标计算、旋转换算、到位判定
  estun_segmented/bridge.py    官方 UDP/JSON 与 ROS2 的转发
  estun_segmented/runner.py    输入、预览、执行、JSONL 日志
  config/target.json          示例目标配置
windows_feedback_relay.py     WSL NAT 环境下的反馈转发工具
tests/                       单元测试、ROS2 模拟通信测试、只读现场探测
official/                    用于协议兼容测试的官方通信参考代码
build.sh                     构建，不启动运动
test_local.sh                本机模拟测试，不连接实机
probe_info.sh                只读探测真实反馈
```

## 安装与构建

先安装 ROS2 Humble、colcon 和 Python 3。在 Ubuntu 或 WSL Ubuntu 22.04 的 Bash 中执行：

```bash
git clone https://github.com/acowopenmarket/estun_ros2_control.git
cd estun_ros2_control
bash build.sh
bash test_local.sh
```

每个终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
```

所有机器人控制节点应使用同一 ROS_DOMAIN_ID。测试脚本使用独立 Domain 和 `127.0.0.1` 模拟控制器；运行测试前关闭占用 UDP 9005/9006 的 bridge。

## 官方协议与单位

| 项目 | 定义 |
|---|---|
| 默认控制器 | `192.168.101.100`，UDP 9005 接收运动 |
| 上位机状态接收 | UDP 9006；控制器 Web「设置 → ROS」配置上位机可达 IP |
| `/RobotInfo` | `codroid_msgs/msg/RobotInfo`，六关节、末端位姿、state、fault_flag |
| `/RobotMove` | `trajectory_msgs/msg/JointTrajectory` |
| 末端分量 | `x,y,z,rx,ry,rz`，单位 **m、rad** |
| 速度 | `[平移 m/s, 旋转 rad/s]`，必须为两个数 |
| 加速度 | `[平移 m/s², 旋转 rad/s²]`，必须为两个数 |
| 执行状态 | 无故障 `AutoReady` 才能接收；执行中为 `AutoRunning` |

姿态采用右手系 X-Y-Z 固定轴欧拉角，矩阵为 `Rz(rz) Ry(ry) Rx(rx)`。来源为[酷卓 S 系列官方用户手册](https://www.codroid.ai/download/shouce/v1107.pdf)第 9.2 节。程序选择邻近等价欧拉角发送终点；整个阶段的姿态插补由官方控制器完成。

协议 rx/ry/rz 不是绕当前法兰局部轴的增量。绕法兰局部 Y 轴转动 θ 应使用 `R目标 = R当前 · Ry(θ)` 换算最终姿态。输入坐标、参考系和 TCP 必须与 RobotInfo 一致；夹爪偏置应通过正确 TCP 或上层坐标换算处理。

## 启动通信与读取反馈

终端 A，只读 bridge：

```bash
ros2 run estun_segmented bridge --receive-only
```

终端 B：

```bash
ros2 topic echo /RobotInfo --once
ros2 run estun_segmented move --read
```

确认能够读取真实数据后，关闭只读 bridge，执行需要运动的任务时改用：

```bash
ros2 run estun_segmented bridge --controller-ip 192.168.101.100
```

bridge 本身不生成运动目标。不要同时运行多个 bridge 或其他 RobotMove 发布程序。

### WSL NAT 的反馈通路

控制器若向 Windows 网卡发送 UDP，而 ROS2 在 WSL 内，需要把反馈转入 WSL。先在 WSL 执行 `ip -4 addr show eth0` 和 `ip route show default`，分别查到 WSL IP 和 Windows 网关 IP。

Windows PowerShell（示例地址需按现场修改）：

```powershell
python .\windows_feedback_relay.py --listen-ip 192.168.101.151 --wsl-ip 172.25.106.226
```

WSL 的 bridge 增加反馈来源参数：

```bash
ros2 run estun_segmented bridge --feedback-source-ip 172.25.96.1
```

转发工具只接收机器人反馈，不向控制器发送运动指令。WSL 重启后 IP 可能改变。原生 Ubuntu 或已直接收到反馈时无需此工具。只读一键探测：

```bash
bash probe_info.sh --feedback-source-ip 172.25.96.1
```

## 输入目标与预览

绝对目标格式（示例坐标不代表现场可达）：

```bash
ros2 run estun_segmented move --target 0.4 0.1 0.3 3.141592653589793 0 0
```

配置文件方式，先修改示例文件中的目标：

```bash
ros2 run estun_segmented move --config ros2_ws/src/estun_segmented/config/target.json
```

离线预览，无需反馈，不允许与 `--execute` 同用：

```bash
ros2 run estun_segmented move --start 0.4 0.1 0.3 3.141592653589793 0 0 --target 0.5 0.2 0.4 3.141592653589793 0.7853981634 0
```

相对目标便利接口 `--delta dx dy dz drx dry drz` 使用 m、rad，与规划起点使用同一帧反馈。姿态增量仅允许一个固定轴欧拉分量；它不是工具局部轴旋转接口。

## 现场测试与执行

先核对坐标单位、参考系、TCP、运动区域和夹爪扫掠区域，确认现场急停可用。含 `--execute` 的命令会操控真实机械臂，需由现场操作者明确运行。

先分别预览以下三个目标，再逐项添加 `--execute` 测试：

```bash
ros2 run estun_segmented move --stage orientation --delta 0 0 0 0 0 0.01745329252
ros2 run estun_segmented move --stage z --delta 0 0 0.002 0 0 0
ros2 run estun_segmented move --stage xy --delta 0.002 0.002 0 0 0 0
```

各阶段确认成功后，预览并人工执行完整三阶段任务：

```bash
# 预览：最多三个大阶段
ros2 run estun_segmented move --delta 0.002 0.002 0.002 0 0 0.01745329252
# 执行同类相对目标，每次重新读取起点
ros2 run estun_segmented move --delta 0.002 0.002 0.002 0 0 0.01745329252 --linear-speed 0.002 --angular-speed 0.01745329252 --execute --log logs/site_all.jsonl
```

默认速度为 5 mm/s、2°/s。`--phase-timeout` 设置整个阶段超时，默认 120 秒；兼容旧参数名 `--segment-timeout`。没有小步长度/角度参数。

## 反馈与故障

每阶段需观察 AutoRunning，再回到 AutoReady，并且终点位置误差不超过 0.5 mm、姿态误差不超过 0.5°，连续稳定至少 0.2 秒和 3 帧，才进入下一阶段。运行中持续检查反馈新鲜度、状态、fault_flag 和 ROS 连接。

出现异常、拒绝/未接收指令、不到位或超时时，停止后续发送，不自动重发、不自动切换模式。协议没有停止/取消接口，已接收的当前阶段可能继续，必要时使用现场控制器停止或急停。具体逆解、奇异点和限位原因需查看示教器，不能仅凭超时断言原因。

JSONL 日志位于 `logs/`，不纳入版本管理。`PREVIEW`、`SEND`、`ARRIVED` 分别表示预览、当前阶段发布、当前阶段到位，`COMPLETE` 表示全部阶段完成，`ABORT` 表示退出。

## 视觉接入

上层先把视觉结果变换到 RobotInfo 的参考系和 TCP，转换为 m、rad，再调用 `runner.make_node(journal, settings)` 创建节点及 `node.move_to(target, execute=True)`。在上层主循环中调用，不要在同一节点的订阅回调里阻塞调用或另外启动并发 spin。

`core.plan(start, target, settings)` 返回最多三个阶段终点。运行前应先预览；测试结果与边界见 [VALIDATION.md](VALIDATION.md)。
