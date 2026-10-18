# 官方代码来源

- `ros2_ws/src/codroid_msgs/` 中的消息定义、CMakeLists.txt 和 package.xml 来自比赛方提供的 `codroid_bridge.zip`，保持原样。
- `official/codroid_bridge/codroid_node/codroid_io.py` 来自同一官方压缩包，仅作为协议对照和序列化兼容测试基准，不需要启动它。
- `estun_segmented/bridge.py` 沿用官方 UDP 端口与 JSON 字段格式，并补充反馈校验、错误报告和命令行参数。

官方 ROS2 协议规定笛卡尔分量使用 x/y/z/rx/ry/rz，单位为 m/rad；姿态定义参见 README 链接的酷卓 S 系列官方用户手册。原始包保留自身的作者和许可证元数据；本项目未将固件、培训材料或比赛文档纳入仓库。
