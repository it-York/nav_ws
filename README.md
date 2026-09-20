# Go2W + MID360s 建图与导航工作空间

当前状态：2026-09-20。目标平台为 Jetson / Ubuntu 22.04 / ROS 2 Humble。
使用 FAST-LIO 与回环后端建图，使用三维地图定位和 Nav2 导航，支持 JSON 命名点位。

## 从哪里开始

| 文档 | 内容 |
| --- | --- |
| [AGENTS.md](AGENTS.md) | Codex 接手入口、修改约束、验证方式、待办 |
| [整体架构](docs/ARCHITECTURE.md) | 软件包、数据流、TF、参数和关键代码位置 |
| [部署与 Git 交接](docs/DEPLOYMENT.md) | 提交范围、依赖、重新编译、网络适配 |
| [地图说明](maps/README.md) | 二维/三维地图配套关系、关键帧和编辑说明 |
| [点位使用](waypoints/README.md) | 7 个命名点位、保存、无运动测试和导航 |
| [回环建图](src/fastlio_loop/README.md) | 建图、回环确认和地图保存 |
| [二维地图生成](scripts/README.md) | 按离地高度投影和网口配置 |
| [优化报告](reports/Go2W_建图定位导航优化报告_2026-09-20.md) | 历次问题、原因、修改和验证记录 |

当前源码与配置优先于历史报告。历史报告中的旧参数不能直接作为当前配置。

## 当前设备快速启动

以下命令用于已经配置依赖的当前 Jetson。新机器先阅读部署文档。

```bash
cd /home/koala2/nav_ws
./start_nav.sh                 # 雷达、定位、规划、RViz；不启动运动桥接
```

默认配套地图为 `maps/magic2.yaml`（引用已人工编辑的 `magic2.pgm`）和
`maps/magic2_loop.pcd`。初始化期间保持静止，确认点云与地图对齐。
建图与导航不要同时运行，也不要同时启动两套导航。

仅检查点位文件，不连接 ROS：

```bash
python3 scripts/goto_waypoint.py --list
python3 scripts/goto_waypoint.py "老板办公室" --dry-run
```

需要真实导航时，先退出旧导航，再启动运动模式并发送目标：

```bash
./start_nav.sh --motion
# 另一个终端：
cd /home/koala2/nav_ws
./goto_waypoint.sh "老板办公室"
```

`--motion` 启动 Unitree 运动桥接；发送目标后可能移动。默认模式不会停止另一套
已经运行的运动桥接，切换模式前必须退出旧进程。点位保存请使用 `/waypoint_pose`
上的 **2D Goal Pose**，实际导航使用 **Nav2 Goal**，详见点位文档。

## 当前工程状态

- 导航 TF：有界 IMU 传播输出 50 Hz `/odom` 和 `odom -> base_link`；三维定位提供 `map -> odom`。
- 规划/控制：SmacPlanner2D + MPPI DiffDrive，前进上限 0.60 m/s、转向上限 0.70 rad/s。
- 青色轮廓：0.75 × 0.45 m；全局膨胀 0.40 m，局部膨胀 0.30 m。
- 橙色动态保护：按实际/目标速度、制动模型检查机身扫过空间；红色急停区独立保留。
- TF、动态减速的离线和隔离 ROS 测试已有记录；实机制动参数和完整现场验收仍需完成。
- 当前不是跨机器即用镜像：Unitree 消息包、系统依赖和网络配置需在目标机准备。

只读时序检查（导航正常运行时）：

```bash
source setup_go2w_navigation.sh
python3 scripts/check_tf_timing.py --duration 60 --strict
```

时序通过不等于地图精度、实际制动距离或全部通道通行能力通过。
