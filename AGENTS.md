# Codex 接手说明

适用于此仓库。状态基线：2026-09-20。与用户沟通默认使用中文。

## 首先阅读

1. `README.md`：当前入口与默认地图。
2. `docs/ARCHITECTURE.md`：节点、TF、数据链和代码位置。
3. `docs/DEPLOYMENT.md`：依赖、构建和迁移限制。
4. 按任务阅读 `maps/README.md`、`waypoints/README.md` 或相关优化报告。

源码、实际 launch 和 YAML 是当前行为依据；旧报告和包内历史章节是变更证据。
不要根据旧文档把当前参数回退。`src/ws_fastlio2` 的 ROS 包名是 `fast_lio`。
`exports/fastlio_loop` 是旧导出副本，不是主工作区源码。

## 工作约束

- 默认检查文件或运行不连接真机的测试。用户明确要求真实运动时才启动 `--motion`
  或向已启用运动的 Nav2 发送目标。无运动导航仍需要传感器，不是仿真。
- 不要为了排除报错关闭 TF/点云/里程计/定位置信度超时保护，或用当前时间重标记旧数据。
- 不要并行运行建图和导航，也不要引入重复的 TF 发布者。
- 不覆盖人工编辑的 `maps/magic2.pgm`、配套 YAML、PCD 或点位；更新前明确范围并保留可恢复副本。
- 不直接修改 `build/`、`install/` 中的生成文件；修改 `src/` 的源文件并重新构建。
- 不调用 `src/livox_ros_driver2/build.sh` 做常规增量构建，该脚本会删除工作区 build/install。
- 不提交 build/install/log、录包、缓存、Git 凭据或本机生成的库。
- 新设备上的网口、IP、Unitree 消息包路径必须检查，不能照搬旧机器路径。

## 常见修改入口

- 全局/局部代价、MPPI、速度平滑、动态保护参数：
  `src/go2w_nav2_navigation/config/nav2_go2w.yaml`。
- 启动组合、自车点云剔除、预测参数：
  `src/go2w_nav2_navigation/launch/go2w_navigation.launch.py`。
- 50 Hz 预测：`src/go2w_nav2_navigation/src/fastlio_imu_odometry.cpp`
  和同包 `include/go2w_nav2_navigation/imu_prediction.hpp`。
- 动态减速：同包 `src/safe_collision_monitor.cpp`
  和 `include/go2w_nav2_navigation/dynamic_slowdown.hpp`。
- 三维定位：`src/open3d_loc/src/global_localization.cpp`。
- 真机指令：`src/go2w_nav2_bridge/src/cmd_vel_to_sport.cpp`。
- 点位：`scripts/save_waypoint.py`、`scripts/goto_waypoint.py`、`waypoints/magic2_goals.json`。

## 验证

纯文档或点位说明修改：校对当前 JSON/YAML、文件链接和命令，无需启动机器狗。
`python3 scripts/goto_waypoint.py --list` 和 `--dry-run` 不连接 ROS。
算法修改运行相关 C++ 测试；环境加载和构建命令见部署文档。

```bash
colcon test --packages-select go2w_nav2_navigation fastlio_loop --event-handlers console_direct+
colcon test-result --verbose
```

ROS 集成测试必须使用独立 ROS 域和 localhost；不要继承生产域 0 或 USB 网卡绑定。
测试入口在 `src/go2w_nav2_navigation/test/`、`src/fastlio_loop/test/`，
具体场景、隔离环境和结果见 `reports/tf_prediction_20260920/README.md`
及 `reports/dynamic_slowdown_20260920/README.md`。不可把历史通过记录当作本次运行结果。

## 尚未完成的现场验证

- 动态减速中的线制动 0.50 m/s²、角制动 0.80 rad/s² 是初值，尚需实机标定。
- 自车点云剔除仍为 0.88 × 0.52 m，大于当前 0.75 × 0.45 m 轮廓，需要结合实物复核。
- 局部膨胀 0.30 m 小于机身半对角线约 0.437 m，不能仅依赖膨胀/中心代价保证转角无碰撞。
  自定义动态保护检查机身扫过空间，但没有替换 MPPI 为完整的独立矩形轨迹碰撞检查器。
- 回环优化不保证所有环境零漂移；地图修改、重建或原点改变后需要复核/重采点位。

修改结束后说明：改变了什么、如何验证、哪些内容还未实机验证；同步相关文档。
