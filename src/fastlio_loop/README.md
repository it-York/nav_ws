# FAST-LIO 回环建图

该包在 FAST-LIO 前端之外运行关键帧回环和 Ceres SE(3) 位姿图优化。回环通过后，重新变换历史关键帧生成地图；前端局部滤波器继续运行，不直接跳变其内部状态。

## 建图和保存

保持机器狗静止，等待终端出现 `IMU Initial Done` 后再移动。

```bash
cd /home/koala2/nav_ws
source install/setup.bash
ros2 launch fastlio_loop mapping_loop.launch.py
```

启动前关闭此前的 FAST-LIO、回环节点和定位节点，避免重复话题及 TF。如果 Livox 驱动已单独运行，增加 `start_driver:=false`。默认同时启动雷达、前端、后端和 RViz，不控制机器狗运动。无显示器时增加 `rviz:=false`。

缓慢完成一圈，并沿起点附近已走过的路继续走几米，让不同关键帧确认回环。终端出现 `LOOP ACCEPTED` 表示回环已被接受。查看诊断：

```bash
source /home/koala2/nav_ws/install/setup.bash
ros2 topic echo /slam/diagnostics --once
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

服务返回 `success: true` 后才退出建图。默认保存 `maps/magic2_loop.pcd`，及同名 `.keyframes_数字/` 文件夹（各关键帧局部点云、原始和优化位姿 CSV、约束 CSV）。这些数据保留了后续离线优化所需的关键帧信息，目前未提供自动恢复会话工具。Ctrl+C 不会自动保存。保存时建议停住机器狗。

默认拒绝覆盖已有 PCD；可启动时指定新绝对路径 `map_path:=/home/koala2/nav_ws/maps/magic3_loop.pcd`。确认需要覆盖时才使用 `overwrite_map:=true`。保存返回 `accepted_loops=0` 说明没有通过回环，不能将该地图视为已消除漂移。

建议新建图同时记录原始数据，便于校准、重放和调参（需要磁盘空间）：

```bash
ros2 bag record -o /home/koala2/nav_ws/recordings/magic2_raw /livox/lidar /livox/imu
```

重放时关闭实物驱动，启动 `start_driver:=false use_sim_time:=true`，另一个终端执行 `ros2 bag play <记录目录> --clock`。每次重放都重新启动建图节点，禁止在同一会话中倒退时间。

## RViz 与坐标系

- Fixed Frame 为 `map`，全局优化地图 `/slam/map`，历史轨迹 `/slam/path`，校正后实时位姿 `/slam/odometry`。
- `/cloud_registered` 只显示当前帧，不能累积旧点云作为优化地图。
- TF 为 `map -> camera_init -> body`。后端只发布第一段；原始 FAST-LIO 发布第二段。不要同时启动发布冲突 map TF 的定位程序。
- 输入精确同步 `/Odometry` 和 `/cloud_registered_body`，点云必须为 `body`（IMU）坐标系，不能直接接 Livox 原始点云。
- 后端旋转前端位姿协方差以匹配 map 坐标系；该协方差不代表整个位姿图的不确定度。

## 回环条件与限制

参数见 `config/backend.yaml`。默认每移动 0.75 m 或转动约 14 度保存一个关键帧；回环候选需相隔至少 25 个关键帧且超过 30 秒，在 XY 8 m 内搜索。采用粗细两级 ICP 加点到面精配准、双向重叠率、RMSE、几何可观测性检查，两个关键帧一致确认后才加入图优化，另有相邻约束及优化后残差检查。

这能校正成功回环带来的累计漂移，但不能保证任何环境都不漂。尚无场景描述子检索；偏差过大、初值很差、重复结构、长直走廊或只看到平面时可能无法闭环或误匹配。保守检查会拒绝几何退化候选。默认最多 2000 个关键帧，达到后必须保存并新建会话；后端队列溢出会计入 `dropped_pairs`，应检查设备负载。

新启动入口增加至少 2 秒的连续静止 IMU 初始化窗口，检查角速度和加速度波动。仍需在实机确认时间同步、LiDAR–IMU 内参和安装稳定性。`2.pdf` 的机身安装外参不能替代 LiDAR–IMU 内部外参。本包不强制把 z 压平，不会用二维投影掩盖三维建图漂移。

## 编译和验证

```bash
source /home/koala2/nav_ws/install/setup.bash
colcon build --symlink-install --base-paths src/ws_fastlio2 src/fastlio_loop --packages-select fast_lio fastlio_loop --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select fastlio_loop --event-handlers console_direct+
colcon test-result --verbose
```

核心测试覆盖位姿变换、闭环图优化、三维配准、平面退化拒绝、重复关键帧确认、保存保护与静止初始化。合成数据测试不能代替实机绕行验证。

ROS 接口集成测试（临时地图保存在 `/tmp` 并自动清理；不启动驱动）：

```bash
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/fastlio_loop_test_logs python3 src/fastlio_loop/test/ros_smoke.py
```

2026-09-17 本机验证：`fast_lio` 和 `fastlio_loop` 编译通过；10 项核心测试通过；独立 ROS 域的 43 关键帧合成绕行产生 1 次回环，地图、轨迹、里程计、TF、保存与拒绝覆盖检查通过。该测试未启动实物雷达，也未验证真实场地效果。
