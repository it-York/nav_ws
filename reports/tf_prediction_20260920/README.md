# Go2W TF 工程化第二轮：2026-09-20

## 问题、原因、改法

第一轮实机采样 `/Odometry` 到达年龄中位数 60.23 ms，P95 77.71 ms，最大 171.53 ms；`odom -> base_link` 中位数 66.42 ms，最大 175.47 ms。FAST-LIO 每秒约 10 帧，严格查询当前时刻还可能等待下一帧。桥接自身中位数只有 0.49 ms，因此仅缩短桥接代码或增大 TF 超时不能解决主要缺口。原始记录在 [user_field_timing.json](../tf_20260920/user_field_timing.json)，没有完整负载、行驶状态记录，不能拿本次合成测试数值直接作实机前后对比。

本轮把“延迟测量”“当前状态估计”“估计失效后的停车”分开处理：

```text
FAST-LIO 扫描更新
  ├─ /Odometry、camera_init -> body：保留扫描时刻与原用途
  └─ /fastlio/estimator_state：校正后的位姿、速度、零偏、重力、单位比例、时间偏移
       + 原始 /livox/imu 历史
       └─ fastlio_imu_odometry：有界传播至当前时刻，50 Hz
            ├─ /odom、odom -> base_link
            └─ /odom_prediction/valid + reason

Open3D 定位：低频配准更新全局修正
  + 高频 /odom 的时间戳 -> map -> odom
  配准真实年龄独立判定，过期不再发布新的全局 TF，置信度置零

速度平滑器 -> safe_collision_monitor -> /cmd_vel -> 运动桥 -> Go2W
  点云失效 / TF 失败：零速度      预测失效 / 旧消息 / 命令超时：StopMove
```

这里的当前时间戳对应实际 IMU 积分结果。没有给旧位姿换上新时间戳，也没有关闭 `base_shift_correction`。原始 LiDAR 更新仍有传感器、计算和调度延迟；高频输出减少导航消费端的状态滞后，不能使物理延迟变成零。

## 已落地的改动

1. **FAST-LIO 导出完整状态。** 新消息 `fast_lio/msg/EstimatorState` 与扫描校正结果对应。加速度按 FAST-LIO 自身初始化的比例换算；传播使用同一套 gyro/accel bias、世界重力和固定时间偏移。新消息的姿态误差协方差转换为世界固定轴；没有改变原 `/Odometry` 接口或建图回环流程。
2. **独立进程高频传播。** 缓存原始 IMU，在迟到的扫描校正上重新积分至当前时刻。中点积分补偿转动，按既有 30° 安装姿态转换机身位姿，并计算安装偏置引起的线速度。只有导航入口替换原 `fastlio_odom_bridge`，两者不得同时发布同一条 TF。
3. **全局修正高频发布。** 导航设置 `use_prediction_tf=true`，`map -> odom` 跟随预测里程计时间戳；持有的是最近有效全局修正。实际配准仍每约 2.5 秒尝试一次，超时仍为 6 秒；不是将配准频率改为 50 Hz。单独 Open3D 启动默认保持旧发布方式，便于兼容。
4. **碰撞监控失败即停车。** 使用本机 Nav2 Humble 1.1.20 的碰撞监控基类和多边形判断，增加有明确有效性结果的点云来源。未收到、过期、异常时间、无效字段、NaN、TF 查询失败都不能解释为“没有障碍”。合法的空过滤点云仍可通行。只支持当前配置使用的 PointCloud2 和 stop/slowdown；不支持的来源或动作会配置失败。
5. **运动端增加独立保护。** 导航必须收到有效预测心跳；同时检查里程计/点云测量时间及接收时间，重复发送旧数据不能续期。拒绝 NaN 速度。保留原定位置信度、运动模式、到点锁定等保护。
6. **验收工具扩展。** `check_tf_timing.py` 现在同时记录到达年龄、周期查询时 TF 最新样本年龄、严格请求时刻的 TF 查询等待和失败数、预测有效性及故障原因。`--strict` 用明确门限返回验收结果，缺失话题不视为通过。

## 时序与故障门限

| 项目 | 当前值 | 行为 |
|---|---:|---|
| 预测输出 | 50 Hz | 同时发布 `/odom` 和 `odom -> base_link` |
| 最大预测时长 | 0.30 s | 超时停止有效输出；不能无限外推 |
| 最新 IMU 最大年龄 / 尾部保持 | 0.04 s | 短尾部保持最新输入，超时无效 |
| IMU 样本最大间隔 | 0.025 s | 覆盖路径中有大缺口则拒绝传播 |
| 校正跳变门限 | 0.15 m / 0.15 rad | 相对上一校正传播结果超限锁定；不是定位精度承诺 |
| 短 IMU 缺口后的校正 | 连续性检查 | 在缺口中另用上一位置、速度及宽松加速度/角速度边界检查 |
| 长校正中断 / 时间回跳 | 锁定停机 | 排除原因后重启整套导航，重新定位 |
| 碰撞监控 TF 等待预算 | 0.05 s | 查询失败输出零速度；保留真实时差运动补偿 |
| 碰撞点云超时 | 0.30 s | 不能用失效点云放行 |
| 预测心跳超时 | 0.15 s | 碰撞监控和运动桥独立检查 |
| 运动桥 `/odom` 超时 | 0.15 s | 测量与接收时间均检查 |
| 运动桥障碍点云超时 | 0.30 s | 测量与接收时间均检查 |
| 命令超时 | 0.30 s | 碰撞监控看门狗与运动桥分别处理 |

当前使用 `livox_frame` IMU，`time_sync_en=false`、固定时间偏移 0。自动时间同步分支不支持预测，会拒绝输出有效状态；不要为了绕过检查开启它。预测协方差使用保守增长项，不是完整的第二套 EKF 协方差传播。50 Hz 是目标频率，普通 Linux 调度不提供硬实时保证。

全局/局部膨胀仍为 0.40/0.30 m；停止框、减速框及规划速度没有扩大。独立矩形碰撞检查与最终轨迹校验仍需后续实现，本轮不是整个导航安全认证。

## 编译及隔离验证

已编译安装 `fast_lio`、`go2w_nav2_navigation`、`open3d_loc`、`go2w_nav2_bridge`，启动参数解析通过。数学/安装偏置测试共 13 个用例通过；colcon 汇总含测试包装记录为 15 项、0 失败。

- [预测、碰撞监控和运动桥集成结果](integration_result.json)：实际 C++ 节点、合成 IMU/延迟状态、实际安装外参；涵盖 175 ms 延迟、转弯、TF 缺失、过期/异常点云、IMU 断流恢复、重复旧输入、时间回跳、长校正中断及位姿跳变。
- [全局定位集成结果](localization_integration_result.json)：实际 Open3D 配准和合成地图，验证预测里程计驱动的 map TF、过期停止发布、初始位姿重置及重新初始化。该测试用原桥接节点产生输入里程计，测试的是全局 TF 路径；预测节点本身由上一项测试覆盖。
- 合成轨迹的数学误差很小只说明此轨迹的积分/坐标变换一致，**不代表实车定位达到同样精度**。测试 API 全部重映射到 `/test/...`，使用仅回环网络及 ROS 域 189/190；未发送实体机器人控制指令。

最终回归共 19 项预测/碰撞监控/运动桥检查及 7 项全局定位检查通过。最终合成测试 `/odom` 为 50.0 Hz，到达年龄中位数 1.85 ms、P95 3.11 ms、最大 4.04 ms。独立只读审计的严格当前时刻 `odom -> base_link` 查询 10 次成功，观测等待最大 21.85 ms，50 ms 预算内失败 0 次。碰撞监控含故障注入的处理耗时最大 53.72 ms；50 ms 查询预算加上轮询/调度开销，并不等于硬实时上限。

[隔离审计记录](timing_audit_isolated_without_map.json) 刻意没有启动全局定位且仅采集 3 秒，因此严格总验收返回 `false`：确认缺失 map TF、缺失置信度和采样过短都会判失败，不应将它当成现场验收失败或完整通过结果。

一次中间复测出现超过 25 ms 的合成 IMU 样本缺口，保护正确停车。测试输入后来改为独立的固定 200 Hz 采样时间轴，模拟硬件采样与 Python 回调调度的区别；故障注入时仍真实丢弃 IMU 样本，生产门限没有放宽。最终结果和测试 XML 已归档；相关日志见 `logs/`。

复测：

```bash
cd /home/koala2/nav_ws
source setup_go2w_navigation.sh
export ROS_DOMAIN_ID=190 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=''
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
python3 src/go2w_nav2_navigation/test/check_prediction_pipeline.py \
  --output reports/tf_prediction_20260920/integration_result.json
```

完成后关闭这个测试终端；不要沿用它的 ROS 域和回环网络设置启动实机导航。

## 实车启用与验收

在机器人静止时退出旧导航、独立 FAST-LIO 或建图进程，避免同时发布机身 TF。用新终端启动检查模式：

```bash
cd /home/koala2/nav_ws
./start_nav.sh
```

保持静止完成 FAST-LIO 初始化，等待全局定位有效并核对点云与地图重合。另一终端执行：

```bash
cd /home/koala2/nav_ws
source setup_go2w_navigation.sh
python3 scripts/check_tf_timing.py --duration 60 --strict \
  --output reports/tf_prediction_20260920/field_acceptance.json
```

`acceptance.passed=true` 才表示本次**时序门限**通过；这个工具不会启用运动。默认建议门限：`/odom` 及两条导航 TF ≥40 Hz，到达年龄 P95 ≤20 ms、最大 ≤50 ms；初始化后严格请求时刻 TF 查询在 50 ms 预算内无失败；预测始终有效；置信度 ≥0.7；障碍点云 ≥8 Hz 且年龄 ≤300 ms。它没有验证姿态准确性或制动距离。无导航指令时碰撞监控显示 `command_timeout` 是停车状态，时序门限不将其当作运动故障。

检查模式通过后退出该进程，再用 `./start_nav.sh --motion` 做受控实车测试。分别记录静止、直线、连续转弯、拐角、动态障碍和真实 CPU/GPU 满负载下的 60 秒采样，再做长时间运行。必须独立核查：

- IMU/点云源时间稳定、只有一个机身 TF 发布者；比较 `/odom_prediction/correction_position_m` 与 `correction_rotation_rad`，结合点云重合及外部参照验证预测误差。不要只看时间戳变新。
- 断流到零速度/StopMove 的软件响应，以及机器人实际停止距离；这些硬件结果不能从隔离测试推断。
- 锁定故障不会自行恢复运动；重启前确认原因为时钟、掉线还是估计跳变，重新核对定位后再给目标。
- 最高实际速度、地图边缘和窄通道的完整碰撞余量。当前矩形检查依赖膨胀的既有问题仍在，时序测试不能替代这项检查。

排查命令：

```bash
ros2 topic echo /odom_prediction/reason
ros2 topic echo /collision_monitor/reason
ros2 topic echo /localization_3d_confidence
```

常见原因：`input_receipt_timeout` 是输入接收断流；`imu_gap` 是 IMU 历史缺口；`anchor_expired_or_future` 是校正超龄或时序异常；`pointcloud_tf_unavailable` 是无法完成带时差的点云变换；含 `restart_required` 的原因锁定停机，需排故后重启。不要通过放大所有超时、改时间戳或关闭补偿来消除提示。

修改前源码备份在 `config_backups/tf_prediction_20260920/src/`，包含 `COLCON_IGNORE` 避免 colcon 扫描为重复包。恢复时应整套恢复这四个包、重编译、停止旧进程后统一重启，不能混用新启动文件与旧 FAST-LIO 二进制。

截至本报告：完成代码、编译和隔离验证；**尚未完成重启后的实机满负载、运动精度和物理停车验收**。
