# Pure Nav2 navigation for Unitree Go2-W

> 当前总入口为 [根 README](../../README.md)，架构基线见
> [ARCHITECTURE.md](../../docs/ARCHITECTURE.md)，点位操作见
> [waypoints/README.md](../../waypoints/README.md)。本文件保留多轮历史记录；
> 下文旧的固定橙框、magic1 路径、10 Hz 旧里程计桥和旧点位脚本不代表当前默认入口。

This package integrates the MID360S, FAST-LIO2, 3D point-cloud localization,
Nav2 SmacPlanner2D, Nav2 MPPI, Collision Monitor, Velocity Smoother, and the
Unitree Go2-W Sport API. It contains no SCAN-Planner components.

## 动态减速保护（2026-09-20 更新）

橙色保护区现在按 `/odom` 实际速度、目标速度、反应预算和制动能力生成。
直行、转弯和倒车分别检查矩形机身经过的空间，采用连续比例限速和受限加速度恢复。
红色急停区优先执行。旧章节中的固定宽 1.40 m / 55% 橙区为历史配置。
默认 RViz 的橙区已改为 MarkerArray，重启导航后加载。

参数、模型假设、隔离测试和实车标定步骤见
[动态减速说明](../../reports/dynamic_slowdown_20260920/README.md)。

## 当前青色机身轮廓（2026-09-20 更新）

按用户指定，全局和局部代价地图的最终轮廓均为长 **0.75 m**、宽 **0.45 m**，
以 `base_link` 为中心：x 为 ±0.375 m，y 为 ±0.225 m。
`footprint_padding=0.0`，避免显示轮廓额外增大。下文历史轮次中的
0.96 × 0.60 m 是之前的配置。配置通过安装目录符号链接加载，无需编译，重启导航生效。

## 2026-09-20 TF 更新

导航现在采用有界 IMU 传播输出 50 Hz 机身 TF，点云变换失败或预测失效会阻止运动。
修改后需重启整套导航。保持静止初始化，再执行：

```bash
python3 scripts/check_tf_timing.py --duration 60 --strict
```

时序通过不等于实车精度和制动距离通过。参数、隔离测试、恢复条件和实车验收见
[TF 第二轮说明](../../reports/tf_prediction_20260920/README.md)。

## 当前 koala2 Jetson：使用修改后的 magic2 地图

快捷启动（自动加载环境、二维地图、三维地图并打开 RViz）：

```bash
cd /home/koala2/nav_ws
./start_nav.sh            # 定位与规划检查，关闭底盘运动
# 退出检查模式并确认定位正常后：
./start_nav.sh --motion   # 启用实际导航运动
```

`./start_nav.sh --show-args` 仅检查启动参数，不启动节点。

已编译本工作空间的 `open3d_loc`、`go2w_nav2_bridge`。环境入口
`setup_go2w_navigation.sh` 使用本机 `koala_ws/install/unitree_api` 的消息包，
可通过 `UNITREE_API_PREFIX` 指定其他安装目录。地图启动参数必须成对指定：
修改后的 `magic2.yaml` 用于规划，原始 `magic2_loop.pcd` 用于三维定位。

先关闭回环建图、单独雷达驱动及其他导航控制程序，然后启动检查模式：

```bash
cd /home/koala2/nav_ws
source setup_go2w_navigation.sh
ros2 launch go2w_nav2_navigation go2w_navigation.launch.py \
  map:=/home/koala2/nav_ws/maps/magic2.yaml \
  pcd_map:=/home/koala2/nav_ws/maps/magic2_loop.pcd \
  enable_motion:=false start_rviz:=true
```

该入口自动启动雷达、FAST-LIO、三维定位、Nav2、RViz，约 20 秒后启动 Nav2。
保持静止完成初始化，建议首次从建图起点附近开始。必要时用 RViz 的
`2D Pose Estimate` 指定初始位置和朝向，并检查点云与地图是否重合。
新终端需同样 `source setup_go2w_navigation.sh`，再查看：

```bash
ros2 topic echo /localization_3d_confidence
ros2 topic echo /go2w/wheel_mode_ready
```

第二个话题只在启用运动后由模式管理器发布。检查模式下机器狗不会执行导航速度。
确认定位置信度持续高于 0.7、障碍物和规划路径合理后，退出上述进程，将
`enable_motion:=false` 改成 `enable_motion:=true` 重新启动。定位就绪且
`wheel_mode_ready` 为 true 后，用 `Nav2 Goal` 设置目标位置和朝向。

此地图的 Z 存在漂移；障碍点先在 `base_link` 中筛选高度 0.05～0.80 m，
再允许代价地图坐标系中的 -5～5 m 高度，避免在低 Z 地段丢弃实时障碍物。
这不能修复三维地图本身的几何误差；编译与地图加载检查也不能代替实机导航验证。

Data flow:

```text
MID360S -> livox_ros_driver2 -> FAST-LIO2
  -> /fastlio/estimator_state + /livox/imu -> fastlio_imu_odometry (50 Hz)
     -> /odom + odom -> base_link + /odom_prediction/valid
  -> /cloud_registered -> open3d_loc -> map -> odom
  -> /cloud_registered_body -> obstacle filter -> /cloud_obstacles
  -> /cloud_obstacles -> cloud_to_scan -> /scan_2d

Nav2 MapServer -> SmacPlanner2D -> MPPI(DiffDrive)
  -> /cmd_vel_raw -> VelocitySmoother -> safe_collision_monitor -> /cmd_vel
  -> Go2-W safety bridge -> /api/sport/request
```

Dynamic-obstacle behavior:

- the yellow Collision Monitor polygon extends 1.60 m forward and 0.70 m to
  either side; it keeps 55 percent of the command while the same MID360 points
  update both Nav2 costmaps;
- the navigation behavior tree checks path validity at 4 Hz and recomputes a
  path to the original goal when the current route is blocked;
- the red polygon is the final `stop` layer after velocity smoothing, so its
  stop command bypasses acceleration limiting. Its user-selected boundary is
  0.50 m forward and 0.30 m to either side of `base_link`; one valid obstacle
  point triggers the emergency stop;
- each MID360 costmap observation source explicitly accepts heights from
  -5.0 m to 5.0 m in the costmap frame after body-frame filtering, preventing a negative base height
  from silently discarding every obstacle point;
- Nav2 costmaps and Collision Monitor subscribe to the same transformed
  `/cloud_obstacles` PointCloud2, avoiding a second LaserScan message-filter
  path that can silently lose dynamic obstacles.

The default maps are:

- `/home/nvidia/nav_ws/maps/magic1.pcd` for 3D localization;
- `/home/nvidia/nav_ws/maps/magic1.yaml` for the Nav2 static map.

The mounting transform is taken from the workspace's `2.pdf` (lengths
converted from millimetres to metres, pitch 30 degrees downward):

```text
base_link -> lidar_link
xyz = [0.29079, 0.0, 0.15633] m
rpy = [0.0, 0.5235987755982988, 0.0] rad
```

The drawing locates the LiDAR optical centre, not the internal IMU. Keeping
the existing FAST-LIO extrinsic `T_imu_lidar` (identity rotation and translation
`[-0.011, -0.02329, 0.04412]` m), the body transform is derived as
`T_base_imu = T_base_lidar * inverse(T_imu_lidar)`:

```text
base_link -> imu_link
xyz = [0.278256279, 0.023290, 0.112620959] m
rpy = [0.0, 0.5235987755982988, 0.0] rad
```

The same transform is used for FAST-LIO odometry and body-cloud conversion.
The internal LiDAR/IMU extrinsic is not verified by this mounting drawing;
do not replace it with the robot's 30-degree mounting rotation.

Build:

```bash
cd /home/nvidia/nav_ws
source /opt/ros/humble/setup.bash
source /home/nvidia/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

First launch with hardware motion disabled:

```bash
ros2 launch go2w_nav2_navigation go2w_navigation.launch.py \
  start_rviz:=true enable_motion:=false
```

Before enabling motion, verify `/odom`, `/scan_2d`, `/cloud_obstacles`,
`/localization_3d_confidence`, the complete `map -> odom -> base_link` TF chain,
and both Nav2 costmaps. Localization confidence must remain above 0.7.

Only after a wheels-off-ground test, enable the Unitree command bridge:

```bash
ros2 launch go2w_nav2_navigation go2w_navigation.launch.py \
  start_rviz:=true enable_motion:=true
```

The bridge requires fresh odometry, localization, obstacle data, and verified
Go2-W `ai-w` mode. The indoor corner profile limits forward motion to
0.60 m/s and yaw to 0.70 rad/s; the bridge retains an independent
0.75 m/s hard limit. A stale safety input produces StopMove.

## 2026-09-20 TF 与定位时序优化

FAST-LIO 的测量时间戳贯穿 `/Odometry`、`/odom` 和 `odom -> base_link`，
不会用当前时间重标记旧位姿。里程计桥使用深度 1 的输入队列，拒绝重复、倒序、
超过 0.5 s、超前超过 0.05 s、非法位姿和错误 frame 的消息；先发布 TF 再发布 `/odom`。
桥将相邻位姿转换到机身中心后差分，输出当前 `base_link` 坐标下的平均速度，
包含雷达安装偏移的影响。首帧或间隔超过 0.5 s 时速度置零、协方差置为很大，
表示速度未知。速度方差默认线速度 0.04、角速度 0.09，可通过参数调整，尚未经实车标定。

定位点云回调与里程计回调分组执行，订阅只保留最新消息；历史点云拼接和 ICP
放到独立线程。共享位姿、置信度与滤波状态由同一互斥量保护，ICP 不持有该锁。
初始位姿重置会使正在计算的旧配准结果作废，并要求两个不同时间戳的成功配准重新初始化。
原 `loc_frequence` 仍兼容，其含义是秒；新配置明确使用 `loc_update_period: 2.5`。
定位进程默认限制 OpenMP 为 2 个线程、OpenBLAS 为 1 个线程。

置信度仍以 10 Hz 发布，兼容运动桥的 0.75 s 心跳保护，但现在会检查真实配准结果：
配准结果年龄或距成功完成的时间超过 `registration_timeout: 6.0`，或里程计年龄
超过 `max_input_age: 0.5`，置信度输出零。新里程计不能刷新旧配准的有效期。
失败配准也输出零。地图 TF 可以继续使用上次修正，但运动桥会根据无效置信度停止。
调试保存点云仍由 `save_scan` 控制，目录改为 `scan_save_directory`（默认 `/tmp`）。

新增诊断话题：

- `/fastlio_odom_bridge/input_age_ms`：输入测量年龄。
- `/fastlio_odom_bridge/processing_ms`：桥接回调处理耗时。
- `/localization_3d_registration_age_ms`：上次接受配准所用最新点云的年龄，-1 表示尚无有效结果。
- `/localization_3d_registration_duration_ms`：最近配准尝试耗时。

导航重启后可在另一终端执行只读采样：

```bash
cd /home/koala2/nav_ws
source setup_go2w_navigation.sh
python3 scripts/check_tf_timing.py --duration 15
```

这轮没有增加 IMU 高频预测，局部 TF 仍随 FAST-LIO 扫描更新；
查询“当前时刻”的未来外推问题仍需实车测量，不能以桥接耗时替代整条链路延迟。
验证与限制见 `reports/tf_20260920/README.md`，原文件备份位于 `config_backups/tf_20260920/`。

## 2026-09-20 膨胀半径调整（当前默认）

按用户要求，全局 inflation_radius 改为 0.40 m，局部改为 0.30 m；
cost_scaling_factor 仍为全局 3.0、局部 6.0。重启导航后生效。
局部半径小于加余量后机身的半对角线 0.566 m，需要重新验证墙角通行时的碰撞检查效果。

## 2026-09-18 窄通道代价调整（上一轮记录）

用户反馈局部膨胀区较大、可通行区域通行不正常。本次没有测得现场通道宽度，
诊断采样时没有正在执行的目标，不能断言当时停住完全由膨胀导致。
历史日志还存在优化器无可行轨迹和观测超时。

- 保留局部 inflation_radius=0.65 m，以覆盖矩形机身的碰撞检查；
  cost_scaling_factor 从 3.0 改成 6.0，使外围软代价更快衰减。
  MPPI CostCritic.cost_weight 从 5.0 改成 4.0，降低对可通行高代价区域的排斥。
  轮廓、padding、consider_footprint、collision_cost 和急停区保持原配置。
- 全局 0.70 m / 3.0 仍用于提前绕开墙角；直行/转向上限仍为 0.60 m/s / 0.70 rad/s。
- 两个 mid360_clear 的 expected_update_rate 改为 0.0：背景清障来源可以暂缺，
  不因缺少清除证据把整张代价地图标为过期。清障不会主动删除未观测障碍。
  mid360_scan 标记源仍按 0.25 秒检查新鲜度，底盘和 Collision Monitor 的超时保护保留。

当前含余量的轮廓宽 0.60 m、长 0.96 m。局部膨胀外圈有颜色不代表全是禁行；
实际障碍、轮廓相交和转弯扫过的空间仍决定能否通行。
80 cm 直通道虚拟跟踪中，修改前后均到达目标且带余量矩形未越墙，
说明不能仅凭外圈宽度诊断现场停住。本次不宣称解决了尚未复现的实机场景。
备份位于 `config_backups/narrow_passage_20260918/`，结果位于 `reports/narrow_passage_20260918/`。
取消目标并退出原导航后重启生效，无需编译。

## 2026-09-18 速度与动态障碍响应调整（上一轮记录）

上一轮墙角避障调整后，用户反馈直行、转向和动态障碍显示偏慢。
当前保留 0.70/0.65 m 膨胀范围、机身轮廓、MPPI 轮廓检查和原急停/减速区域，调整：

- 前进上限 0.60 m/s，转向上限 0.70 rad/s；MPPI、速度平滑器与底盘转向限幅一致。
  线加速度 0.40 m/s²，角加速度 1.00 rad/s²。黄色区域仍保留指令的 55%，
  对应上限约 0.33 m/s、0.385 rad/s，实际命令还受路径和障碍代价影响。
- 控制仍为 10 Hz / 3.2 s 预测时域，路径截取距离 2.4 m，覆盖 0.60×3.2=1.92 m。
- 全局代价地图更新/发布从 2/1 Hz 改为 5/5 Hz；局部更新仍为 10 Hz，
  发布从 3 Hz 改为 10 Hz。全局使用增量更新减少传输，RViz 已配置对应 Update Topic。
- 两个障碍层 observation_persistence=0，只保留最新观测，不反复标记 0.3 秒内的旧点。
  该参数不是栅格过期时间，未再次观测到自由空间的障碍不会自动消失。
- 新增 clearing-only 的 `/cloud_registered_body` 来源，使用真实雷达 `lidar_link`
  作为射线原点，让被高度过滤掉的地面等背景回波也能清除旧障碍；该来源不标记障碍。
  `/cloud_obstacles` 继续负责标记，代价地图先清除、再用当前观测重新标记障碍。
  不将无回波/空点云当作全方向自由空间，不使用虚构无限远射线。
- 修正障碍层自身的世界 Z 高度限制为 -5..5 m；之前只有来源缓冲区放宽高度，
  层级默认高度仍可能丢弃本地图负 Z 地段的有效障碍。车体坐标高度筛选保留 0.05..0.80 m。

独立 ROS 域测试验证了插件加载、负 Z 地段标记、观测到背景后的清除、
无回波保留障碍、当前障碍重新标记，以及空场直行/转向、黄色减速、红色急停输出。
测试不连接 Unitree 控制桥，不等同于实机动态避障和制动测试。
记录位于 `reports/responsiveness_20260918/`，原配置和 launch 备份在
`config_backups/responsiveness_20260918/`。取消旧目标、停止旧导航后重新
`./start_nav.sh --motion` 生效，无需编译。

现场 Collision Monitor 仍曾出现 TF 时刻查询超时，当前没有放宽超时或关闭位姿补偿。
本轮独立测试用静态 TF，不验证这一现场延迟；重启后仍需检查是否有新超时。
障碍清除速度取决于背景回波覆盖，雷达未看见的区域仍保留旧障碍。

## 2026-09-18 墙角避障调整（上一轮记录）

确认左前侧墙壁反复进入 StopZone 后，调整默认配置：

- 全局/局部 inflation_radius 从 0.28/0.25 m 改为 0.70/0.65 m，
  cost_scaling_factor 仍为 3.0。机身 footprint 与 0.05 m padding 保留，
  加余量后尺寸为 0.96 × 0.60 m，半对角线约 0.566 m。
  外围膨胀代价引导避墙，不表示整片膨胀区都禁止通行。
- SmacPlanner2D 的 cost_travel_multiplier 为 2.5，强化走低代价区域的倾向。
  它仍是二维规划器，不保证矩形机身沿整条路径的所有转向都可执行；
  局部 MPPI 保留 consider_footprint=true，实机仍需检查角落转向空间。
- 速度上限 0.35 m/s，线加速度 0.25 m/s²，转速上限仍为 0.40 rad/s。
  MPPI 为 10 Hz、model_dt=0.10、32 步、1000 条候选轨迹；
  降低原 20 Hz / 48 步 / 1600 条的计算量，并保留 3.2 秒预测时间。
- 直接使用 MPPI，移除 RotationShim 的每次换路对齐过程。
  Humble 的 shim 用 now() 查询路径点到 base_link 的变换，而现场日志反复
  显示 TF 落后该时间并等待超时。此次没有篡改传感器时间戳或向未来发布 TF；
  其他 TF 延迟仍需在重启后复查。
- StopZone、SlowZone、自车剔除和底盘安全检查保持原配置。

生效前先在 RViz 取消旧目标，停止导航；通过遥控将机器狗移到与墙有足够距离的
开阔处，避免仍处于急停区域。重新 `./start_nav.sh --motion` 后确认定位对齐，
再发送原目标验证。不要同时运行两套导航。配置为安装目录符号链接，无需重新编译。
原配置备份在工作空间 `config_backups/corner_20260918/nav2_go2w.yaml`。
配置调整不等于实机验证通过，也无法解决物理空间不足、地图错位或定位偏差。

## Collect and send navigation points

Open a new terminal and load the same ROS 2 environment before using either
helper:

```bash
cd /home/nvidia/nav_ws
source setup_go2w_navigation.sh
```

Before collecting a point, make sure `/localization_3d_confidence` is stable
above `0.7`. Save the current `map -> base_link` pose with a descriptive name:

```bash
python3 /home/nvidia/nav_ws/scripts/save_current_pose.py --name "正式PPT"
```

The script writes a timestamped JSON file under
`/home/nvidia/nav_ws/collected_point`, for example:

```text
/home/nvidia/nav_ws/collected_point/20260916_153455_982532.json
```

The JSON contains the point name, XYZ position, and complete quaternion. To
validate a collected point without connecting to Nav2 or moving the robot:

```bash
POINT_FILE="/home/nvidia/nav_ws/collected_point/20260916_153455_982532.json"
python3 /home/nvidia/nav_ws/scripts/navigate_collected_point.py \
  --dry-run "$POINT_FILE"
```

Confirm that the Nav2 action is available before sending the point:

```bash
ros2 action list -t | grep '/navigate_to_pose'
```

Then send the target and wait for Nav2 to report its final result:

```bash
python3 /home/nvidia/nav_ws/scripts/navigate_collected_point.py "$POINT_FILE"
```

The sender preserves the saved X/Y and yaw, while setting Z, roll, and pitch to
zero for 2D Nav2. Its default navigation timeout is 120 seconds. Override it
when needed with `--navigation-timeout SECONDS`. A timeout or Ctrl-C requests
that Nav2 cancel the accepted goal before the script exits.

With `enable_motion:=false`, this workflow can be used to inspect the goal and
planned path in RViz, but the Go2-W command bridge is absent and the robot will
not move. Only restart with `enable_motion:=true` and resend the target after
the map, localization, costmaps, obstacles, and planned path have been checked.
This second command can cause real robot motion.
