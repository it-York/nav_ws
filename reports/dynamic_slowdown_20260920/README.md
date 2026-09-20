# 随实际运动变化的减速保护范围 — 2026-09-20

## 问题与本轮改动

原橙色框前伸 1.60 m、左右各 0.70 m；其中至少两个点会把平移和转向统一压到 55%。沿墙直行时，即使墙不在机身将经过的空间内，也容易持续触发减速。本轮在现有 `safe_collision_monitor` 内实现动态保护和连续比例限速，导航默认配置启用。

输入为最新 `/odom` 的机身速度、`/cmd_vel_smoothed` 的目标速度，以及已经完成时差运动补偿的 `/cloud_obstacles`。蓝青色规划轮廓仍使用用户指定的 0.75 × 0.45 m；红色 StopZone 仍前 0.50 m、后 0.48 m、左右各 0.30 m，并优先执行。原固定 SlowZone 从活动多边形列表移除，避免与动态策略同时限速。

动态模型检查三组矩形轨迹：

1. 按实测速度经过反应时间，然后制动至零；用于覆盖当前惯性。即使目标速度为零或反向，当前惯性也不能被忽略。
2. 经过反应时间后执行候选目标速度，保持短时预览，再制动。
3. 经过反应时间后，按加减速度限制逐渐过渡到候选目标速度，再制动；用于补充转向、倒车及速度切换过程。

每组轨迹检查完整矩形内部，包含前后四角；通过空间分桶减少障碍查询量。轨迹采样同时约束时间间隔和角点移动距离，另加采样保护量，降低小障碍从采样点之间漏过的风险。当前制动模型按线速度与角速度共同缩放到零，保持制动曲率并不超过配置的减速度。

对目标线速度与角速度使用同一个比例，保持候选目标的转弯曲率。先尝试恢复加速度允许的最大比例；若轨迹不安全，逐级寻找较低的安全比例并细化。每个最终输出都经过检查，转弯场景不假设所有比例下的可行性单调。若没有找到安全候选，则输出零速度；如果当前实测速度的制动轨迹已不安全，也立即输出零速度。

风险上升允许立即压低速度。恢复增加 0.15 s 的释放保持和加速度约束，避免障碍边缘附近在全速和固定 55% 之间反复跳变。新策略不修改规划路径、不提升速度上限，也不改变现有 TF/预测失效停车保护。

## 当前参数

配置入口：`src/go2w_nav2_navigation/config/nav2_go2w.yaml` 的 `collision_monitor.dynamic_slowdown`。

| 参数 | 当前值 | 含义 |
|---|---:|---|
| body_length / body_width | 0.75 / 0.45 m | 与当前规划机身尺寸一致；以后修改轮廓时须同步 |
| margin | 0.04 m | 动态轨迹中的几何余量，不修改青色轮廓 |
| step_distance | 0.01 m | 最大角点采样位移，并作为额外采样保护量 |
| reaction_time | 0.20 s | 控制/执行反应预算；计算时另加点云和里程计年龄中的较大值 |
| preview_time | 0.60 s | 执行目标或过渡速度的预览时间，之后还会模拟完整制动 |
| linear_braking | 0.50 m/s² | 正值制动能力幅值，暂按保守假设配置 |
| angular_braking | 0.80 rad/s² | 正值角减速度幅值，暂按保守假设配置 |
| linear_acceleration / angular_acceleration | 0.40 m/s² / 1.0 rad/s² | 过渡轨迹使用的加速度约束 |
| recovery_acceleration / recovery_angular_acceleration | 0.40 m/s² / 1.0 rad/s² | 输出恢复时的上限；危险制动不受此恢复限幅拖延 |
| release_hold | 0.15 s | 安全干预后的恢复保持时间 |
| step_time | 0.04 s | 轨迹采样最大时间步，实际步长还受角点位移约束 |
| odom_timeout | 0.15 s | 同时检查测量年龄和接收间隔，旧消息重复发送不能续期 |
| max_processing_time | 0.08 s | 本次处理超预算后拒绝运动输出；普通 Linux 不保证硬实时 |
| scale_samples / refinement_steps | 16 / 5 | 比例搜索和边界细化预算 |

静止时橙色轨迹的基础检查矩形包含 4 cm 几何余量和 1 cm 采样保护量，约为 0.85 × 0.55 m。运动时它沿直线、弧线或后方展开；不再要求橙色区域始终是包含红色框的大矩形。

制动能力、执行反应时间目前是**可配置的模型假设，不是已经测得的 Go2W 制动性能**。要在不同速度、载荷和地面上实测，设置不能高估实际制动能力。跟随目标的响应和制动轨迹是近似模型，并不保证覆盖未知的底盘打滑、侧滑或所有执行器响应。该层按当前点云判断空间占用，没有新增移动物体速度跟踪或行人轨迹预测。

模型参数设置为启动时只读，避免运行时参数服务显示修改成功而算法仍使用旧模型。修改 YAML 后统一重启导航生效。

## RViz 与诊断

默认 RViz 配置已把橙色 Polygon 改为 MarkerArray：

- `/collision_monitor/slow_zone`：橙色矩形序列，显示上述三组轨迹；显示做了抽样，碰撞判断使用全部采样。显示目标是原目标速度，便于看到为何受到限制，并非只画已经减速后的短轨迹。
- `/collision_monitor/speed_scale`：本次输出相对输入的比例，范围 0～1。
- `/collision_monitor/effective_reaction_s`：加入实际数据年龄后的反应预算。
- `/collision_monitor/reason`：原因，包括 `dynamic_slowdown`、`speed_recovery`、`measured_braking_path_blocked`、`obstacle_stop` 等。
- `/collision_monitor/processing_ms`：处理耗时。

`ready=true` 表示输入及处理有效，也可能因障碍而输出零速度；不表示允许非零运动。`dynamic_odometry_invalid_or_stale`、点云/TF/预测失效或 `collision_processing_deadline` 都输出零速度。点云处理后、发布速度前会再次检查时效及处理预算。

如果使用自己的 RViz 配置，需要把原 `/collision_monitor/slow_zone` 的 Polygon 显示项改成 MarkerArray；默认 `start_nav.sh` 使用的配置已经更新。

## 验证与使用

数学与几何测试覆盖平行侧墙、前方连续限速、倒车、左右转向和前后角扫掠、零目标速度时的实际惯性、制动距离、数据延迟、恢复加速度、立即制动、采样间小障碍、非法输入及矩形内部障碍。ROS 集成测试启动真实监控节点，速度输出强制映射到测试话题，且不启动 Unitree 控制节点。

本轮编译安装通过，新增 13 项动态减速单元测试与原有 13 项里程计测试均通过。实际 ROS 监控节点的 [19 项集成检查](integration_result.json) 通过：0.80 m 平行墙通道保持 0.60 m/s 指令；前方障碍场景自动限至约 0.307 m/s；左前角扫碰场景角速度约 0.445 rad/s，反向转开允许恢复至 -0.70 rad/s；倒车障碍场景约 -0.119 m/s。数值是各合成场景的结果，不是固定限速档位。

集成处理耗时中位数 10.85 ms、P95 20.97 ms、最大 51.28 ms（含主动注入 TF 失败）。20,000 点侧墙单元基准的几何计算平均 0.904 ms/次；不含 ROS、TF 等待和实机导航负载。模型参数只读、生命周期清理/重新激活以及所有输入失效停车也已检查。

与上一轮 IMU 预测和运动桥的 [19 项兼容性回归](tf_regression_result.json) 全部通过，包括 175 ms 延迟、旧数据、断流、时间回跳和定位跳变停车。

隔离测试命令：

```bash
cd /home/koala2/nav_ws
source setup_go2w_navigation.sh
export ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=''
python3 src/go2w_nav2_navigation/test/check_dynamic_slowdown.py \
  --output reports/dynamic_slowdown_20260920/integration_result.json
```

测试后关闭这个终端，避免把测试域/回环网络设置带入实机导航。真实使用时先退出旧导航，用新终端执行：

```bash
cd /home/koala2/nav_ws
./start_nav.sh
```

保持静止完成定位。在另一正常导航环境终端观察：

```bash
source /home/koala2/nav_ws/setup_go2w_navigation.sh
ros2 topic echo /collision_monitor/reason
# 或查看限速比例：
ros2 topic echo /collision_monitor/speed_scale
```

检查模式不启用底盘运动。之后在受控场地分速度验证沿墙、转弯、倒车、障碍撤离、断流和物理停止距离，并核查负载下处理耗时。红色框仍宽 0.60 m，窄于此宽度的通道不会仅因本轮动态橙区而获得放行。

本轮没有修改自车点云剔除范围、MPPI 矩形碰撞实现、全局路径或膨胀半径。这些属于已识别的独立问题；本轮结果不能当作完整局部规划及感知系统的验收。

本轮修改前备份：`config_backups/dynamic_slowdown_20260920_144223/`。回滚时整套恢复监控程序、配置及 RViz，重编译并统一重启。
