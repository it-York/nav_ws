# 速度与动态障碍响应验证

现场只读采样：障碍点云约 9.3 Hz，中位数据年龄 0.086 s；
原始机身点云约 8.8 Hz，中位 0.084 s；odom/base_link TF 约 10 Hz、年龄约 0.08 s；
RViz 局部地图约 2.5 Hz，全局地图约 1 Hz。采样期间未收到运动速度消息，
所以速度偏低依据配置上限及 55% 减速比例判断，没有声称测得底盘最高速度。

本轮参数与重启步骤见导航包 README。修改前配置在
`config_backups/responsiveness_20260918/`。最终参数提高直行上限至 0.60 m/s，
转向至 0.70 rad/s；保留上一轮膨胀场与原红/黄区域。

## 测试范围

ROS_DOMAIN_ID=187、ROS_LOCALHOST_ONLY=1；实际 Nav2 map_server/planner_server/
controller_server/collision_monitor 插件，合成地图和虚拟点云。
规划器及 Collision Monitor 在测试域激活；controller_server 仅 configure，
未激活控制器，不启动 Unitree 桥，也不向实机发送命令。

静态 TF 将机身放在世界 Z=-2 m，以检查上下游高度筛选。虚拟障碍在机身前方
1.525 m、侧向 0.025 m，高 0.4/0.5 m。背景为前方 3.05 m、侧向 -0.2..0.2 m
的一组有限地面回波，高 -0.4 m，不假定无回波方向可通行。

1. 当前障碍能标为致命代价 254。
2. 空点云持续 1 秒后，该障碍仍保留。
3. 仅背景回波出现后，旧配置在观察的 2 秒内没有清除，新配置清除了目标格。
4. 同时出现当前障碍与背景回波，障碍格仍标为 254。
5. Collision Monitor 空场输入 0.60 m/s 与 0.70 rad/s 可通过；
   前方黄色区域输入 0.60 m/s 时输出 0.33 m/s；红色区域单点输出零速度。

为单独比较清障效果，旧配置测试也暂时放宽了层级 Z 门限，否则旧配置在 Z=-2 m
处无法标记测试障碍。这个覆盖仅用于测试，未改备份或实机参数。

结果见 `result.json`。清除时间从测试阶段切换起算，并受地图更新周期相位影响；
不代表实机端到端检测延迟或保证值。测试没有验证底盘跟踪、机械转向能力、制动距离，
也没有模拟现场延迟 TF。当前 Collision Monitor 的 TF 超时仍需现场复查。

测试执行中发现单根射线在栅格化后可能擦过目标格，所以背景测试用一组实际有限
回波覆盖目标格；这也是现场不能承诺“人走开后固定时间内全部清除”的原因。

实现依据：[Humble ObstacleLayer](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp)
先 raytrace 清障再标记当前障碍，且层级高度门限独立于来源高度门限；
[ObservationBuffer](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/observation_buffer.cpp)
的 observation_persistence 控制观测缓存，不是地图栅格存活时间。
