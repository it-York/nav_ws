# 墙角配置验证记录

现场证据：用户确认左前侧点云来自墙壁。只读采样 85 帧中 82 帧有点进入
StopZone，样例 base_link 坐标为 (0.292, 0.299, 0.573) m。
此前导航日志存在 StopZone 急停、Starting point in lethal space、20 Hz 控制
超时和 RotationShim 路径点 map→base_link 变换的 future extrapolation。

修改文件：`src/go2w_nav2_navigation/config/nav2_go2w.yaml`。
修改前备份：`config_backups/corner_20260918/nav2_go2w.yaml`。
详细参数和重启步骤见导航包 README。没有修改实时节点参数、发送运动目标或重启实机导航。

## 独立 ROS 测试

使用本机安装的 Nav2 Humble 1.1.20，在 ROS_DOMAIN_ID=186、
ROS_LOCALHOST_ONLY=1 环境运行 map_server、planner_server、controller_server。
仅激活地图与规划器；控制器仅执行 configure，未执行 activate。
测试域内禁用实时点云层，使用静态合成地图，不连接 Unitree 运动接口。

地图分辨率 0.05 m，水平走廊为 x∈(0.5,4.7)、y∈(1.0,2.6)，
竖直走廊为 x∈(3.1,4.7)、y∈(1.0,5.5)，两者取并集，走廊宽 1.6 m。
起点 (1.2,1.8,0°)，终点 (3.9,4.8,90°)。通过 ComputePathToPose 取得真实
SmacPlanner2D 输出，然后沿折线每不超过 0.01 m 采样，以路径切线为机身朝向，
用 Pillow 多边形栅格化检查含余量的 0.96×0.60 m 矩形是否覆盖障碍栅格。

| 指标 | 修改前 | 修改后 |
|---|---:|---:|
| 规划输出点数 | 91 | 102 |
| 加密检查样本数 | 548 | 608 |
| 轮廓覆盖障碍的样本数 | 140 | 0 |
| 路径中心所在栅格到最近障碍栅格中心的最小距离 | 0.283 m | 0.671 m |

数值为该合成案例的离散栅格检查结果，不是连续几何安全距离，也不是实机通过率。
140 表示采样位置数，不表示发生 140 次实际碰撞。没有模拟速度跟踪、轮胎打滑、
定位误差、雷达延迟、急停动作或起终点旋转。控制器仅验证插件与参数可成功加载；
10 Hz 实时性、TF 日志和实机转弯仍需重启后检查。

![配置前后的路径和机身轮廓](path_comparison.png)

机器狗若已停在红色急停区内，应先取消旧目标并由人工遥控移至开阔位置，
再用新配置启动、确认点云与地图对齐并发送目标。新配置用于提前留出余量，
不绕过已触发的墙壁急停。

## 实现依据

- [Humble MPPI CostCritic 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/critics/cost_critic.cpp)：
  score() 对中心代价小于 1 的轨迹点直接跳过，随后才做轮廓碰撞检查。
  因此膨胀范围需要覆盖机身外接半径；本机带余量后的半对角线约 0.566 m。
- [Humble RotationShim 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp)：
  getSampledPathPt() 使用当前时刻并向 base_link 转换，与现场错误中的调用路径一致。
  直接使用 MPPI 移除这个调用环节，不代表所有 TF 延迟均已解决。
- [Humble SmacPlanner2D 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/smac_planner_2d.cpp)：
  二维搜索使用半径式碰撞检查，并通过 cost_travel_multiplier 调整代价偏好。
  保留二维规划器的当前方案依赖局部轮廓检查，不承诺所有狭窄转弯都能通过。
