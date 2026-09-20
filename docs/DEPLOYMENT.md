# Git 提交与重新部署

本说明根据 2026-09-20 当前工作区整理。此次文档整理没有执行全新机器部署，
以下构建流程仍需在目标设备验证；不会把当前机器的 build/install 当成可迁移环境。

## 提交范围

建议作为一个完整工作空间快照提交：

- `src/` 六个 ROS 包及各自许可证、`third_party/Livox-SDK2/` SDK 源码。
- `maps/` 配套二维/三维地图及关键帧、`waypoints/` 点位和说明。
- 根目录启动/环境脚本、`scripts/`、`README.md`、`AGENTS.md`、`docs/`。
- `reports/` 优化报告与现有测试证据、安装位姿图 `2.pdf`。

不提交 `build/`、`install/`、`log/`、录包、Python 缓存、SDK 编译/安装目录、
本机 `config_backups/`、旧 `exports/` 副本及点位锁文件。
保留各来源的许可证；本仓库快照不把第三方源码统一改成新的许可证。
当前原始来源提交号没有完整记录，不应编造上游 revision。

`.gitignore` 已显式允许 `maps/` 下的 PCD 和根目录 `2.pdf`。
驱动目录也已取消对当前 ROS 2 `package.xml` 的忽略，克隆后 colcon 才能识别该包。
在有有效 Git 仓库之后，用 `git status --short` 检查提交范围，特别确认
`maps/magic2_loop.pcd` 和关键帧点云未被忽略。
目标仓库地址和可见性需明确后再配置远端、提交并推送；本说明不表示已经上传成功。

## 当前环境与外部依赖

平台：Jetson ARM64，Ubuntu 22.04，ROS 2 Humble。

本机已安装版本（仅为基线记录）：

| 依赖 | 版本 |
| --- | --- |
| `ros-humble-nav2-collision-monitor` | `1.1.20-1jammy.20260804.215756` |
| `ros-humble-nav2-mppi-controller` | `1.1.20-1jammy.20260804.230127` |
| `ros-humble-rmw-cyclonedds-cpp` | `1.3.4-1jammy.20260717.012644` |
| `libopen3d-dev` | `0.14.1+dfsg-7build3` |
| `libceres-dev` | `2.0.0+dfsg1-5` |

`safe_collision_monitor` 使用 Humble Collision Monitor 的受保护接口，升级 Nav2 后要重新编译和回归。
其他依赖包括编译工具、colcon、rosdep、Nav2、PCL、Eigen、OpenCV、yaml-cpp 和 ROS 消息/TF 包。
二维地图工具需要 NumPy、SciPy、Pillow、PyYAML。

**Unitree `unitree_api` 是仓库外部依赖。** 当前机器复用
`/home/koala2/koala_ws/install/unitree_api`，该目录及其上游源码不在此次工作空间中。
新机器需要先准备并构建兼容的消息包，再设置其安装前缀：

```bash
export UNITREE_API_PREFIX=/实际路径/install/unitree_api
```

不能仅靠克隆此工作空间完成全部依赖安装；首次部署需补齐该包，并记录其来源和 revision。
不要 source 整个旧 SLAM 工作区，以免同名 FAST-LIO/定位包遮蔽当前源码。

## 重新构建流程

从克隆后的工作空间根目录执行；下列步骤要求系统依赖和 Unitree 消息包已准备好。
先构建仓库内的 Livox SDK 到本地前缀：

```bash
source /opt/ros/humble/setup.bash
export NAV_WS="$PWD"
cmake -S third_party/Livox-SDK2 -B third_party/Livox-SDK2/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$NAV_WS/third_party/livox_sdk2_install"
cmake --build third_party/Livox-SDK2/build --parallel 2
cmake --install third_party/Livox-SDK2/build
```

然后加载消息包与 SDK 路径，构建六个 ROS 包：

```bash
source "$UNITREE_API_PREFIX/share/unitree_api/local_setup.bash"
export CMAKE_INCLUDE_PATH="$NAV_WS/third_party/livox_sdk2_install/include${CMAKE_INCLUDE_PATH:+:$CMAKE_INCLUDE_PATH}"
export CMAKE_LIBRARY_PATH="$NAV_WS/third_party/livox_sdk2_install/lib${CMAKE_LIBRARY_PATH:+:$CMAKE_LIBRARY_PATH}"
export LD_LIBRARY_PATH="$NAV_WS/third_party/livox_sdk2_install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
colcon build --symlink-install --base-paths src --parallel-workers 1 \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DBUILD_TESTING=ON
source install/setup.bash
```

`--base-paths src` 避免扫描备份、第三方和导出副本。不要运行驱动自带的 `build.sh`，
它会清理整个工作区的 build/install。新机器应从源码编译，不能复用旧机器的绝对路径符号链接。

已有构建目录时可运行相关测试：

```bash
colcon test --packages-select go2w_nav2_navigation fastlio_loop --event-handlers console_direct+
colcon test-result --verbose
```

集成测试的环境和历史结果见 [动态减速报告](../reports/dynamic_slowdown_20260920/README.md)
和 [TF 报告](../reports/tf_prediction_20260920/README.md)。

## 网络与本机参数

| 连接 | 当前网口 | Jetson 地址 | 对端地址 |
| --- | --- | --- | --- |
| MID360s | `eno1` | `192.168.1.6/24` | `192.168.1.195` |
| Go2W | `enx00e04c356b10`（USB） | `192.168.123.99/24` | `192.168.123.161` |

新设备按实际网卡设置 `GO2W_INTERFACE`。环境脚本将 CycloneDDS 绑定该网卡，
默认 `ROS_DOMAIN_ID=0`。检查驱动配置中的雷达和主机 IP，不能只改 DDS 网口。
需要修改网络时参考 [网络配置说明](../scripts/README.md)，网络脚本会修改 NetworkManager 设置。

`setup_go2w_navigation.sh` 自动添加本地 SDK 的动态库路径，不必每次手动设置 `LD_LIBRARY_PATH`。
`start_nav.sh` 通过 `NAV_WS` 传入当前路径和 magic2 地图；直接裸用底层 launch 仍有历史
`/home/nvidia/nav_ws` / magic1 默认值，应优先用快捷入口或显式传入地图参数。

## 拉取后验收顺序

1. 阅读 `AGENTS.md`，确认地图、点位和外部依赖完整。
2. 完成构建与相关单元测试；先用 `--dry-run` 检查点位。
3. 接传感器后用 `./start_nav.sh` 检查定位、TF、点云和规划，默认禁用运动桥。
4. 用时序检查工具采样；核对机身尺寸、安装外参与实际设备一致。
5. 获得真实运动任务后再启用 `--motion`，开展实机通道、转弯与制动验收。

新设备通过编译或时序测试，不等于已完成实际导航验收。
