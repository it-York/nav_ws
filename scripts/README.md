# 按高度生成二维地图

```bash
cd /home/koala2/nav_ws
python3 scripts/pcd_to_2d_map.py maps/magic2_loop.pcd \
  --poses maps/magic2_loop.pcd.keyframes_8151498739720/poses.csv \
  --output maps/magic2 \
  --min-height 0.15 --max-height 1.50 --resolution 0.05
```

输出 `magic2.pgm`、供 Nav2 加载的 `magic2.yaml`、预览 `magic2.png` 和参数记录 `magic2.report.json`。已有输出默认不覆盖；明确需要重新生成时加 `--overwrite`。

高度上下限是相对局部地面的高度，适用于本次同层平地数据。程序通过优化轨迹附近的地面回波估计雷达到地面的距离，再用局部地面观测建立高度基准。若地图地面已水平且 Z 已知，可用 `--ground-z -0.55` 替代 `--poses ...`，此时上下限相对该固定 Z。

原 PCD 与 XY 坐标保持不变。白色来自地面观测，黑色来自高度范围内的障碍物，灰色为未知。地面点半径 0.18 m、障碍点半径 0.10 m 用于补偿点云采样间隔；这不是机器狗轮廓膨胀，导航仍需配置 footprint 与 inflation layer。孤立障碍回波按 0.40 m 范围内至少三个点筛除。

局部高度投影不能修复 SLAM 的重影、水平错位或倾斜。本次点云有明显 Z 趋势，因此没有直接使用固定 Z 切片。依赖：Python 3、NumPy、SciPy、Pillow、PyYAML。

## Go2W 与雷达分网口连接

本机接线：`enx00e04c356b10`（USB 网口）接机器狗，`eno1` 接 MID360s。
目标地址分别是 `192.168.123.99/24` 和 `192.168.1.6/24`；雷达为 `192.168.1.195`。
导航环境默认将 CycloneDDS 绑定到 USB 网口，可用 `GO2W_INTERFACE` 显式覆盖。

先退出所有导航进程，避免连接恢复后继续执行旧目标，再修正 NetworkManager 配置：

```bash
sudo bash /home/koala2/nav_ws/scripts/configure_go2w_network.sh
```

脚本会保存旧设置到 `/tmp/nav_ws_network_backup.*`，修改两张有线网卡的地址和路由，
并测试常用机器狗地址 `192.168.123.161` 和雷达地址。检测到本工作空间的底盘控制
节点仍在运行时会拒绝修改。机器狗地址若被更改过，ping 结果需结合实际地址判断。
完成后先用 `./start_nav.sh` 检查定位，再按正常流程启用运动并重新发送目标。
