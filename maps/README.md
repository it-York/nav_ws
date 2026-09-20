# magic2 地图资产

状态基线：2026-09-20。导航默认使用以下配套资产：

| 文件 | 用途 |
| --- | --- |
| `magic2.yaml` | Nav2 地图元数据；引用 `magic2.pgm` |
| `magic2.pgm` | 已由用户人工修饰的二维占据地图；当前规划依据 |
| `magic2_loop.pcd` | 回环优化后的三维点云；当前全局定位依据 |
| `magic2.png` | 生成时的预览图；不保证与后续人工编辑的 PGM 完全同步 |
| `magic2.report.json` | 二维生成参数和统计；不代表人工编辑记录 |
| `magic2_loop.pcd.keyframes_8151498739720/` | 461 个关键帧 PCD、`poses.csv`、`edges.csv`；保留优化和投影依据 |

`magic2.yaml` 当前分辨率为 0.05 m，原点为 `[-9.5, -43.0, 0.0]`。
该原点是栅格图像相对于 map 的位姿，不是机器狗当前位置。
命名点位保存在 [../waypoints/magic2_goals.json](../waypoints/magic2_goals.json)。

提交仓库时保留上述资产。只运行导航不需要加载关键帧目录，但它用于追溯和重新生成地图。
当前主 PCD 约 4.1 MiB，关键帧目录约 11 MiB，适合随此快照一并归档。

重新投影时优先指定新输出名称，不覆盖已经修饰的 `magic2.pgm`。
生成命令见 [scripts/README.md](../scripts/README.md)。二维投影不能修复 SLAM 漂移。
重建地图、改变分辨率或原点后，必须复核三维定位与二维地图对齐情况及已有点位。
