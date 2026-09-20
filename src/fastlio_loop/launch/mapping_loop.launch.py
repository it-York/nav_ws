"""Standalone mapping: FAST-LIO odometry + validated loop closure backend."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("fastlio_loop")
    fast_lio = get_package_share_directory("fast_lio")
    driver = get_package_share_directory("livox_ros_driver2")
    # colcon's isolated install: <workspace>/install/fastlio_loop/share/fastlio_loop.
    workspace = Path(os.environ.get("NAV_WS", str(Path(share).parents[3])))
    for directory in (workspace / "maps", workspace / "src/ws_fastlio2/Log", workspace / "src/ws_fastlio2/PCD"):
        directory.mkdir(parents=True, exist_ok=True)
    sim = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("map_path", default_value=str(workspace / "maps/magic2_loop.pcd")),
        DeclareLaunchArgument("overwrite_map", default_value="false"),
        DeclareLaunchArgument("backend_config", default_value=os.path.join(share, "config/backend.yaml")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(driver, "launch_ROS2/msg_MID360s_launch.py")),
            condition=IfCondition(LaunchConfiguration("start_driver"))),
        Node(package="fast_lio", executable="fastlio_mapping", output="screen",
             parameters=[os.path.join(fast_lio, "config/mid360_navigation.yaml"), {
                 "use_sim_time": sim,
                 "filter_size_surf": 0.3, "filter_size_map": 0.3, "max_iteration": 4,
                 "publish.map_en": False, "pcd_save.pcd_save_en": False,
                 "publish.scan_publish_en": True, "publish.scan_bodyframe_pub_en": True,
                 "imu_init.min_duration": 2.0, "imu_init.max_gyro_norm": 0.10,
                 "imu_init.max_acc_std": 0.25,
             }], remappings=[("map_save", "/frontend/map_save_disabled")]),
        Node(package="fastlio_loop", executable="loop_backend", name="fastlio_loop_backend",
             output="screen", parameters=[LaunchConfiguration("backend_config"), {
                 "use_sim_time": sim, "map_path": LaunchConfiguration("map_path"),
                 "overwrite_map": LaunchConfiguration("overwrite_map"),
             }]),
        Node(package="rviz2", executable="rviz2", output="screen",
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", os.path.join(share, "rviz/mapping_loop.rviz")],
             parameters=[{"use_sim_time": sim}]),
    ])
