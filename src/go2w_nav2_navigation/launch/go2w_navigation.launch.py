"""MID360S + FAST-LIO2 + 3D localization + pure Nav2 MPPI for Go2W."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    workspace = os.environ.get("NAV_WS", "/home/nvidia/nav_ws")
    share = get_package_share_directory("go2w_nav2_navigation")
    livox_share = get_package_share_directory("livox_ros_driver2")
    fast_lio_share = get_package_share_directory("fast_lio")
    open3d_share = get_package_share_directory("open3d_loc")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    pcd_map = LaunchConfiguration("pcd_map")
    enable_motion = LaunchConfiguration("enable_motion")

    sensor_stack = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "mid360_mount.launch.py")
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(livox_share, "launch_ROS2", "msg_MID360s_launch.py")
            )
        ),
        TimerAction(
            period=LaunchConfiguration("fast_lio_delay"),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(fast_lio_share, "launch", "mapping.launch.py")
                    ),
                    launch_arguments={
                        "config_file": "mid360_navigation.yaml",
                        "rviz": "false",
                        "use_sim_time": use_sim_time,
                    }.items(),
                )
            ],
        ),
    ]

    localization = TimerAction(
        period=LaunchConfiguration("localization_delay"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(open3d_share, "launch", "open3d_loc_g1.launch.py")
                ),
                launch_arguments={"pcd_map": pcd_map, "use_sim_time": use_sim_time, "use_prediction_tf": "true"}.items(),
            )
        ],
    )

    odom = Node(
        package="go2w_nav2_navigation",
        executable="fastlio_imu_odometry",
        name="odom_prediction",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "publish_rate": 50.0,
            "max_prediction_horizon": 0.30,
            "max_imu_age": 0.04,
            "max_imu_gap": 0.025,
            "base_T_imu_xyz": [0.278256279, 0.023290, 0.112620959],
            "base_T_imu_rpy_deg": [0.0, 30.0, 0.0],

        }],
    )

    obstacle_cloud = Node(
        package="go2w_nav2_navigation",
        executable="cloud_obstacle_filter",
        name="cloud_obstacle_filter",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/cloud_registered_body",
            "output_topic": "/cloud_obstacles",
            "target_frame": "base_link",
            "base_T_cloud_xyz": [0.278256279, 0.023290, 0.112620959],
            "base_T_cloud_rpy_deg": [0.0, 30.0, 0.0],
            "min_height": 0.05,
            "max_height": 0.80,
            # Sensor-only blind radius. Self returns are removed by the
            # rectangular Go2W body envelope below.
            "min_range": 0.12,
            "max_range": 8.0,
            "voxel_leaf": 0.03,
            # Physical-body rejection only; keep safety zones visible.
            "self_min_x": -0.44,
            "self_max_x": 0.44,
            "self_min_y": -0.26,
            "self_max_y": 0.26,
        }],
    )

    # Optional scan visualization/consumers. Costmaps use PointCloud2 sources;
    # unobserved scan bins must not be used as evidence of clear space.
    clearing_scan = Node(
        package="go2w_nav2_navigation",
        executable="cloud_to_scan",
        name="cloud_to_local_scan",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            # Reuse the transformed, self-filtered cloud so the costmaps and
            # Collision Monitor make decisions from the same obstacle set.
            "input_topic": "/cloud_obstacles",
            "output_topic": "/scan_2d",
            "target_frame": "base_link",
            "base_T_cloud_xyz": [0.0, 0.0, 0.0],
            "base_T_cloud_rpy_deg": [0.0, 0.0, 0.0],
            "min_height": 0.05,
            "max_height": 0.80,
            "range_min": 0.12,
            "range_max": 8.0,
            "self_filter_enabled": False,
        }],
    )

    nav_nodes = [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", parameters=[params, {"yaml_filename": map_yaml, "use_sim_time": use_sim_time}]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             output="screen", parameters=[params, {"use_sim_time": use_sim_time}]),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             output="screen", parameters=[params, {"use_sim_time": use_sim_time}],
             remappings=[("cmd_vel", "/cmd_vel_raw"), ("odom", "/odom")]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             output="screen", parameters=[params, {
                 "use_sim_time": use_sim_time,
                 "default_nav_to_pose_bt_xml": os.path.join(
                     share, "behavior_trees", "yellow_zone_replanning.xml"),
             }]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             output="screen", parameters=[params, {"use_sim_time": use_sim_time}],
             remappings=[("cmd_vel", "/cmd_vel_raw")]),
        Node(package="go2w_nav2_navigation", executable="safe_collision_monitor", name="collision_monitor",
             output="screen", parameters=[params, {"use_sim_time": use_sim_time}]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             output="screen", parameters=[params, {"use_sim_time": use_sim_time}],
             remappings=[("cmd_vel", "/cmd_vel_raw"),
                         ("cmd_vel_smoothed", "/cmd_vel_smoothed")]),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_go2w_nav2", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "bond_timeout": 4.0,
                "node_names": ["map_server", "planner_server", "controller_server",
                               "bt_navigator", "behavior_server", "velocity_smoother",
                               "collision_monitor"],
            }],
        ),
    ]

    motion_nodes = [
        Node(
            package="go2w_nav2_bridge", executable="go2w_mode_manager", name="go2w_mode_manager",
            output="screen", condition=IfCondition(enable_motion),
            parameters=[{"target_mode": "ai-w", "auto_select": True,
                         "check_period": 1.0, "response_timeout": 2.0}],
        ),
        Node(
            package="go2w_nav2_bridge", executable="cmd_vel_to_sport", name="go2w_cmd_vel_control",
            output="screen", condition=IfCondition(enable_motion),
            parameters=[{
                "use_sim_time": use_sim_time,
                "publish_rate": 20.0, "cmd_timeout": 0.3,
                "max_vx": 0.75, "max_vy": 0.01, "max_vyaw": 0.70,
                "linear_deadband": 0.02, "yaw_deadband": 0.02,
                "odom_timeout": 0.15, "require_prediction": True, "prediction_timeout": 0.15, "localization_timeout": 0.75,
                "obstacle_cloud_timeout": 0.3,
                "min_localization_confidence": 0.7,
                "max_abs_position": 100.0, "max_abs_z": 5.0, "max_odom_jump": 2.0,
                "require_localization": True, "require_obstacle_cloud": True,
                "require_wheeled_mode": True, "wheeled_mode_timeout": 1.0,
                "lock_on_goal_reached": True,
            }],
            remappings=[
                ("cmd_vel", "/cmd_vel"), ("odom", "/odom"),
                ("localization_confidence", "/localization_3d_confidence"),
                ("obstacle_cloud", "/cloud_obstacles"),
                ("wheel_mode_ready", "/go2w/wheel_mode_ready"),
                ("navigate_to_pose_status", "/navigate_to_pose/_action/status"),
                ("api/sport/request", "/api/sport/request"),
            ],
        ),
    ]

    rviz = Node(
        package="rviz2", executable="rviz2", name="go2w_nav2_rviz", output="screen",
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        arguments=["-d", os.path.join(share, "rviz", "go2w_navigation.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", default_value=os.path.join(workspace, "maps", "magic1.yaml")),
        DeclareLaunchArgument("pcd_map", default_value=os.path.join(workspace, "maps", "magic1.pcd")),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "nav2_go2w.yaml")),
        DeclareLaunchArgument("fast_lio_delay", default_value="3.0"),
        DeclareLaunchArgument("localization_delay", default_value="10.0"),
        DeclareLaunchArgument("navigation_delay", default_value="20.0"),
        *sensor_stack,
        odom,
        obstacle_cloud,
        clearing_scan,
        localization,
        TimerAction(period=LaunchConfiguration("navigation_delay"), actions=nav_nodes + motion_nodes + [rviz]),
    ])
