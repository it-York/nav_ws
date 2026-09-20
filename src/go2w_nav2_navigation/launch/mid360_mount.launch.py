"""Static transforms for the MID360S mounted on Koala No. 2."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Sensor pose drawing, expressed in ROS REP-103 coordinates:
    # base_link -> lidar_link
    #   xyz = (290.79, 0, 156.33) mm
    #   rpy = (0, pi/6, 0) rad (30 degrees downward, per 2.pdf)
    base_to_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="koala2_base_to_mid360",
        arguments=[
            "--x", "0.29079",
            "--y", "0.0",
            "--z", "0.15633",
            "--roll", "0.0",
            "--pitch", "0.5235987755982988",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "lidar_link",
        ],
    )

    # The Livox driver currently stamps both raw cloud and IMU messages with
    # livox_frame.  Keep that public frame connected without changing the
    # driver or FAST-LIO's internal LiDAR/IMU calibration.
    lidar_to_driver_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mid360_to_livox_driver_frame",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "lidar_link",
            "--child-frame-id", "livox_frame",
        ],
    )

    return LaunchDescription([base_to_lidar, lidar_to_driver_frame])
