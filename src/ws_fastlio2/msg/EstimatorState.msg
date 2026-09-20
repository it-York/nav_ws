# Corrected scan-end IMU state. Pose error covariance uses world-fixed axes.
std_msgs/Header header
geometry_msgs/PoseWithCovariance pose
geometry_msgs/Vector3 velocity_world
geometry_msgs/Vector3 gyro_bias
geometry_msgs/Vector3 accel_bias
geometry_msgs/Vector3 gravity_world
# raw acceleration * accel_scale - accel_bias is specific force in m/s^2.
float64 accel_scale
# corrected IMU time = raw header time - imu_time_offset (seconds).
float64 imu_time_offset
float64[9] velocity_covariance
