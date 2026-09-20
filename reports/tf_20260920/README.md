# TF / localization timing verification — 2026-09-20

This change improves callback scheduling, localization state consistency and
freshness reporting. It does not implement IMU propagation or promise that
requests for a transform at `now()` will no longer wait for the next scan.

## Build and tests

- `open3d_loc`: built and installed with colcon.
- `go2w_nav2_navigation`: built and installed with CMake using its colcon build
  directory. The incremental colcon invocation stalled during environment
  preparation and was interrupted; the subsequent direct build succeeded.
- `test_odom_kinematics`: five GTests passed (translation frame, yaw wrap,
  quaternion sign, mounted-IMU lever arm, pose covariance lever arm).
- `check_tf_pipeline.py`: real localization and odometry-bridge executables,
  synthetic corner point cloud, ROS domain **189**, localhost only. No Unitree
  node, velocity command or navigation action was started.
- Verified initialization and both TF edges, original measurement timestamps,
  repeated-old-cloud expiry despite fresh odometry, recovery with fresh clouds,
  expiry on odometry outage, initial-pose reset and reinitialization, rejection
  of invalid odometry, and 0.6 m/s
  base-frame finite-difference velocity. See `integration_result.json`.
- Integration test uses a 0.3 s registration period and 1.5 s timeout to test
  expiry quickly. Production defaults are 2.5 s and 6.0 s. Both use two OpenMP
  threads and one OpenBLAS thread for the registration process.

The bridge callback's isolated-test median was about **0.304 ms**, P95
**0.417 ms**, maximum **0.568 ms** over 103 samples. These are processing times,
not end-to-end lidar-to-navigation latency, and are not a before/after benchmark.

Reproduce the isolated test after sourcing ROS and the workspace:

```bash
export ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=''
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
python3 src/go2w_nav2_navigation/test/check_tf_pipeline.py \
  --output /tmp/tf_pipeline_result.json
```

## Deployment and limits

Restart navigation to load the new executables. Existing maps, footprint,
stop/slow polygons and inflation radii were not changed by this task.

The confidence heartbeat now expires based on actual accepted registration
data and successful completion, not arrival of new odometry. This may expose
previously hidden localization stalls as navigation stops. The existing
motion bridge requires confidence at least 0.7 and a fresh heartbeat.

Velocity is an estimate averaged between consecutive FAST-LIO poses, not
high-rate IMU propagation. Its configurable variances are initial estimates,
not experimentally calibrated covariances. The pose covariance reference-point
conversion assumes the ROS header-frame fixed-axis covariance convention.
Underlying FAST-LIO covariance quality is not established by these tests.

The input timestamp guards fail closed on significant clock rollback; restarting
the navigation pipeline after a clock reset is required. Sensor hardware clock
synchronization, full-load timing, slip/braking behaviour and real-corner safety
were not verified by the synthetic tests. The existing `base_shift_correction`
and transform timeout settings are retained.

After restart, `scripts/check_tf_timing.py --duration 15` is a read-only tool
to measure message ages and the new timing topics on the real robot.

## Original files

Original versions are under `config_backups/tf_20260920/src/`. That directory
contains `COLCON_IGNORE` so it cannot be discovered as duplicate ROS packages.

## User-supplied field timing follow-up

Filed on 2026-09-20; the pasted output has no absolute capture timestamp or
robot motion/load metadata. Full values are in `user_field_timing.json`.

Over 15.0089 s, odometry and primary TF edges each delivered 150 samples
(about 9.99 Hz); obstacle clouds delivered 137 (about 9.13 Hz).
The bridge processing median/P95/max were 0.486/1.502/6.055 ms.
Its input was already 57.36 ms old at the median. Base TF arrival age
median/P95/max was 66.42/85.24/175.47 ms. This identifies upstream age as
the larger contribution but does not distinguish sensor timing, compute,
queueing and transport. Percentiles of separate topics are not paired delays.

Registration age ranged 0.229–2.729 s with reported computation durations
140.50–167.74 ms, consistent with a 2.5 s registration period. Diagnostic
heartbeats repeat the latest result; 150 messages are not 150 registrations.
Confidence stayed 0.9473–0.9528, above the 0.7 guard threshold.

These are real-system user-supplied observations, separate from the isolated
benchmark above. They do not establish reduced end-to-end latency against a
matched baseline or absence of TF query failures. Arrival age is not the
maximum age of the latest transform between messages. No further navigation
parameters were changed for this follow-up.
