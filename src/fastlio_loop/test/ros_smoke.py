#!/usr/bin/env python3
"""Isolated ROS integration test; no driver or physical robot is started."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


def main():
    rclpy.init()
    node = rclpy.create_node('loop_integration_test')
    output = tempfile.TemporaryDirectory(prefix='fastlio_loop_ros_')
    path = Path(output.name) / 'test.pcd'
    command = [get_package_prefix('fastlio_loop') + '/lib/fastlio_loop/loop_backend',
               '--ros-args', '-p', f'map_path:={path}',
               '-p', 'keyframe_distance:=0.15', '-p', 'min_loop_age:=5.0',
               '-p', 'min_loop_separation:=10', '-p', 'submap_neighbors:=0',
               '-p', 'search_radius:=1.6', '-p', 'map_publish_period:=0.1']
    log = open(Path(output.name) / 'backend.log', 'w+')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    odom_pub = node.create_publisher(Odometry, '/Odometry', 10)
    cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered_body', 10)
    state = {'keyframes': 0, 'accepted_loops': 0}
    received = {}

    def diagnostic(msg):
        for status in msg.status:
            state.update({v.key: v.value for v in status.values})

    subscriptions = [node.create_subscription(DiagnosticArray, '/slam/diagnostics', diagnostic, 10)]
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for msg_type, topic, qos in [(Odometry, '/slam/odometry', 10),
                                 (RosPath, '/slam/path', latched),
                                 (PointCloud2, '/slam/map', latched),
                                 (TFMessage, '/tf', 10)]:
        subscriptions.append(node.create_subscription(
            msg_type, topic, lambda msg, t=topic: received.update({t: msg}), qos))

    def wait(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if process.poll() is not None:
                raise RuntimeError('backend exited')
            if predicate():
                return
        raise AssertionError(f'timeout; status={state}')

    service = node.create_client(Trigger, '/map_save')

    def save():
        future = service.call_async(Trigger.Request())
        wait(future.done, 30)
        return future.result()

    try:
        wait(lambda: service.service_is_ready() and odom_pub.get_subscription_count() > 0
             and cloud_pub.get_subscription_count() > 0)
        assert not save().success, 'empty map must be refused'
        points = []
        for x in np.arange(-5, 5, .18):
            for y in np.arange(-4, 4, .18):
                points.append((x, y, -1.2))
            for z in np.arange(-1.2, 3, .18):
                points.extend([(x, -4, z), (x, 4, z)])
        for y in np.arange(-4, 4, .18):
            for z in np.arange(-1.2, 3, .18):
                points.append((-5, y, z))
        world = np.array(points, dtype='<f4')
        for i in range(43):
            angle = 2*math.pi*i/40
            truth = np.array([2*(1-math.cos(angle)), 2*math.sin(angle), 0])
            raw = truth + [0.012*i, 0, -0.007*i]
            odom = Odometry()
            odom.header.stamp.sec = 100+i
            odom.header.frame_id = 'camera_init'
            odom.child_frame_id = 'body'
            odom.pose.pose.orientation.w = 1.0
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, raw)
            cloud = PointCloud2()
            cloud.header.stamp = odom.header.stamp
            cloud.header.frame_id = 'body'
            cloud.height, cloud.width = 1, len(world)
            cloud.fields = [PointField(name=name, offset=j*4, datatype=PointField.FLOAT32, count=1)
                            for j, name in enumerate(('x', 'y', 'z'))]
            cloud.point_step, cloud.row_step = 12, len(world)*12
            cloud.is_dense = True
            cloud.data = (world-truth).astype('<f4').tobytes()
            odom_pub.publish(odom)
            cloud_pub.publish(cloud)
            wait(lambda: int(state['keyframes']) >= i+1)
        wait(lambda: int(state['accepted_loops']) > 0 and '/slam/map' in received)
        assert received['/slam/path'].header.frame_id == 'map'
        assert len(received['/slam/path'].poses) == 43
        assert received['/slam/map'].header.frame_id == 'map'
        assert received['/slam/odometry'].header.frame_id == 'map'
        tf = received['/tf'].transforms[0]
        assert (tf.header.frame_id, tf.child_frame_id) == ('map', 'camera_init')
        assert abs(tf.transform.translation.z) > .1
        result = save()
        assert result.success, result.message
        assert path.stat().st_size > 1000
        bundles = list(path.parent.glob('test.pcd.keyframes_*'))
        assert len(bundles) == 1 and (bundles[0]/'poses.csv').exists()
        assert len(list(bundles[0].glob('*.pcd'))) == 43
        assert not save().success, 'existing map must be protected'
        print(f'PASS: ROS sync, {state["keyframes"]} keyframes, {state["accepted_loops"]} loops, '
              'map/path/odometry/TF, save bundle and overwrite refusal')
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.seek(0)
        print(log.read())
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        output.cleanup()


if __name__ == '__main__':
    main()
