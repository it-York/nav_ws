#!/usr/bin/env python3
"""Isolated integration test: synthetic odometry/clouds; no robot command topics."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import qos_profile_sensor_data


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '189' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('This test requires ROS_DOMAIN_ID=189 and ROS_LOCALHOST_ONLY=1.')
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--prediction-tf', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    # test/ -> package -> src -> workspace
    work = Path(tempfile.mkdtemp(prefix='nav_ws_tf_test_'))
    os.environ['ROS_LOG_DIR'] = str(work / 'ros_logs')
    processes, files = [], []
    result = {'work_directory': str(work), 'checks': []}
    rclpy.init()
    node = rclpy.create_node('tf_pipeline_test')
    odoms, confidences, registration_ages, processing_times, tf_stamps = [], [], [], [], []
    subscriptions = [
        node.create_subscription(Odometry, '/odom', lambda m: odoms.append(m), qos_profile_sensor_data),
        node.create_subscription(Float32, '/localization_3d_confidence', lambda m: confidences.append(m.data), 10),
        node.create_subscription(Float32, '/localization_3d_registration_age_ms', lambda m: registration_ages.append(m.data), 10),
        node.create_subscription(Float32, '/fastlio_odom_bridge/processing_ms', lambda m: processing_times.append(m.data), 10),
        node.create_subscription(TFMessage, '/tf', lambda m: tf_stamps.extend(
            (t.header.frame_id, t.child_frame_id, t.header.stamp.sec, t.header.stamp.nanosec)
            for t in m.transforms), qos_profile_sensor_data),
    ]
    odom_pub = node.create_publisher(Odometry, '/Odometry', 10)
    cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered', 2)
    initial_pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    static = StaticTransformBroadcaster(node)
    transforms = []
    for parent, child in [('base_link', 'imu_link'), ('base_link', 'motion_link'), ('odom', 'camera_init')]:
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = parent, child
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    static.sendTransform(transforms)
    points = []
    for i in range(41):
        for j in range(31):
            a, b = -2.0+i*0.1, -1.5+j*0.1
            points.extend([(a, b, -0.4), (3.0, a, b+1.0), (a, 2.0, b+1.0)])
    pcd = work / 'fixture.pcd'
    pcd.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                   f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n'
                   + '\n'.join(' '.join(map(str,p)) for p in points)+'\n')
    cloud_template = PointCloud2()
    cloud_template.header.frame_id = 'camera_init'
    cloud_template.height, cloud_template.width = 1, len(points)
    cloud_template.fields = [PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                             for i,n in enumerate(('x','y','z'))]
    cloud_template.point_step, cloud_template.row_step = 12, len(points)*12
    cloud_template.is_dense = True
    cloud_template.data = b''.join(struct.pack('<fff',*p) for p in points)
    config = work/'params.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'path_map': str(pcd), 'initialpose':[0.0]*6, 'use_prediction_tf': args.prediction_tf,
        'loc_update_period':0.3, 'registration_timeout':1.5, 'max_input_age':0.5,
        'pcd_queue_maxsize':3, 'voxelsize_fine':0.15, 'voxelsize_coarse':0.15,
        'threshold_fitness':0.7, 'threshold_fitness_init':0.7,
        'maxpoints_source':6000,'maxpoints_target':10000,
        'base_T_imu_xyz':[0.0]*3, 'base_T_imu_rpy_deg':[0.0]*3,
    }}}))

    def spawn(package, executable, extra=()):
        log = open(work/(executable+'.log'),'w')
        files.append(log)
        process = subprocess.Popen([str(root/'install'/package/'lib'/package/executable),
            '--ros-args', '--params-file',str(config), *extra], stdout=log,stderr=subprocess.STDOUT,
            start_new_session=True)
        processes.append(process)
        return process

    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.005)
            for process in processes:
                if process.poll() is not None:
                    raise AssertionError(f'Node exited ({process.returncode}); logs: {work}')

    last_odom, last_cloud = None, None
    sent_stamps=set()
    def odometry(stamp=None,x=0.0):
        msg=Odometry()
        msg.header.frame_id, msg.child_frame_id='camera_init','body'
        msg.header.stamp=stamp or rclpy.time.Time(nanoseconds=node.get_clock().now().nanoseconds-80_000_000).to_msg()
        msg.pose.pose.orientation.w=1.0
        msg.pose.pose.position.x=x
        for i in range(6): msg.pose.covariance[i*6+i]=0.001
        return msg

    def pump(duration, odom=True, cloud=True, repeat_cloud=False):
        nonlocal last_odom,last_cloud
        end=time.monotonic()+duration
        next_odom=next_cloud=time.monotonic()
        while time.monotonic()<end:
            now=time.monotonic()
            if odom and now>=next_odom:
                last_odom=odometry()
                sent_stamps.add((last_odom.header.stamp.sec,last_odom.header.stamp.nanosec))
                odom_pub.publish(last_odom)
                next_odom=now+1/30
            if now>=next_cloud:
                if cloud:
                    last_cloud=copy.deepcopy(cloud_template)
                    last_cloud.header.stamp=odometry().header.stamp
                    cloud_pub.publish(last_cloud)
                elif repeat_cloud and last_cloud is not None:
                    cloud_pub.publish(last_cloud)
                next_cloud=now+0.1
            spin(0.008)

    def wait_valid(timeout=20.0):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            pump(0.2)
            if confidences and confidences[-1]>0.7:
                return
        raise AssertionError(f'No valid localization; see logs {work}')

    try:
        spawn('go2w_nav2_navigation','fastlio_odom_bridge')
        spawn('open3d_loc','global_localization_node',
              ['-r','/Odometry_loc:=/Odometry','-r','/cloud_registered_1:=/cloud_registered'])
        wait_valid()
        pump(0.2)
        assert odoms and any(x[0:2]==('map','odom') for x in tf_stamps)
        assert all((m.header.stamp.sec,m.header.stamp.nanosec) in sent_stamps for m in odoms)
        output_stamps={(m.header.stamp.sec,m.header.stamp.nanosec) for m in odoms}
        assert any(t[0:2]==('odom','base_link') and t[2:] in output_stamps for t in tf_stamps)
        result['checks'].append('Initialization and both TF edges; original measurement timestamps preserved')
        before=len(odoms)
        pump(2.2,cloud=False,repeat_cloud=True)
        assert confidences[-1]==0.0, 'Repeated old cloud incorrectly refreshed registration health'
        assert registration_ages[-1]>1500.0
        assert len(odoms)-before>35, 'Odometry callback blocked while clouds stopped'
        if args.prediction_tf:
            count = len([t for t in tf_stamps if t[:2] == ('map','odom')])
            pump(0.2,cloud=False,repeat_cloud=True)
            assert count == len([t for t in tf_stamps if t[:2] == ('map','odom')]), 'Expired registration kept publishing map TF'
        result['checks'].append('Repeated old cloud cannot renew health; odometry continues after registration expires')
        wait_valid()
        result['checks'].append('Fresh cloud restores localization health')
        pump(0.8,odom=False,cloud=True)
        assert confidences[-1]==0.0, 'Stale odometry must invalidate localization health'
        result['checks'].append('Odometry outage invalidates health independently of fresh clouds')
        wait_valid()
        reset = PoseWithCovarianceStamped()
        reset.header.frame_id = 'map'
        reset.header.stamp = node.get_clock().now().to_msg()
        reset.pose.pose.orientation.w = 1.0
        initial_pose_pub.publish(reset)
        pump(0.5, cloud=False)
        assert confidences[-1] == 0.0, 'Initial pose must invalidate previous registration health'
        wait_valid()
        result['checks'].append('Initial-pose reset invalidates health until fresh registration reinitializes')
        pump(0.3,odom=False,cloud=False)
        count=len(odoms)
        invalid=[]
        invalid.append(copy.deepcopy(last_odom))  # duplicate
        stale=odometry(rclpy.time.Time(nanoseconds=node.get_clock().now().nanoseconds-2_000_000_000).to_msg())
        invalid.append(stale)
        future=odometry(rclpy.time.Time(nanoseconds=node.get_clock().now().nanoseconds+2_000_000_000).to_msg())
        invalid.append(future)
        nan=odometry(); nan.pose.pose.position.x=float('nan'); invalid.append(nan)
        bad_frame=odometry(); bad_frame.header.frame_id='wrong'; invalid.append(bad_frame)
        for msg in invalid:
            odom_pub.publish(msg); spin(0.05)
        assert len(odoms)==count, 'Invalid odometry was published'
        result['checks'].append('Duplicate, stale, future, NaN and wrong-frame odometry rejected')
        start_ns=node.get_clock().now().nanoseconds-80_000_000
        for i in range(7):
            stamp=rclpy.time.Time(nanoseconds=start_ns+i*100_000_000).to_msg()
            odom_pub.publish(odometry(stamp,x=i*0.06))
            spin(0.1)
        assert abs(odoms[-1].twist.twist.linear.x-0.6)<1e-5
        assert odoms[-1].twist.covariance[0]>0.0
        result['checks'].append('Base-frame finite-difference velocity reports 0.6 m/s')
        ordered=sorted(processing_times)
        result['bridge_processing_ms']={'samples':len(ordered),'median':ordered[len(ordered)//2],
                                       'p95':ordered[int((len(ordered)-1)*.95)],'max':ordered[-1]}
        result['success']=True
    finally:
        for process in processes:
            if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL); process.wait()
        for log in files: log.close()
        node.destroy_node()
        rclpy.shutdown()
        Path(args.output).parent.mkdir(parents=True,exist_ok=True)
        Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
