#!/usr/bin/env python3
"""Loopback-only test of the actual monitor. No Unitree nodes or control interfaces."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String, Float32
from visualization_msgs.msg import MarkerArray
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from tf2_ros import TransformBroadcaster


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='191' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('Requires ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1')
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    root=Path(__file__).resolve().parents[3]
    work=Path(tempfile.mkdtemp(prefix='nav_ws_dynamic_test_'))
    os.environ['ROS_LOG_DIR']=str(work/'ros_logs')
    result={'work_directory':str(work),'checks':[]}
    rclpy.init();node=rclpy.create_node('dynamic_slowdown_test');process=None
    commands=[];reasons=[];scales=[];durations=[];markers=[]
    subscriptions=[node.create_subscription(Twist,'/test/cmd_out',lambda m:commands.append((time.monotonic(),m)),10),
      node.create_subscription(String,'/collision_monitor/reason',lambda m:reasons.append(m.data),10),
      node.create_subscription(Float32,'/collision_monitor/speed_scale',lambda m:scales.append(m.data),10),
      node.create_subscription(Float32,'/collision_monitor/processing_ms',lambda m:durations.append(m.data),10),
      node.create_subscription(MarkerArray,'/collision_monitor/slow_zone',lambda m:markers.append(m),10)]
    cmd_pub=node.create_publisher(Twist,'/test/cmd_in',1)
    cloud_pub=node.create_publisher(PointCloud2,'/test/cloud',5)
    odom_pub=node.create_publisher(Odometry,'/test/odom',qos_profile_sensor_data)
    health_pub=node.create_publisher(Bool,'/odom_prediction/valid',1)
    tf=TransformBroadcaster(node)
    measured=(0.,0.);desired=(.6,0.);points=[]
    feed_odom=feed_cloud=feed_health=feed_command=True
    bad_tf=stale_odom=invalid_odom=False
    x=y=yaw=0.;last_pose_time=time.monotonic();latest_stamp=None
    next_tf=next_cloud=next_command=0.
    def pump(duration):
        nonlocal x,y,yaw,last_pose_time,latest_stamp,next_tf,next_cloud,next_command
        end=time.monotonic()+duration
        while time.monotonic()<end:
            now=time.monotonic();stamp=node.get_clock().now().to_msg()
            if now>=next_tf:
                dt=now-last_pose_time;last_pose_time=now
                v,w=measured
                x+=v*math.cos(yaw+w*dt*.5)*dt;y+=v*math.sin(yaw+w*dt*.5)*dt;yaw+=w*dt
                t=TransformStamped();t.header.frame_id='odom';t.child_frame_id='base_link';t.header.stamp=stamp
                t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2)
                tf.sendTransform(t);latest_stamp=stamp
                if feed_odom:
                    m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_link';m.header.stamp=stamp
                    if stale_odom:m.header.stamp=rclpy.time.Time(nanoseconds=node.get_clock().now().nanoseconds-1_000_000_000).to_msg()
                    m.pose.pose.position.x=x;m.pose.pose.position.y=y;m.pose.pose.orientation=t.transform.rotation
                    m.twist.twist.linear.x=float('nan') if invalid_odom else float(v);m.twist.twist.angular.z=float(w);odom_pub.publish(m)
                if feed_health:
                    m=Bool();m.data=True;health_pub.publish(m)
                next_tf=now+.01
            if feed_cloud and now>=next_cloud and latest_stamp is not None:
                m=PointCloud2();m.header.frame_id='missing_sensor_frame' if bad_tf else 'base_link';m.header.stamp=latest_stamp
                m.height=1;m.width=len(points);m.point_step=12;m.row_step=12*len(points)
                m.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
                m.data=b''.join(struct.pack('<fff',p[0],p[1],.3) for p in points);cloud_pub.publish(m);next_cloud=now+.1
            if feed_command and now>=next_command:
                m=Twist();m.linear.x=float(desired[0]);m.angular.z=float(desired[1]);cmd_pub.publish(m);next_command=now+.05
            rclpy.spin_once(node,timeout_sec=.001)
            if process is not None and process.poll() is not None:raise AssertionError(f'monitor exited; see {work}')
    def transition(number):
        client=node.create_client(ChangeState,'/collision_monitor/change_state')
        assert client.wait_for_service(timeout_sec=5)
        request=ChangeState.Request();request.transition.id=number;future=client.call_async(request)
        end=time.monotonic()+5
        while not future.done() and time.monotonic()<end:pump(.02)
        assert future.done() and future.result().success, f'lifecycle {number}: {work}'
        node.destroy_client(client)
    def output():return commands[-1][1]
    def stopped():return commands and abs(output().linear.x)<1e-6 and abs(output().angular.z)<1e-6
    def latest_reason(expected):assert expected in reasons[-12:],(expected,reasons[-12:])
    config=yaml.safe_load((root/'src/go2w_nav2_navigation/config/nav2_go2w.yaml').read_text())
    params=config['collision_monitor']['ros__parameters']
    params['cmd_vel_in_topic']='/test/cmd_in';params['cmd_vel_out_topic']='/test/cmd_out'
    params['mid360']['topic']='/test/cloud';params['dynamic_slowdown']['odom_topic']='/test/odom'
    config_path=work/'monitor.yaml';config_path.write_text(yaml.safe_dump({'collision_monitor':{'ros__parameters':params}}))
    log=open(work/'monitor.log','w')
    try:
        process=subprocess.Popen([str(root/'install/go2w_nav2_navigation/lib/go2w_nav2_navigation/safe_collision_monitor'),
          '--ros-args','--params-file',str(config_path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        pump(.8);transition(1);transition(3);pump(2.)
        assert output().linear.x>.58,reasons[-10:]
        result['checks'].append('Fresh odometry, prediction and cloud permit forward command after smooth startup')
        points=[(i*.05,y) for i in range(-20,61) for y in (-.4,.4)];measured=(.6,0)
        pump(1.)
        assert output().linear.x>.58,reasons[-10:]
        result['checks'].append('0.8 m parallel-wall corridor does not trigger old fixed-width slowdown')
        points=[(.8,0)];measured=(.25,0);pump(1.5)
        slow_speed=output().linear.x
        assert 0<slow_speed<.55,(slow_speed,reasons[-10:])
        result['forward_limited_speed_m_s']=slow_speed
        result['checks'].append('Forward obstacle produces geometry/braking-based continuous speed reduction')
        points=[];start=len(commands);pump(2.)
        recovered=[m.linear.x for _,m in commands[start:]]
        assert recovered[-1]>.58
        assert max(b-a for a,b in zip(recovered,recovered[1:]))<=.045
        result['max_recovery_increment_m_s']=max(b-a for a,b in zip(recovered,recovered[1:]))
        result['checks'].append('Obstacle removal restores speed with bounded increments')
        measured=(0,0);desired=(0,.7);points=[(.2,.38)];pump(2.)
        assert 0<output().angular.z<.69,(output().angular.z,reasons[-10:])
        result['left_turn_limited_rad_s']=output().angular.z
        assert markers and any(abs(p.y)>.3 for marker in markers[-1].markers for p in marker.points)
        result['checks'].append('Left turn sweeps front corner; orange markers rotate and expand with motion')
        desired=(0,-.7);pump(2.)
        assert output().angular.z<-.68,(output().angular.z,reasons[-10:])
        result['checks'].append('Turning away from the same side obstacle is allowed after recovery')
        desired=(0,.7);points=[(-.2,-.38)];pump(2.)
        assert 0<output().angular.z<.69,(output().angular.z,reasons[-10:])
        result['checks'].append('Rotation also protects the rear corner sweep')
        desired=(-.18,0);measured=(-.1,0);points=[(1,0)];pump(1.5)
        assert output().linear.x<-.17,(output().linear.x,reasons[-10:])
        points=[(-.55,0)];pump(1.)
        assert -.17<output().linear.x<0,(output().linear.x,reasons[-10:])
        result['reverse_limited_speed_m_s']=output().linear.x
        assert any(p.x<-.5 for marker in markers[-1].markers for p in marker.points)
        result['checks'].append('Reverse protects rear obstacle and does not react to distant front obstacle')
        measured=(.6,0);desired=(0,0);points=[(.8,0)];pump(.4)
        assert stopped();latest_reason('measured_braking_path_blocked')
        result['checks'].append('Zero target speed cannot hide measured forward inertia')
        measured=(0,0);desired=(.6,0);points=[(.45,0)];pump(.4)
        assert stopped();latest_reason('obstacle_stop')
        result['checks'].append('Unchanged red StopZone overrides dynamic slowdown immediately')
        points=[];pump(2.)
        for fault in ('missing_odom','stale_odom','invalid_odom','missing_prediction','missing_cloud','bad_tf','missing_command'):
            feed_odom=fault!='missing_odom';stale_odom=fault=='stale_odom';invalid_odom=fault=='invalid_odom'
            feed_health=fault!='missing_prediction';feed_cloud=fault!='missing_cloud';bad_tf=fault=='bad_tf';feed_command=fault!='missing_command'
            pump(.5);assert stopped(),(fault,reasons[-10:])
            expected={'missing_odom':'dynamic_odometry_invalid_or_stale','stale_odom':'dynamic_odometry_invalid_or_stale',
              'invalid_odom':'dynamic_odometry_invalid_or_stale','missing_prediction':'prediction_invalid_or_stale',
              'missing_cloud':'stale_or_future_cloud','bad_tf':'pointcloud_tf_unavailable','missing_command':'command_timeout'}[fault]
            latest_reason(expected)
            feed_odom=feed_cloud=feed_health=feed_command=True;stale_odom=invalid_odom=bad_tf=False
            pump(2.);assert output().linear.x>.58,(fault,reasons[-10:])
            result['checks'].append(fault+' stops; valid data recovers smoothly')
        client=node.create_client(SetParameters,'/collision_monitor/set_parameters')
        assert client.wait_for_service(timeout_sec=3)
        request=SetParameters.Request()
        request.parameters=[Parameter(name='dynamic_slowdown.linear_braking',value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=.1))]
        future=client.call_async(request)
        deadline=time.monotonic()+3
        while not future.done() and time.monotonic()<deadline:pump(.02)
        assert future.done() and not future.result().results[0].successful
        node.destroy_client(client)
        result['checks'].append('Startup-only model rejects misleading runtime parameter updates')
        # Lifecycle reset must not retain a previously active output speed.
        transition(4);pump(.1);transition(2);pump(.1);transition(1);transition(3)
        start=len(commands);pump(.25)
        assert all(abs(m.linear.x)<.15 for _,m in commands[start:])
        pump(2.);assert output().linear.x>.58
        result['checks'].append('Lifecycle cleanup/reconfigure/activate resets output and requires fresh prediction')
        ordered=sorted(durations)
        result['processing_ms']={'samples':len(ordered),'median':ordered[len(ordered)//2],'p95':ordered[int(.95*len(ordered))],'max':max(ordered)}
        result['success']=True
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close();node.destroy_node();rclpy.shutdown()
        Path(args.output).parent.mkdir(parents=True,exist_ok=True)
        Path(args.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
