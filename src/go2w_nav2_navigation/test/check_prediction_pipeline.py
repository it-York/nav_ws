#!/usr/bin/env python3
"""Loopback-only integration; robot API is forcibly remapped to /test/sport/request."""
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
from fast_lio.msg import EstimatorState
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Float32
from lifecycle_msgs.srv import ChangeState
from unitree_api.msg import Request


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '190' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires ROS_DOMAIN_ID=190 ROS_LOCALHOST_ONLY=1')
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    root=Path(__file__).resolve().parents[3]
    work=Path(tempfile.mkdtemp(prefix='nav_ws_prediction_test_'))
    os.environ['ROS_LOG_DIR']=str(work/'ros_logs')
    rclpy.init();node=rclpy.create_node('prediction_pipeline_test')
    processes=[];logs=[];result={'work_directory':str(work),'checks':[]}
    odoms=[];valid=[];reason=[];commands=[];requests=[];processing=[];monitor_reason=[];guard_requests=[]
    def received_odom(m): odoms.append((node.get_clock().now().nanoseconds,m))
    subs=[node.create_subscription(Odometry,'/odom',received_odom,qos_profile_sensor_data),
          node.create_subscription(Bool,'/odom_prediction/valid',lambda m:valid.append(m.data),10),
          node.create_subscription(String,'/odom_prediction/reason',lambda m:reason.append(m.data),10),
          node.create_subscription(Twist,'/cmd_vel',lambda m:commands.append(m),10),
          node.create_subscription(Request,'/test/sport/request',lambda m:requests.append(m),10),
          node.create_subscription(Float32,'/collision_monitor/processing_ms',lambda m:processing.append(m.data),10),
          node.create_subscription(String,'/collision_monitor/reason',lambda m:monitor_reason.append(m.data),10)]
    subs.append(node.create_subscription(Request,'/test/guard/request',lambda m:guard_requests.append(m),10))
    guard_cmd=node.create_publisher(Twist,'/test/guard/cmd',1)
    guard_odom=node.create_publisher(Odometry,'/test/guard/odom',1)
    guard_cloud=node.create_publisher(PointCloud2,'/test/guard/cloud',1)
    guard_mode='valid'
    anchor_pub=node.create_publisher(EstimatorState,'/fastlio/estimator_state',10)
    imu_pub=node.create_publisher(Imu,'/livox/imu',qos_profile_sensor_data)
    cloud_pub=node.create_publisher(PointCloud2,'/cloud_obstacles',5)
    cmd_pub=node.create_publisher(Twist,'/cmd_vel_smoothed',1)
    origin=node.get_clock().now().nanoseconds
    anchor_delay=.08; speed=.6; yaw_rate=.7
    cloud_mode='clear';feed_imu=True;feed_anchor=True;feed_cloud=True;anchor_shift=0.
    last_anchor_stamp=0;last_imu_stamp=0;next_imu_ns=origin
    def stamp(ns): return rclpy.time.Time(nanoseconds=ns).to_msg()
    def pose(t): return speed*t, yaw_rate*t
    def pump(seconds):
        nonlocal last_anchor_stamp,last_imu_stamp,next_imu_ns
        end=time.monotonic()+seconds;ni=na=nc=nv=0.
        while time.monotonic()<end:
            wall=time.monotonic();ns=node.get_clock().now().nanoseconds
            if feed_imu:
                # Model the hardware's 200 Hz sampling clock independently of
                # Python callback scheduling. Deliberate outages below discard
                # missing samples rather than filling them in on recovery.
                while next_imu_ns <= ns-2_000_000:
                    m=Imu();m.header.frame_id='livox_frame';m.header.stamp=stamp(next_imu_ns)
                    m.angular_velocity.z=yaw_rate;m.linear_acceleration.z=1.
                    imu_pub.publish(m);last_imu_stamp=next_imu_ns;next_imu_ns+=5_000_000
            else:
                next_imu_ns=ns-2_000_000
            if feed_anchor and wall>=na:
                anchor_ns=ns-int(anchor_delay*1e9)
                if anchor_ns>last_anchor_stamp:
                    m=EstimatorState();m.header.frame_id='camera_init';m.header.stamp=stamp(anchor_ns)
                    t=(anchor_ns-origin)/1e9;x,yaw=pose(t)
                    m.pose.pose.position.x=x+anchor_shift;m.pose.pose.orientation.w=math.cos(yaw/2);m.pose.pose.orientation.z=math.sin(yaw/2)
                    m.velocity_world.x=speed;m.gravity_world.z=-9.809;m.accel_scale=9.809
                    for i in range(6):m.pose.covariance[i*6+i]=.001
                    anchor_pub.publish(m);last_anchor_stamp=anchor_ns
                na=wall+.1
            if feed_cloud and wall>=nc:
                m=PointCloud2();m.header.frame_id='missing_frame' if cloud_mode=='bad_tf' else 'base_link'
                m.header.stamp=stamp(ns-(2_000_000_000 if cloud_mode=='stale' else 80_000_000))
                m.height=1;m.point_step=12;m.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
                points=[] if cloud_mode in ('clear','bad_tf','stale','malformed') else [(.1,0.,.3)]
                m.width=len(points);m.row_step=m.width*12;m.data=b''.join(struct.pack('<fff',*p) for p in points)
                if cloud_mode=='malformed':m.fields=[]
                if cloud_mode=='bad_offset':m.fields[0].offset=4294967295
                cloud_pub.publish(m);nc=wall+.1
            if wall>=nv:
                m=Twist();m.linear.x=.3;cmd_pub.publish(m);nv=wall+.05
                gm=Twist();gm.linear.x=float('nan') if guard_mode=='nan_command' else .3;guard_cmd.publish(gm)
                go=Odometry();go.header.frame_id='odom';go.child_frame_id='base_link';go.pose.pose.orientation.w=1.
                go.header.stamp=stamp(ns-(1_000_000_000 if guard_mode=='stale_odom' else -1_000_000_000 if guard_mode=='future_odom' else 20_000_000))
                guard_odom.publish(go)
                gc=PointCloud2();gc.header.frame_id='base_link';gc.header.stamp=stamp(ns-(1_000_000_000 if guard_mode=='stale_cloud' else 20_000_000));guard_cloud.publish(gc)
            rclpy.spin_once(node,timeout_sec=.001)
            for p in processes:
                if p.poll() is not None:raise AssertionError(f'Child exited {p.returncode}; {work}')
    def spawn(pkg,exe,extra):
        f=open(work/(exe+'.log'),'a');logs.append(f)
        p=subprocess.Popen([str(root/'install'/pkg/'lib'/pkg/exe),'--ros-args',*extra],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p);return p
    def transition(number):
        client=node.create_client(ChangeState,'/collision_monitor/change_state')
        assert client.wait_for_service(timeout_sec=5),'No monitor lifecycle service'
        request=ChangeState.Request();request.transition.id=number
        future=client.call_async(request)
        end=time.monotonic()+10
        while not future.done() and time.monotonic()<end:pump(.02)
        assert future.done() and future.result().success,f'Transition {number} failed; {work}'
        node.destroy_client(client)
    def moving():return commands and commands[-1].linear.x>.01
    def stopped():return commands and commands[-1].linear.x==0 and commands[-1].angular.z==0
    try:
        config=yaml.safe_load((root/'src/go2w_nav2_navigation/config/nav2_go2w.yaml').read_text())
        monitor_config=work/'monitor.yaml';monitor_config.write_text(yaml.safe_dump({'collision_monitor':config['collision_monitor']}))
        predictor=spawn('go2w_nav2_navigation','fastlio_imu_odometry',[])
        spawn('go2w_nav2_navigation','safe_collision_monitor',['--params-file',str(monitor_config)])
        spawn('go2w_nav2_bridge','cmd_vel_to_sport',['-p','require_wheeled_mode:=false','-p','require_prediction:=true','-p','odom_timeout:=0.15','-p','require_obstacle_cloud:=true','-p','obstacle_cloud_timeout:=0.3',
            '-r','obstacle_cloud:=/cloud_obstacles','-r','api/sport/request:=/test/sport/request'])
        spawn('go2w_nav2_bridge','cmd_vel_to_sport',['-r','__node:=guard_test_bridge',
            '-p','require_wheeled_mode:=false','-p','require_prediction:=true','-p','odom_timeout:=0.15',
            '-p','require_obstacle_cloud:=true','-p','obstacle_cloud_timeout:=0.3',
            '-r','cmd_vel:=/test/guard/cmd','-r','odom:=/test/guard/odom',
            '-r','obstacle_cloud:=/test/guard/cloud','-r','api/sport/request:=/test/guard/request'])
        pump(1.0);transition(1);transition(3);pump(2.)
        assert valid and valid[-1],reason[-10:]
        assert moving(),monitor_reason[-10:]
        assert any(m.header.identity.api_id==1008 for m in requests),'Bridge did not issue remapped test Move'
        result['checks'].append('Valid prediction and motion-compensated empty cloud permit test command')
        start=len(odoms);pump(2.);sample=odoms[start:]
        assert len(sample)>70, f'prediction below 35 Hz: {len(sample)/2}'
        errors=[];ages=[]
        # Real 30-degree mounting: base offset in IMU frame is -R_base_imu^-1 * t_base_imu.
        bx,by,bz=.278256279,.023290,.112620959;c=math.cos(math.pi/6);s=math.sin(math.pi/6)
        lx,ly=-(c*bx-s*bz),-by
        for received,m in sample:
            ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            x,yaw=pose((ns-origin)/1e9)
            expected=(x+math.cos(yaw)*lx-math.sin(yaw)*ly, math.sin(yaw)*lx+math.cos(yaw)*ly)
            errors.append(math.hypot(m.pose.pose.position.x-expected[0],m.pose.pose.position.y-expected[1]))
            ages.append((received-ns)/1e6)
        assert max(errors)<.002,max(errors)
        result['prediction_rate_hz']=len(sample)/2
        result['pose_xy_error_max_m']=max(errors)
        result['odom_receipt_age_ms']={'median':sorted(ages)[len(ages)//2],'p95':sorted(ages)[int(.95*len(ages))],'max':max(ages)}
        result['checks'].append('50 Hz delayed-anchor prediction matches analytic translating / turning trajectory with actual mounting')
        # Exercise the read-only auditor: odom queries should succeed, absent
        # map TF and localization MUST fail the strict commissioning gate.
        audit_file=work/'audit_without_global_localization.json'
        with open(work/'audit.log','w') as audit_log:
            audit=subprocess.Popen(['python3',str(root/'scripts/check_tf_timing.py'),
                '--duration','3','--strict','--output',str(audit_file)],
                stdout=audit_log,stderr=subprocess.STDOUT)
            try:
                pump(4.)
                assert audit.wait(timeout=3)==2
            finally:
                if audit.poll() is None:audit.terminate();audit.wait(timeout=3)
        audit_result=json.loads(audit_file.read_text())
        assert not audit_result['tf_lookup_failures'].get('odom -> base_link at request time',0),audit_result
        assert audit_result['tf_lookup_wait_ms']['odom -> base_link at request time']['samples']>0
        assert audit_result['tf_lookup_failures'].get('map -> base_link at request time',0)>0
        assert not audit_result['acceptance']['passed']
        result['checks'].append('Timing auditor resolves current-time odom within 50 ms and rejects missing map TF')
        anchor_delay=.175;pump(2.);assert valid[-1] and moving(),(reason[-10:],monitor_reason[-10:])
        result['checks'].append('175 ms estimator delay remains inside bounded prediction horizon')
        for mode,expected_reason in [('obstacle','obstacle_stop'),('bad_tf','pointcloud_tf_unavailable'),('stale','stale_or_future_cloud'),('malformed','invalid_cloud_layout'),('bad_offset','invalid_cloud_layout')]:
            cloud_mode=mode;pump(.5);assert stopped(),mode
            assert expected_reason in monitor_reason[-12:],monitor_reason[-12:]
            cloud_mode='clear';pump(.5);assert moving(),monitor_reason[-10:]
            result['checks'].append(mode+' stops; valid fresh cloud recovers')
        before=len(requests);feed_imu=False;pump(.25)
        assert not valid[-1] and stopped(),(reason[-5:],monitor_reason[-5:])
        assert any(m.header.identity.api_id==1003 for m in requests[before:]),'Missing StopMove on IMU outage'
        feed_imu=True;pump(.7);assert valid[-1] and moving(),reason[-10:]
        result['checks'].append('IMU outage inhibits TF and commands; fresh anchored IMU history recovers')
        feed_cloud=False;pump(.6);assert stopped();feed_cloud=True;pump(.5);assert moving()
        result['checks'].append('Pointcloud outage stops; fresh pointcloud recovers')
        assert guard_requests[-1].header.identity.api_id==1008
        for mode in ('stale_odom','future_odom','stale_cloud','nan_command'):
            guard_mode=mode;pump(.4)
            assert guard_requests[-1].header.identity.api_id==1003,mode
            guard_mode='valid';pump(.3)
            assert guard_requests[-1].header.identity.api_id==1008,mode+' did not recover'
            result['checks'].append('Independent motion bridge rejects '+mode+' despite fresh message receipt')
        # Clock rollback must latch even if valid messages resume.
        m=Imu();m.header.frame_id='livox_frame';m.header.stamp=stamp(last_imu_stamp-200_000_000);m.linear_acceleration.z=1.
        imu_pub.publish(m);pump(.5)
        assert not valid[-1] and stopped() and 'rollback' in reason[-1],reason[-10:]
        result['checks'].append('IMU timestamp rollback latches stop until predictor restart')
        def restart_predictor():
            nonlocal predictor
            os.killpg(predictor.pid,signal.SIGINT);predictor.wait(timeout=5);processes.remove(predictor)
            predictor=spawn('go2w_nav2_navigation','fastlio_imu_odometry',[]);pump(1.)
            assert valid[-1] and moving(),(reason[-10:],monitor_reason[-10:])
        restart_predictor()
        result['checks'].append('Explicit predictor restart restores valid output after rollback')
        feed_anchor=False;pump(.5)
        assert not valid[-1] and stopped(),reason[-10:]
        feed_anchor=True;pump(.5)
        assert not valid[-1] and 'estimator_gap' in reason[-1],reason[-10:]
        result['checks'].append('Estimator outage exceeds horizon and latches stop when correction resumes')
        restart_predictor()
        anchor_shift=.5;pump(.5)
        assert not valid[-1] and stopped() and 'jump' in reason[-1],reason[-10:]
        result['checks'].append('Half-metre estimator correction jump latches stop')
        result['monitor_processing_ms']={'samples':len(processing),'median':sorted(processing)[len(processing)//2],'max':max(processing)}
        result['success']=True
    finally:
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for f in logs:f.close()
        node.destroy_node();rclpy.shutdown()
        Path(args.output).parent.mkdir(parents=True,exist_ok=True)
        Path(args.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
