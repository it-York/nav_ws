import json,math,subprocess,time
from pathlib import Path
import yaml,numpy as np
from PIL import Image
import rclpy
from rclpy.action import ActionClient
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped,Twist,PoseStamped
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
ROOT=Path('/home/koala2/nav_ws');TMP=Path('/tmp/nav_ws_narrow_check');TMP.mkdir(exist_ok=True)
xx,yy=np.meshgrid((np.arange(120)+.5)*.05,(np.arange(80)+.5)*.05)
free=(xx>.5)&(xx<5.5)&(yy>1.6)&(yy<2.4)
Image.fromarray(np.flipud(np.where(free,254,0).astype('uint8'))).save(TMP/'map.pgm')
(TMP/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[0.,0.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
rclpy.init();n=rclpy.create_node('narrow_simulation_test');tf=TransformBroadcaster(n);stf=StaticTransformBroadcaster(n)
st=TransformStamped();st.header.frame_id='map';st.child_frame_id='odom';st.transform.rotation.w=1.;stf.sendTransform(st)
odom=n.create_publisher(Odometry,'/odom',10)
s={'x':1.2,'y':2.,'yaw':0.,'v':0.,'w':0.,'run':False,'last':time.monotonic(),'collision':False,'max_v':0.}
def cmd(m):s['v']=m.linear.x;s['w']=m.angular.z
sub=n.create_subscription(Twist,'/cmd_vel',cmd,10)
def tick():
 now=time.monotonic();dt=min(now-s['last'],.1);s['last']=now
 if s['run']:
  s['x']+=s['v']*math.cos(s['yaw'])*dt;s['y']+=s['v']*math.sin(s['yaw'])*dt;s['yaw']+=s['w']*dt;s['max_v']=max(s['max_v'],s['v'])
  for a,b in [(-.48,-.3),(-.48,.3),(.48,.3),(.48,-.3)]:
   x=s['x']+math.cos(s['yaw'])*a-math.sin(s['yaw'])*b;y=s['y']+math.sin(s['yaw'])*a+math.cos(s['yaw'])*b
   if not (.5<x<5.5 and 1.6<y<2.4):s['collision']=True
 t=TransformStamped();t.header.frame_id='odom';t.child_frame_id='base_link';t.header.stamp=n.get_clock().now().to_msg();t.transform.translation.x=s['x'];t.transform.translation.y=s['y'];t.transform.rotation.z=math.sin(s['yaw']/2);t.transform.rotation.w=math.cos(s['yaw']/2);tf.sendTransform(t)
 o=Odometry();o.header=t.header;o.child_frame_id='base_link';o.pose.pose.position.x=s['x'];o.pose.pose.position.y=s['y'];o.pose.pose.orientation=t.transform.rotation;o.twist.twist.linear.x=s['v'];o.twist.twist.angular.z=s['w'];odom.publish(o)
timer=n.create_timer(.05,tick)
def wait(f,sec=12):
 rclpy.spin_until_future_complete(n,f,timeout_sec=sec)
 if not f.done():raise RuntimeError('timeout')
 return f.result()
def change(name,num):
 c=n.create_client(ChangeState,'/'+name+'/change_state');assert c.wait_for_service(timeout_sec=10)
 req=ChangeState.Request();req.transition.id=num;assert wait(c.call_async(req)).success;n.destroy_client(c)
def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
results={}
for label,file in [('before',ROOT/'config_backups/narrow_passage_20260918/nav2_go2w.yaml'),('after',ROOT/'src/go2w_nav2_navigation/config/nav2_go2w.yaml')]:
 c=yaml.safe_load(file.read_text());c['map_server']['ros__parameters']['yaml_filename']=str(TMP/'map.yaml')
 l=c['local_costmap']['local_costmap']['ros__parameters'];l['plugins']=['static_layer','inflation_layer'];l['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True}
 p=TMP/(label+'.yaml');p.write_text(yaml.safe_dump(c));procs=[];files=[]
 s.update(x=1.2,y=2.,yaw=0.,v=0.,w=0.,run=False,collision=False,max_v=0.)
 try:
  for pkg,exe in [('nav2_map_server','map_server'),('nav2_controller','controller_server')]:
   f=(TMP/(label+'_'+exe+'.log')).open('w');files.append(f);procs.append(subprocess.Popen(['/opt/ros/humble/lib/'+pkg+'/'+exe,'--ros-args','--params-file',str(p)],stdout=f,stderr=subprocess.STDOUT))
  change('map_server',1);change('map_server',3);change('controller_server',1);change('controller_server',3);spin(.5)
  client=ActionClient(n,FollowPath,'/follow_path');assert client.wait_for_server(timeout_sec=5)
  g=FollowPath.Goal();g.controller_id='FollowPath';g.goal_checker_id='goal_checker';g.path.header.frame_id='map'
  for x in np.linspace(1.2,4.8,73):
   pose=PoseStamped();pose.header.frame_id='map';pose.pose.position.x=float(x);pose.pose.position.y=2.;pose.pose.orientation.w=1.;g.path.poses.append(pose)
  h=wait(client.send_goal_async(g));assert h.accepted;result=h.get_result_async();s['run']=True;start=time.monotonic()
  while not result.done() and time.monotonic()-start<15 and not s['collision']:rclpy.spin_once(n,timeout_sec=.02)
  s['run']=False
  results[label]={k:s[k] for k in ['x','y','yaw','collision','max_v']};results[label]['duration_s']=time.monotonic()-start;results[label]['status']=result.result().status if result.done() else 'timeout'
  if not result.done():wait(h.cancel_goal_async())
  client.destroy()
 finally:
  s['run']=False
  for p in procs:p.terminate()
  for p in procs:
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:p.kill();p.wait()
  for f in files:f.close()
  spin(.5)
print(json.dumps(results,indent=2));(TMP/'result.json').write_text(json.dumps(results,indent=2));n.destroy_node();rclpy.shutdown()
assert results['after']['status']==4 and not results['after']['collision'],results
