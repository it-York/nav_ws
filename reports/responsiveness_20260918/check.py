import json,os,subprocess,time
from pathlib import Path
import yaml
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped,Twist,PolygonStamped,Point32
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
ROOT=Path('/home/koala2/nav_ws');TMP=Path('/tmp/nav_ws_response_check');TMP.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node('response_configuration_test');b=StaticTransformBroadcaster(n)
tfs=[]
# Negative world Z reproduces the flat map's vertical drift.
for parent,child,x,y,z in [('map','odom',0.,0.,0.),('odom','base_link',1.2,1.8,-2.),('base_link','body',0.,0.,0.),('base_link','lidar_link',0.,0.,.15)]:
 t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.translation.x=x;t.transform.translation.y=y;t.transform.translation.z=z;t.transform.rotation.w=1.;tfs.append(t)
b.sendTransform(tfs)
cloud=n.create_publisher(PointCloud2,'/cloud_obstacles',qos_profile_sensor_data)
raw=n.create_publisher(PointCloud2,'/cloud_registered_body',qos_profile_sensor_data)
cmd=n.create_publisher(Twist,'/cmd_vel_smoothed',10)
foot=n.create_publisher(PolygonStamped,'/local_costmap/published_footprint',10)
state={'mode':'obstacle','monitor':False}
def publish():
 stamp=n.get_clock().now().to_msg();h=Header(stamp=stamp,frame_id='base_link');rh=Header(stamp=stamp,frame_id='body')
 if state['mode'] in ['obstacle','obstacle_and_background']:
  pts=[(1.525,.025,.4),(1.525,.03,.5)];rp=pts+([(3.05,y,-.4) for y in [-.2,-.15,-.1,-.05,0.,.05,.1,.15,.2]] if state['mode']=='obstacle_and_background' else [])
 elif state['mode']=='background':pts=[];rp=[(3.05,y,-.4) for y in [-.2,-.15,-.1,-.05,0.,.05,.1,.15,.2]]
 elif state['mode']=='no_returns':pts=[];rp=[]
 elif state['mode']=='near_wall':pts=[(.8,0.,.4),(.8,.02,.4)];rp=pts
 elif state['mode']=='stop':pts=[(.49,.29,.4)];rp=pts
 else:pts=[];rp=[]
 cloud.publish(create_cloud_xyz32(h,pts));raw.publish(create_cloud_xyz32(rh,rp))
 if state['monitor']:
  p=PolygonStamped(header=h);p.polygon.points=[Point32(x=x,y=y,z=0.) for x,y in [(-.48,-.3),(-.48,.3),(.48,.3),(.48,-.3)]];foot.publish(p)
  v=Twist();v.linear.x=0. if state['mode']=='turn' else .6;v.angular.z=.7 if state['mode']=='turn' else 0.;cmd.publish(v)
timer=n.create_timer(.1,publish)
def wait(f,timeout=12):
 rclpy.spin_until_future_complete(n,f,timeout_sec=timeout)
 if not f.done():raise RuntimeError('timeout')
 return f.result()
def change(name,t):
 c=n.create_client(ChangeState,'/'+name+'/change_state');assert c.wait_for_service(timeout_sec=10),name
 req=ChangeState.Request();req.transition.id=t;assert wait(c.call_async(req)).success,(name,t);n.destroy_client(c)
def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
results={}
for label,file in [('before',ROOT/'config_backups/responsiveness_20260918/nav2_go2w.yaml'),('after',ROOT/'src/go2w_nav2_navigation/config/nav2_go2w.yaml')]:
 cfg=yaml.safe_load(file.read_text());cfg['map_server']['ros__parameters']['yaml_filename']=str(Path(__file__).resolve().parent/'corner.yaml')
 # Use correct low map-Z limits in baseline for clearing comparison so that
 # the old layer-height bug does not prevent placing the test obstacle.
 cfg['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer'].update(min_obstacle_height=-5.,max_obstacle_height=5.)
 config=TMP/(label+'.yaml');config.write_text(yaml.safe_dump(cfg));procs=[];logs=[];costs=[];out=[];state['mode']='obstacle';state['monitor']=False
 def cost_cb(m):
  ix=int((2.725-m.metadata.origin.position.x)/m.metadata.resolution);iy=int((1.825-m.metadata.origin.position.y)/m.metadata.resolution)
  if 0<=ix<m.metadata.size_x and 0<=iy<m.metadata.size_y:costs.append((time.monotonic(),int(m.data[iy*m.metadata.size_x+ix])))
 sub=n.create_subscription(Costmap,'/global_costmap/costmap_raw',cost_cb,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 subcmd=n.create_subscription(Twist,'/cmd_vel',lambda m:out.append((m.linear.x,m.angular.z)),10)
 try:
  packages=[('nav2_map_server','map_server'),('nav2_planner','planner_server'),('nav2_controller','controller_server')]
  if label=='after':packages.append(('nav2_collision_monitor','collision_monitor'))
  for package,exe in packages:
   f=(TMP/(label+'_'+exe+'.log')).open('w');logs.append(f)
   procs.append(subprocess.Popen(['/opt/ros/humble/lib/'+package+'/'+exe,'--ros-args','--params-file',str(config)],stdout=f,stderr=subprocess.STDOUT))
  change('map_server',1);change('map_server',3);change('planner_server',1);change('planner_server',3);change('controller_server',1)
  spin(2);assert costs and costs[-1][1]==254,(label,'obstacle not marked',costs[-5:])
  state['mode']='no_returns';spin(1);assert costs[-1][1]==254,(label,'missing returns incorrectly clear obstacles')
  state['mode']='background';beg=time.monotonic();spin(2)
  cleared=next((t-beg for t,v in costs if t>=beg and v==0),None)
  results[label]={'observed_background_clear_delay_s':cleared,'last_cost':costs[-1][1],'missing_returns_keep_obstacle':True}
  if label=='after':
   assert cleared is not None and cleared<1.,results
   # A continuously observed obstacle must survive simultaneous background rays.
   state['mode']='obstacle_and_background';spin(1);assert costs[-1][1]==254
   change('collision_monitor',1);change('collision_monitor',3);state['monitor']=True
   for mode in ['empty','near_wall','stop','turn']:
    state['mode']=mode;out.clear();spin(1.2);assert out,(mode,'no command')
    results[label][mode+'_last_vx']=out[-1][0]
    if mode=='turn':results[label]['empty_last_wz']=out[-1][1]
   assert abs(results[label]['empty_last_vx']-.6)<1e-6
   assert abs(results[label]['near_wall_last_vx']-.33)<1e-6
   assert results[label]['stop_last_vx']==0.
   assert abs(results[label]['empty_last_wz']-.7)<1e-6
 finally:
  state['monitor']=False
  n.destroy_subscription(sub);n.destroy_subscription(subcmd)
  for p in procs:p.terminate()
  for p in procs:
   try:p.wait(timeout=6)
   except subprocess.TimeoutExpired:p.kill();p.wait()
  for f in logs:f.close()
  spin(1)
print(json.dumps(results,indent=2));(TMP/'result.json').write_text(json.dumps(results,indent=2));n.destroy_node();rclpy.shutdown()
