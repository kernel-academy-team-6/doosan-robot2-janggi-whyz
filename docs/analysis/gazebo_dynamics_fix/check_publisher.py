"""Check the compiled stroke publisher in a separate ROS domain; no robot driver."""
import json, os, subprocess, time
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='213'
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
ROOT=Path(__file__).resolve().parents[3]
rclpy.init();node=rclpy.create_node('isolated_stroke_check');rows=[]
def callback(message):
 if 'gripper_rh_r1' in message.name:
  rows.append((time.monotonic(),message.position[message.name.index('gripper_rh_r1')]))
node.create_subscription(JointState,'/isolated_hand/joint_states',callback,10)
pub=node.create_publisher(Int32,'/isolated_hand/gripper/stroke',10)
process=subprocess.Popen([str(ROOT/'install/e0509_gripper_description/lib/e0509_gripper_description/gripper_joint_publisher'),'--ros-args','-r','__ns:=/isolated_hand'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
try:
 deadline=time.monotonic()+8
 while pub.get_subscription_count()==0:
  if time.monotonic()>deadline:raise RuntimeError('No isolated stroke subscriber')
  rclpy.spin_once(node,timeout_sec=.05)
 pub.publish(Int32(data=700));start=time.monotonic()
 while time.monotonic()-start<2.5:rclpy.spin_once(node,timeout_sec=.02)
 assert rows and abs(rows[-1][1]-1.0)<1e-8
 max_step=max(abs(b[1]-a[1]) for a,b in zip(rows,rows[1:]))
 assert max_step<=8*1.0/700+1e-8
 summary=dict(ros_domain_id=213,samples=len(rows),final_rad=rows[-1][1],max_step_rad=max_step,max_nominal_rad_per_sec=max_step/.02)
 (Path(__file__).parent/'publisher_check.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
finally:
 process.terminate();process.wait(timeout=5);node.destroy_node();rclpy.shutdown()
