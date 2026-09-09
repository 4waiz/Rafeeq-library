import sys, time, math, subprocess, re
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
sys.path.insert(0,'/opt/erc_ws/src/rafeeq_solution')
from rafeeq_solution import arena
out=subprocess.run(['gz','model','-m','tiago_pro','-p'],capture_output=True,text=True,timeout=30).stdout
nums=re.findall(r'\[([^\[\]]*)\]',out)
vals=[]
for g in nums:
    try: v=[float(x) for x in g.replace(',',' ').split()]
    except ValueError: continue
    if len(v)==3: vals.append(v)
print('gz truth  xyz=%s  rpy=%s'%(vals[0] if vals else '?', vals[1] if len(vals)>1 else '?'))
rclpy.init(); n=Node('yc'); st={}
def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
n.create_subscription(Odometry,'/odom',lambda m: st.update(
    o=(m.pose.pose.position.x,m.pose.pose.position.y,yaw_of(m.pose.pose.orientation))),20)
def jcb(m):
    for a,b in zip(m.name,m.position):
        if a in ('head_1_joint','head_2_joint','torso_lift_joint'): st[a]=b
n.create_subscription(JointState,'/joint_states',jcb,10)
t=time.time()
while time.time()-t<6: rclpy.spin_once(n,timeout_sec=0.05)
o=st['o']; w=arena.odom_to_world(o[0],o[1])
print('odom      xyz=(%.4f, %.4f)  yaw=%.4f'%(o[0],o[1],o[2]))
print('-> world  xy =(%.4f, %.4f)  yaw=%.4f'%(w[0],w[1],arena.odom_yaw_to_world(o[2])))
print('head_1 (pan) =%.4f  head_2 (tilt)=%.4f  torso=%.4f'%(
    st.get('head_1_joint',float('nan')),st.get('head_2_joint',float('nan')),
    st.get('torso_lift_joint',float('nan'))))
if vals:
    print()
    print('TRUE world yaw = %.4f ; odom-derived world yaw = %.4f ; error = %+.4f rad'%(
        vals[1][2], arena.odom_yaw_to_world(o[2]), vals[1][2]-arena.odom_yaw_to_world(o[2])))
    print('TRUE world xy  = (%.4f, %.4f) ; odom-derived = (%.4f, %.4f)'%(
        vals[0][0],vals[0][1],w[0],w[1]))
rclpy.shutdown()
