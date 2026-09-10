"""DIAGNOSTIC: exercise the staged grasp end-to-end.
Ground truth supplies the target and the oracle, isolating manipulation from
perception. The shipped solution never reads it."""
import time, sys, math, subprocess, re, os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts
sys.path.insert(0,'/opt/erc_ws/src/rafeeq_solution')
from rafeeq_solution import arena
TARGET=os.environ.get('BOOK','book_col_3_row_3_red'); SLOT=int(os.environ.get('SLOT','3'))
def pose_of(m):
    out=subprocess.run(['gz','model','-m',m,'-p'],capture_output=True,text=True,timeout=25).stdout
    for g in re.findall(r'\[([^\[\]]*)\]',out):
        try: v=[float(x) for x in g.replace(',',' ').split()]
        except ValueError: continue
        if len(v)==3: return v
def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
rclpy.init(); n=Node('staged')
pub=n.create_publisher(String,'/rafeeq/manip/command',10)
cmd=n.create_publisher(Twist,'/cmd_vel',10)
st={'s':[],'ready':False,'hold':0.0,'odom':None,'c':{}}
n.create_subscription(String,'/rafeeq/manip/status',lambda m: st['s'].append(m.data),10)
n.create_subscription(Bool,'/rafeeq/manip/ready',lambda m: st.__setitem__('ready',m.data),10)
n.create_subscription(Float32,'/rafeeq/manip/hold',lambda m: st.__setitem__('hold',m.data),10)
n.create_subscription(Odometry,'/odom',lambda m: st.__setitem__('odom',
 (m.pose.pose.position.x,m.pose.pose.position.y,yaw_of(m.pose.pose.orientation))),20)
def ccb(m):
    for c in m.contacts:
        a,b=c.collision1.name,c.collision2.name
        link=a if 'tiago' in a else b; other=b if 'tiago' in a else a
        k=((link.split('::')[1] if '::' in link else link),other.split('::')[0])
        st['c'][k]=st['c'].get(k,0)+1
n.create_subscription(Contacts,'/contacts',ccb,10)
def spin(s):
    t=time.time()
    while time.time()-t<s: rclpy.spin_once(n,timeout_sec=0.03)
SEQ=[0]
def send(verb,*a,**kw):
    timeout=kw.get('timeout',300)
    SEQ[0]+=1; sq=SEQ[0]; st['s']=[]
    pub.publish(String(data='|'.join([str(sq),verb]+[str(x) for x in a])))
    t0=time.time()
    while time.time()-t0<timeout:
        spin(0.2)
        for m in st['s']:
            if m.startswith('done|%d|'%sq):   print('   done: %s'%m.split('|',3)[-1][:120]); return True,m
            if m.startswith('failed|%d|'%sq): print('   FAIL: %s'%m.split('|',3)[-1][:150]); return False,m
    print('   TIMEOUT'); return False,''
def drive(wx,wy,wyaw,tol=0.02,lat=0.008,timeout=140):
    gx,gy=arena.world_to_odom(wx,wy); gyaw=arena.world_yaw_to_odom(wyaw)
    t0=time.time()
    while time.time()-t0<timeout:
        spin(0.05)
        if st['odom'] is None: continue
        x,y,yaw=st['odom']; ex,ey=gx-x,gy-y; eyaw=arena.normalize_angle(gyaw-yaw)
        # Anisotropic, like navigation_node: error across the heading puts a
        # fingertip into the book's spine, error along it only changes grip depth.
        cg,sg=math.cos(gyaw),math.sin(gyaw)
        along=ex*cg+ey*sg; across=-ex*sg+ey*cg
        if abs(along)<tol and abs(across)<lat and abs(eyaw)<0.02: cmd.publish(Twist()); return True
        c,s=math.cos(-yaw),math.sin(-yaw)
        t=Twist(); t.linear.x=max(-0.25,min(0.25,0.9*(c*ex-s*ey)))
        t.linear.y=max(-0.25,min(0.25,0.9*(s*ex+c*ey))); t.angular.z=max(-0.5,min(0.5,1.5*eyaw))
        cmd.publish(t)
    cmd.publish(Twist()); return False
print('waiting for readiness...'); t=time.time()
while not st['ready'] and time.time()-t<180: spin(0.5)
print('ready=%s'%st['ready'])
if not st['ready']: rclpy.shutdown(); sys.exit(1)
b0=pose_of(TARGET); print('\nbook before %s'%[round(v,4) for v in b0])
print('\n[1] home'); send('home',timeout=240)
stage_x=b0[0]-arena.ARM_TRANSIT_STANDOFF
print('\n[2] drive to staging (%.3f, %.3f)'%(stage_x,b0[1]))
print('   arrived=%s'%drive(stage_x,b0[1],0.0))
print('\n[3] plan_grasp')
adv=arena.ARM_TRANSIT_STANDOFF-arena.GRASP_STANDOFF
ok,msg=send('plan_grasp','%.4f,%.4f,%.4f'%tuple(b0),SLOT,'%.4f'%adv,timeout=300)
if not ok: rclpy.shutdown(); sys.exit(1)
poses={}
for it in msg.split('|',3)[-1].split():
    if '=' in it: k,v=it.split('=',1); poses[k]=v
print('\n[4] arm_pregrasp'); st['c']={}
send('arm_pregrasp',timeout=300)
gx,gy,gyaw=[float(v) for v in poses['grasp'].split(',')]
print('\n[5] advance base to grasp pose (%.3f, %.3f)'%(gx,gy))
print('   arrived=%s'%drive(gx,gy,gyaw))
print('\n[6] refine_grasp'); send('refine_grasp',timeout=200)
if os.environ.get('STOP_AFTER_REFINE'):
    print('\n[stopping before close]'); rclpy.shutdown(); sys.exit(0)
print('\n[7] close_grasp'); st['c']={}
send('close_grasp',timeout=200)
print('\n[8] lift'); send('lift',timeout=200)
rx,ry,ryaw=[float(v) for v in poses['retreat'].split(',')]
print('\n[9] retreat base to (%.3f, %.3f)'%(rx,ry))
print('   arrived=%s'%drive(rx,ry,ryaw))
print('\n[10] check_hold'); send('check_hold',timeout=120)
spin(2.0)
b1=pose_of(TARGET)
print('\nbook after  %s'%[round(v,4) for v in b1])
d=[a-b for a,b in zip(b1,b0)]
print('DISPLACEMENT dx=%+.4f dy=%+.4f dz=%+.4f  |d|=%.4f m'%(d[0],d[1],d[2],sum(v*v for v in d)**0.5))
print('hold width %.4f m'%st['hold'])
print('contacts:', dict(list(st['c'].items())[:6]) or 'none')
print('\nVERDICT:', 'BOOK EXTRACTED' if abs(d[0])>0.15 else 'not extracted')
rclpy.shutdown()
