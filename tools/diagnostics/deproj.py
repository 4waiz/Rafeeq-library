"""Deproject known targets and compare with ground truth."""
import sys, time, math, subprocess, re
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import tf2_ros, tf2_geometry_msgs  # noqa
sys.path.insert(0,'/opt/erc_ws/src/rafeeq_solution')
from rafeeq_solution import arena, vision
from rafeeq_solution.image_utils import PinholeModel, depth_to_metres, patch_depth, patch_half_for_bbox, to_bgr

def pose_of(m):
    o=subprocess.run(['gz','model','-m',m,'-p'],capture_output=True,text=True,timeout=25).stdout
    for g in re.findall(r'\[([^\[\]]*)\]',o):
        try: v=[float(x) for x in g.replace(',',' ').split()]
        except ValueError: continue
        if len(v)==3: return v

rclpy.init(); n=Node('dp')
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
st={}
n.create_subscription(Image,'/head_front_camera/head_front_camera/color/image_raw',
                      lambda m: st.update(img=to_bgr(m), hdr=m.header), qos_profile_sensor_data)
n.create_subscription(Image,'/head_front_camera/head_front_camera/depth/image_rect_raw',
                      lambda m: st.update(depth=depth_to_metres(m)), qos_profile_sensor_data)
n.create_subscription(CameraInfo,'/head_front_camera/head_front_camera/color/camera_info',
                      lambda m: st.update(cam=PinholeModel(m)), qos_profile_sensor_data)
def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
n.create_subscription(Odometry,'/odom',lambda m: st.update(
    odom=(m.pose.pose.position.x,m.pose.pose.position.y,yaw_of(m.pose.pose.orientation))),20)
t=time.time()
while time.time()-t<8: rclpy.spin_once(n,timeout_sec=0.05)
if 'img' not in st or 'depth' not in st or 'cam' not in st:
    print('missing data', list(st)); sys.exit(1)
o=st['odom']; w=arena.odom_to_world(o[0],o[1])
print('robot odom (%.3f, %.3f, %.4f)  -> world (%.3f, %.3f) yaw %.4f'%(
    o[0],o[1],o[2],w[0],w[1],arena.odom_yaw_to_world(o[2])))
cam=st['cam']; depth=st['depth']; hdr=st['hdr']; img=st['img']
print('camera frame: %s  fx=%.2f cx=%.2f cy=%.2f'%(hdr.frame_id,cam.fx,cam.cx,cam.cy))
tr=buf.lookup_transform('odom',hdr.frame_id,Time()).transform
print('TF odom<-camera: t=(%.4f, %.4f, %.4f)'%(tr.translation.x,tr.translation.y,tr.translation.z))
cw=arena.odom_to_world(tr.translation.x,tr.translation.y)
print('camera in WORLD: (%.4f, %.4f, %.4f)'%(cw[0],cw[1],tr.translation.z))
print()
books=vision.detect_books(img)
print('%d books detected'%len(books))
for b in books:
    u,v=b['centroid']; half=patch_half_for_bbox(b['bbox'])
    d=patch_depth(depth,u,v,half=half)
    if d is None: print('  %-7s u=%.0f no depth'%(b['colour'],u)); continue
    x,y,z=cam.deproject(u,v,d)
    p=PointStamped(); p.header.frame_id=hdr.frame_id; p.header.stamp=Time().to_msg()
    p.point.x,p.point.y,p.point.z=x,y,z
    try:
        op=buf.transform(p,'odom',timeout=Duration(seconds=0.4))
    except Exception as e:
        print('  tf fail',e); continue
    ww=arena.odom_to_world(op.point.x,op.point.y)
    print('  %-7s u=%3.0f depth=%.3f  cam(%.3f,%.3f,%.3f) -> world (%.3f, %.3f, %.3f)'%(
        b['colour'],u,d,x,y,z,ww[0],ww[1],op.point.z))
print()
print('ground truth of nearby books:')
out=subprocess.run(['gz','model','--list'],capture_output=True,text=True,timeout=30).stdout
for name in sorted(set(re.findall(r'book_col_[23]_row_\d_\w+',out))):
    p=pose_of(name)
    if p: print('  %-26s world (%.3f, %.3f, %.3f)'%(name,p[0],p[1],p[2]))
rclpy.shutdown()
