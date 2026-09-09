"""Is the depth image planar z, or Euclidean range?

Sample a horizontal line across a flat, fronto-parallel surface (the shelf
face). Planar depth is constant across the line; Euclidean range grows as
sqrt(1 + ((u-cx)/fx)^2).
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
sys.path.insert(0,'/opt/erc_ws/src/rafeeq_solution')
from rafeeq_solution.image_utils import PinholeModel, depth_to_metres
rclpy.init(); n=Node('dk'); st={}
n.create_subscription(Image,'/head_front_camera/head_front_camera/depth/image_rect_raw',
                      lambda m: st.update(d=depth_to_metres(m)), qos_profile_sensor_data)
n.create_subscription(CameraInfo,'/head_front_camera/head_front_camera/color/camera_info',
                      lambda m: st.update(c=PinholeModel(m)), qos_profile_sensor_data)
t=time.time()
while ('d' not in st or 'c' not in st) and time.time()-t<15: rclpy.spin_once(n,timeout_sec=0.05)
d=st['d']; cam=st['c']
v=int(cam.cy)
print('row v=%d, fx=%.2f cx=%.1f'%(v,cam.fx,cam.cx))
print('%-6s %-9s %-11s %-11s'%('u','depth','d/d_centre','range model'))
centre=np.nanmedian(d[v-2:v+3, int(cam.cx)-2:int(cam.cx)+3])
for u in range(60, 600, 60):
    val=np.nanmedian(d[v-2:v+3, u-2:u+3])
    if not np.isfinite(val): print('%-6d  no return'%u); continue
    model=np.sqrt(1.0+((u-cam.cx)/cam.fx)**2)
    print('%-6d %-9.4f %-11.4f %-11.4f'%(u,val,val/centre,model))
print()
print('If d/d_centre tracks the range model, the image is EUCLIDEAN RANGE and')
print('deproject() must divide by that factor before using it as z.')
rclpy.shutdown()
