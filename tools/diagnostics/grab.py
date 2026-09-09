"""Grab the live camera frame and run the book detector on it."""
import sys, time
import numpy as np, cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
sys.path.insert(0,'/opt/erc_ws/src/rafeeq_solution')
from rafeeq_solution import vision
from rafeeq_solution.image_utils import to_bgr
rclpy.init(); n=Node('grab'); st={}
n.create_subscription(Image,'/head_front_camera/head_front_camera/color/image_raw',
                      lambda m: st.__setitem__('img', to_bgr(m)), qos_profile_sensor_data)
t=time.time()
while 'img' not in st and time.time()-t<20: rclpy.spin_once(n, timeout_sec=0.1)
img=st.get('img')
if img is None: print('no image'); sys.exit(1)
cv2.imwrite('/tmp/live_view.png', img)
h,w=img.shape[:2]; print('image %dx%d'%(w,h))
books=vision.detect_books(img)
print('books detected: %d'%len(books))
for b in books:
    x,y,bw,bh=b['bbox']
    print('  %-7s bbox=(%3d,%3d,%3d,%3d) centroid=(%.0f,%.0f) area=%.0f aspect=%.2f'%(
        b['colour'],x,y,bw,bh,b['centroid'][0],b['centroid'][1],b['area'],bw/float(bh)))
print()
# what does the raw colour mask say, before the shape filters?
for colour in ('red','green','blue','yellow'):
    m=vision.colour_mask(img,colour)
    cnts,_=cv2.findContours(m,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    big=[c for c in cnts if cv2.contourArea(c)>=60]
    print('%-7s blobs>=60px: %d'%(colour,len(big)))
    for c in sorted(big,key=cv2.contourArea,reverse=True)[:4]:
        x,y,bw,bh=cv2.boundingRect(c); a=cv2.contourArea(c)
        print('    bbox=(%3d,%3d,%3d,%3d) area=%.0f aspect=%.2f fill=%.2f  %s'%(
            x,y,bw,bh,a,bw/float(bh),a/float(bw*bh),
            'PASS' if (0.04<=bw/float(bh)<=0.60 and a>=100 and a/(bw*bh)>=0.55) else 'rejected'))
rclpy.shutdown()
