"""How much of the error in a measured book position is drift, and how much is
the depth measurement itself?

This is the script that separated the two.  It found the depth patch reading
*between* the book and the shelf behind it, and the spine face being published
where every consumer expected the centre.

Measured book x scatters by +/-0.25 m against a true 2.900.  Two very different
things could cause that and they need opposite fixes:

* odometry drift moves the base's own estimate too, so it cancels when the
  robot drives to the measurement and nothing needs correcting;
* an error in the depth itself does not cancel, and puts the fingers in the
  wrong place.

So this reports both: each measurement against Gazebo ground truth, and the
base's own pose error at the same moment, which is the drift.  What is left
after removing the drift is the measurement.
"""
import math
import os
import re
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as MsgDuration
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PointStamped
from rclpy.time import Time

sys.path.insert(0, '/opt/erc_ws/src/rafeeq_solution')
from rafeeq_solution import arena, vision
from rafeeq_solution.image_utils import (PinholeModel, depth_to_metres,
                                         patch_depth, patch_half_for_bbox,
                                         to_bgr)
from ament_index_python.packages import get_package_share_directory

COL = int(os.environ.get('COL', '3'))

share = get_package_share_directory('erc_description')
TEMPLATES = vision.load_digit_templates(
    os.path.join(share, 'models', 'number_marker', 'textures'))


def gz_models(pattern):
    out = subprocess.run(['gz', 'model', '--list'], capture_output=True,
                         text=True, timeout=40).stdout
    return sorted(set(re.findall(pattern, out)))


def gz_pose(model):
    out = subprocess.run(['gz', 'model', '-m', model, '-p'],
                         capture_output=True, text=True, timeout=30).stdout
    for group in re.findall(r'\[([^\[\]]*)\]', out):
        try:
            vals = [float(x) for x in group.replace(',', ' ').split()]
        except ValueError:
            continue
        if len(vals) == 3:
            return vals
    return None


rclpy.init()
node = Node('depthcheck')
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, node)
cmd = node.create_publisher(Twist, '/cmd_vel', 10)
head = node.create_publisher(JointTrajectory,
                             '/head_controller/joint_trajectory', 10)
torso = node.create_publisher(JointTrajectory,
                              '/torso_controller/joint_trajectory', 10)
st = {}
node.create_subscription(
    Image, '/head_front_camera/head_front_camera/color/image_raw',
    lambda m: st.update(img=to_bgr(m), hdr=m.header), qos_profile_sensor_data)
node.create_subscription(
    Image, '/head_front_camera/head_front_camera/depth/image_rect_raw',
    lambda m: st.update(depth=depth_to_metres(m)), qos_profile_sensor_data)
node.create_subscription(
    CameraInfo, '/head_front_camera/head_front_camera/color/camera_info',
    lambda m: st.update(cam=PinholeModel(m)), qos_profile_sensor_data)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


node.create_subscription(Odometry, '/odom', lambda m: st.update(
    odom=(m.pose.pose.position.x, m.pose.pose.position.y,
          yaw_of(m.pose.pose.orientation))), 20)


def spin(seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.03)


def send(pub, joints, positions, secs=6):
    traj = JointTrajectory()
    traj.joint_names = list(joints)
    point = JointTrajectoryPoint()
    point.positions = [float(v) for v in positions]
    point.velocities = [0.0] * len(positions)
    point.time_from_start = MsgDuration(sec=secs)
    traj.points = [point]
    pub.publish(traj)


def drive(wx, wy, tol=0.03, timeout=200):
    gx, gy = arena.world_to_odom(wx, wy)
    gyaw = arena.world_yaw_to_odom(0.0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        spin(0.05)
        if 'odom' not in st:
            continue
        x, y, yaw = st['odom']
        ex, ey = gx - x, gy - y
        eyaw = arena.normalize_angle(gyaw - yaw)
        if math.hypot(ex, ey) < tol and abs(eyaw) < 0.03:
            cmd.publish(Twist())
            return True
        c, s = math.cos(-yaw), math.sin(-yaw)
        t = Twist()
        t.linear.x = max(-0.25, min(0.25, 0.9 * (c * ex - s * ey)))
        t.linear.y = max(-0.25, min(0.25, 0.9 * (s * ex + c * ey)))
        t.angular.z = max(-0.5, min(0.5, 1.5 * eyaw))
        cmd.publish(t)
    cmd.publish(Twist())
    return False


def deproject(u, v, bbox, baseline=0.015):
    cam, depth, hdr = st['cam'], st['depth'], st['hdr']
    half = patch_half_for_bbox(bbox)
    shift = cam.fx * baseline / max(1e-3, patch_depth(depth, u, v, half=half) or 1.0)
    d = patch_depth(depth, u - shift, v, half=half)
    if d is None:
        return None, None
    x, y, z = cam.deproject(u, v, d)
    p = PointStamped()
    p.header.frame_id = hdr.frame_id
    p.header.stamp = Time().to_msg()
    p.point.x, p.point.y, p.point.z = x, y, z
    try:
        op = buf.transform(p, 'odom', timeout=Duration(seconds=0.5))
    except Exception:
        return None, d
    wx, wy = arena.odom_to_world(op.point.x, op.point.y)
    return (wx, wy, op.point.z), d


spin(4.0)
truth_robot = gz_pose('tiago_pro')
odom = st['odom']
est = arena.odom_to_world(odom[0], odom[1])
print('robot: truth (%.3f, %.3f)  estimate (%.3f, %.3f)  drift (%+.3f, %+.3f)'
      % (truth_robot[0], truth_robot[1], est[0], est[1],
         est[0] - truth_robot[0], est[1] - truth_robot[1]))

target_y = arena.COLUMN_Y[COL]
print('\ndriving to the book-scan pose in front of column %d' % COL)
drive(arena.BOOK_X - arena.BOOK_SCAN_STANDOFF, target_y)
send(torso, ['torso_lift_joint'], [0.30], secs=12)
send(head, ['head_1_joint', 'head_2_joint'], [0.0, -0.26], secs=3)
spin(25.0)

truth_robot = gz_pose('tiago_pro')
odom = st['odom']
est = arena.odom_to_world(odom[0], odom[1])
drift = (est[0] - truth_robot[0], est[1] - truth_robot[1])
print('robot: truth (%.3f, %.3f)  estimate (%.3f, %.3f)  DRIFT (%+.3f, %+.3f)'
      % (truth_robot[0], truth_robot[1], est[0], est[1], drift[0], drift[1]))

books = vision.detect_books(st['img'])
truth = {}
for name in gz_models(r'book_col_%d_row_\d_\w+' % COL):
    truth[name] = gz_pose(name)

print('\n=== books detected from this pose ===')
print('%-8s %-22s %-22s %-22s %s'
      % ('colour', 'measured world', 'measured - drift', 'true CENTRE', 'depth err vs face/centre'))
for b in sorted(books, key=lambda b: b['centroid'][1]):
    world, d = deproject(b['centroid'][0], b['centroid'][1], b['bbox'])
    if world is None:
        print('%-8s no depth' % b['colour'])
        continue
    corrected = (world[0] - drift[0], world[1] - drift[1], world[2])
    # Which true book is this?  Match on height and colour.
    best, bestd = None, 9.9
    for name, pose in truth.items():
        if b['colour'] not in name:
            continue
        if abs(pose[2] - world[2]) < bestd:
            best, bestd = (name, pose), abs(pose[2] - world[2])
    if best is None:
        print('%-8s (%+.3f,%+.3f,%.3f)  no ground truth' % (b['colour'], *world))
        continue
    name, pose = best
    face = pose[0] - arena.BOOK_DEPTH / 2.0
    print('%-8s (%+.3f,%+.3f,%.3f)  (%+.3f,%+.3f,%.3f)  (%+.3f,%+.3f,%.3f)  '
          'vs face %+.3f  vs centre %+.3f'
          % (b['colour'], world[0], world[1], world[2],
             corrected[0], corrected[1], corrected[2],
             pose[0], pose[1], pose[2],
             corrected[0] - face, corrected[0] - pose[0]))

print('\nface is centre - %.3f m (BOOK_DEPTH / 2)' % (arena.BOOK_DEPTH / 2.0))
rclpy.shutdown()
