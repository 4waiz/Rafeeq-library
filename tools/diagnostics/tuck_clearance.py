"""Which arm link comes closest to the table while the base turns to face the
shelf, and by how much?

The robot spawns at world (0, 0) yawed +90 deg and rotates to yaw 0 in place.
Every link sits at a fixed position in the base frame, so the sweep is just a
rotation of those points about the base origin.  This walks the rotation and
reports, per link, the worst penetration of the table's box.

Reads the live TF tree for the link positions, so it measures the robot the
simulator is actually running rather than a model of it.
"""
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as MsgDuration

sys.path.insert(0, '/opt/erc_ws/src/rafeeq_solution')
from rafeeq_solution import arena

# The pose under test: default is whatever arm_poses.yaml currently holds.
POSE = os.environ.get('POSE', '')
TORSO = float(os.environ.get('TORSO', '0.30'))

LEFT_JOINTS = ['arm_left_{}_joint'.format(i) for i in range(1, 8)]
RIGHT_JOINTS = ['arm_right_{}_joint'.format(i) for i in range(1, 8)]

#: Every left-side link that could reach anything, plus the gripper.
LINKS = (['arm_left_{}_link'.format(i) for i in range(1, 8)]
         + ['gripper_left_base_link', 'gripper_left_left_finger_link',
            'gripper_left_right_finger_link'])


def load_yaml_pose(name):
    import yaml
    path = '/opt/erc_ws/src/rafeeq_solution/config/arm_poses.yaml'
    with open(path) as handle:
        cfg = yaml.safe_load(handle)
    for section in cfg.values():
        if isinstance(section, dict) and name in section:
            value = section[name]
            if isinstance(value, list):
                return [float(v) for v in value]
    raise KeyError(name)


rclpy.init()
node = Node('tuckdiag')
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, node)
arm_left = node.create_publisher(
    JointTrajectory, '/arm_left_controller/joint_trajectory', 10)
arm_right = node.create_publisher(
    JointTrajectory, '/arm_right_controller/joint_trajectory', 10)
torso = node.create_publisher(
    JointTrajectory, '/torso_controller/joint_trajectory', 10)


def spin(seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.03)


def send(pub, joints, positions, secs=8):
    traj = JointTrajectory()
    traj.joint_names = list(joints)
    point = JointTrajectoryPoint()
    point.positions = [float(v) for v in positions]
    point.velocities = [0.0] * len(positions)
    point.time_from_start = MsgDuration(sec=secs)
    traj.points = [point]
    pub.publish(traj)


spin(3.0)

left = ([float(v) for v in POSE.split(',')] if POSE
        else load_yaml_pose('tucked_left'))
right = load_yaml_pose('tucked')
print('torso %.2f' % TORSO)
print('left  pose %s' % ['%+.3f' % v for v in left])
print('right pose %s' % ['%+.3f' % v for v in right])

send(torso, ['torso_lift_joint'], [TORSO], secs=14)
send(arm_left, LEFT_JOINTS, left, secs=10)
send(arm_right, RIGHT_JOINTS, right, secs=10)
spin(30.0)

# Link positions in the base frame.  Fixed for the whole sweep.
base_points = {}
for link in LINKS:
    try:
        tf = buf.lookup_transform('base_link', link, rclpy.time.Time(),
                                  timeout=Duration(seconds=1.0))
    except Exception as exc:
        print('  %-32s NO TF (%s)' % (link, str(exc)[:40]))
        continue
    t = tf.transform.translation
    base_points[link] = (t.x, t.y, t.z)

print('\n=== link positions in base_link frame ===')
for link, (x, y, z) in base_points.items():
    print('  %-32s (%+.3f, %+.3f, %+.3f)  world z %.3f'
          % (link, x, y, z, z + arena.BASE_LINK_Z))

# The sweep: base at world (0, 0), yaw from +90 deg to 0.
print('\n=== worst approach to the table during the FACE_SHELF turn ===')
print('    table x %.3f..%.3f  y %.3f..%.3f  top z %.3f'
      % (arena.TABLE_X[0], arena.TABLE_X[1],
         arena.TABLE_Y[0], arena.TABLE_Y[1], arena.TABLE_TOP_Z))

worst = []
for link, (bx, by, bz) in base_points.items():
    wz = bz + arena.BASE_LINK_Z
    best_gap = 9.9
    best_yaw = None
    for step in range(0, 91):
        yaw = math.radians(90 - step)
        c, s = math.cos(yaw), math.sin(yaw)
        wx = arena.START_XY[0] + bx * c - by * s
        wy = arena.START_XY[1] + bx * s + by * c
        # Signed clearance: how far outside the table box the point is.
        dx = max(arena.TABLE_X[0] - wx, wx - arena.TABLE_X[1])
        dy = max(arena.TABLE_Y[0] - wy, wy - arena.TABLE_Y[1])
        dz = wz - arena.TABLE_TOP_Z
        gap = max(dx, dy, dz)
        if gap < best_gap:
            best_gap, best_yaw = gap, math.degrees(yaw)
    worst.append((best_gap, link, best_yaw, wz))

for gap, link, yaw, wz in sorted(worst):
    flag = 'HITS' if gap <= 0 else 'clear'
    print('  %-32s %s by %+.3f m  at yaw %5.1f deg  (world z %.3f)'
          % (link, flag, gap, yaw, wz))

rclpy.shutdown()
