"""Interactive arm-pose tuner.

The arm poses in ``config/arm_poses.yaml`` cannot be derived offline - TIAGo
Pro has no MoveIt config in this repository, so there is no IK solver to ask.
They have to be found once, by eye, in the simulator. This tool makes that a
ten-minute job instead of an afternoon.

Run it in a second container shell while the simulation is up::

    ros2 run rafeeq_solution tune_arm

Then jog joints until the gripper is where you want it and press ``p`` to
print a YAML block to paste into ``config/arm_poses.yaml``.

Commands:
    1..7 <delta>     jog that arm joint by delta radians (e.g. ``2 -0.1``)
    s <n> <value>    set arm joint n to an absolute value
    t <metres>       set the torso lift (0.00 .. 0.35)
    h <pan> <tilt>   set the head (tilt is capped at +0.349)
    g <metres>       set the gripper finger (0.000 .. 0.069)
    load <name>      send a named pose from arm_poses.yaml
    p [name]         print the current arm pose as YAML
    r                show the live joint states
    q                quit
"""

import os
import sys
import threading

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ament_index_python.packages import get_package_share_directory

LIMITS = {
    'arm_right_1_joint': (-4.712, 0.524),
    'arm_right_2_joint': (-2.443, 1.134),
    'arm_right_3_joint': (-2.618, 2.618),
    'arm_right_4_joint': (-2.443, 1.134),
    'arm_right_5_joint': (-1.571, 3.665),
    'arm_right_6_joint': (-1.885, 3.002),
    'arm_right_7_joint': (-2.443, 2.443),
    'arm_left_1_joint': (-0.524, 4.712),
    'arm_left_2_joint': (-2.443, 1.134),
    'arm_left_3_joint': (-2.618, 2.618),
    'arm_left_4_joint': (-2.443, 1.134),
    'arm_left_5_joint': (-3.665, 1.571),
    'arm_left_6_joint': (-1.885, 3.002),
    'arm_left_7_joint': (-2.443, 2.443),
}


def _duration(seconds):
    msg = Duration()
    msg.sec = int(seconds)
    msg.nanosec = int((seconds - int(seconds)) * 1e9)
    return msg


class ArmTuner(Node):

    def __init__(self):
        super().__init__('rafeeq_tune_arm')

        self.declare_parameter('arm_side', 'right')
        self.declare_parameter('move_time', 2.0)

        self.side = str(self.get_parameter('arm_side').value).lower()
        self.move_time = float(self.get_parameter('move_time').value)

        self.joints = [
            'arm_{}_{}_joint'.format(self.side, i) for i in range(1, 8)]
        self.target = [0.0] * 7
        self.actual = {}

        self.pub_arm = self.create_publisher(
            JointTrajectory,
            '/arm_{}_controller/joint_trajectory'.format(self.side), 10)
        self.pub_torso = self.create_publisher(
            JointTrajectory, '/torso_controller/joint_trajectory', 10)
        self.pub_head = self.create_publisher(
            JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.pub_gripper = self.create_publisher(
            JointTrajectory,
            '/gripper_{}_controller/joint_trajectory'.format(self.side), 10)

        self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10)

    def on_joint_states(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.actual[name] = position

    def sync_from_actual(self):
        """Seed the target pose from wherever the arm currently is."""
        for i, joint in enumerate(self.joints):
            if joint in self.actual:
                self.target[i] = self.actual[joint]

    def send(self, publisher, joints, positions, seconds=None):
        traj = JointTrajectory()
        traj.joint_names = list(joints)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = _duration(
            self.move_time if seconds is None else seconds)
        traj.points = [point]
        publisher.publish(traj)

    def send_arm(self):
        for i, joint in enumerate(self.joints):
            lo, hi = LIMITS.get(joint, (-3.14, 3.14))
            self.target[i] = max(lo, min(hi, self.target[i]))
        self.send(self.pub_arm, self.joints, self.target)

    def load_pose(self, name):
        path = os.path.join(
            get_package_share_directory('rafeeq_solution'),
            'config', 'arm_poses.yaml')
        with open(path, 'r') as handle:
            config = yaml.safe_load(handle) or {}

        poses = config.get('arm_poses', {})
        if name not in poses:
            print('no such pose: {} (have {})'.format(
                name, ', '.join(sorted(poses))))
            return

        self.target = [float(v) for v in poses[name]]
        self.send_arm()
        print('sent {}'.format(name))

    def print_yaml(self, name='tuned_pose'):
        values = ', '.join('{:+.2f}'.format(v) for v in self.target)
        print('\n  {}: [{}]\n'.format(name, values))

    def report(self):
        print('\n  target -> actual')
        for i, joint in enumerate(self.joints):
            actual = self.actual.get(joint)
            actual_text = '{:+.3f}'.format(actual) if actual is not None \
                else '   ?  '
            lo, hi = LIMITS.get(joint, (0.0, 0.0))
            print('  {} {}  {:+.3f} -> {}   [{:+.2f}, {:+.2f}]'.format(
                i + 1, joint, self.target[i], actual_text, lo, hi))

        for extra in ('torso_lift_joint',
                      'head_1_joint', 'head_2_joint',
                      'gripper_{}_finger_joint'.format(self.side)):
            if extra in self.actual:
                print('    {:<28} {:+.3f}'.format(extra, self.actual[extra]))
        print()


def repl(node):
    print(__doc__)
    node.sync_from_actual()
    node.report()

    for line in sys.stdin:
        parts = line.split()
        if not parts:
            continue

        head = parts[0].lower()

        try:
            if head in ('q', 'quit', 'exit'):
                break

            elif head == 'p':
                node.print_yaml(parts[1] if len(parts) > 1 else 'tuned_pose')

            elif head == 'r':
                node.sync_from_actual()
                node.report()

            elif head == 'load' and len(parts) > 1:
                node.load_pose(parts[1])

            elif head == 's' and len(parts) > 2:
                node.target[int(parts[1]) - 1] = float(parts[2])
                node.send_arm()
                node.report()

            elif head == 't' and len(parts) > 1:
                value = max(0.0, min(0.35, float(parts[1])))
                node.send(node.pub_torso, ['torso_lift_joint'], [value], 3.0)
                print('torso -> {:.3f}'.format(value))

            elif head == 'h' and len(parts) > 2:
                pan = max(-1.309, min(1.309, float(parts[1])))
                tilt = max(-1.047, min(0.349, float(parts[2])))
                node.send(node.pub_head, ['head_1_joint', 'head_2_joint'],
                          [pan, tilt], 1.5)
                print('head -> pan {:+.3f} tilt {:+.3f}'.format(pan, tilt))

            elif head == 'g' and len(parts) > 1:
                value = max(0.0, min(0.069, float(parts[1])))
                node.send(node.pub_gripper,
                          ['gripper_{}_finger_joint'.format(node.side)],
                          [value], 1.5)
                print('gripper -> {:.3f}'.format(value))

            elif head.isdigit() and len(parts) > 1:
                node.target[int(head) - 1] += float(parts[1])
                node.send_arm()
                node.report()

            else:
                print('unrecognised: {}'.format(line.strip()))

        except (ValueError, IndexError) as exc:
            print('bad input ({}): {}'.format(exc, line.strip()))


def main(args=None):
    rclpy.init(args=args)
    node = ArmTuner()

    spinner = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    try:
        repl(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
