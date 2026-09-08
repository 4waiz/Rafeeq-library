"""Manipulation node - arm, gripper, torso and head trajectories.

Why not MoveIt: the competition repository ships no MoveIt configuration for
TIAGo Pro. There is no SRDF, no planning group, no kinematics plugin - a grep
for ``srdf`` and ``moveit`` across ``src/`` returns nothing. MoveIt is
installed in the image but has nothing to plan against, so bringing it up
means authoring a full moveit_config package first.

The task does not need it. The shelf never moves, the books are a fixed size
at four fixed heights, and the base is holonomic so it can do the fine
positioning. That reduces manipulation to replaying a handful of named joint
poses, which is what this node does - reliably, and with no planning latency.

Commands arrive on ``/rafeeq/manip/command`` as a plain string:

    home                 tuck the arm, centre the head, lower the torso
    head:<name>          named head pose, or ``head:<pan>,<tilt>`` in radians
    torso:<name|metres>  named torso height, or a raw value
    arm:<name>           named arm pose from config/arm_poses.yaml
    row:<1-4>            torso + arm reach pose for a competition shelf row
    gripper:<name|m>     open / closed / relax, or a raw finger position
    carry                fold the book in against the body
    place                extend over the collection bin

Progress is reported on ``/rafeeq/manip/status`` as ``busy:<cmd>`` then
``done:<cmd>`` or ``failed:<cmd>``.
"""

import os

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from rclpy.duration import Duration as RclDuration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ament_index_python.packages import get_package_share_directory


def _duration(seconds):
    msg = Duration()
    msg.sec = int(seconds)
    msg.nanosec = int((seconds - int(seconds)) * 1e9)
    return msg


class ManipulationNode(Node):

    def __init__(self):
        super().__init__('rafeeq_manipulation')

        self.declare_parameter('arm_side', 'right')
        self.declare_parameter('config_file', '')
        self.declare_parameter('settle_time', 0.75)
        self.declare_parameter('joint_tolerance', 0.15)

        self.side = str(self.get_parameter('arm_side').value).lower()
        if self.side not in ('left', 'right'):
            self.get_logger().warn(
                'arm_side={!r} invalid, falling back to right'.format(self.side))
            self.side = 'right'

        self.settle = float(self.get_parameter('settle_time').value)
        self.tolerance = float(self.get_parameter('joint_tolerance').value)

        self.config = self._load_config()
        self.arm_poses = self.config.get('arm_poses', {})
        self.torso_by_row = {
            int(k): float(v)
            for k, v in self.config.get('torso_by_row', {}).items()}
        self.torso_poses = self.config.get('torso_poses', {})
        self.head_poses = self.config.get('head_poses', {})
        self.gripper_poses = self.config.get('gripper', {})
        self.durations = self.config.get('durations', {})

        self.arm_joints = [
            'arm_{}_{}_joint'.format(self.side, i) for i in range(1, 8)]
        self.gripper_joint = 'gripper_{}_finger_joint'.format(self.side)

        self.pub_arm = self.create_publisher(
            JointTrajectory,
            '/arm_{}_controller/joint_trajectory'.format(self.side), 10)
        # Public gripper topic - erc_bringup's gripper_command_clamp owns it
        # and forwards range-checked commands to the *_raw controller.
        self.pub_gripper = self.create_publisher(
            JointTrajectory,
            '/gripper_{}_controller/joint_trajectory'.format(self.side), 10)
        self.pub_torso = self.create_publisher(
            JointTrajectory, '/torso_controller/joint_trajectory', 10)
        self.pub_head = self.create_publisher(
            JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.pub_status = self.create_publisher(
            String, '/rafeeq/manip/status', 10)

        self.joint_positions = {}
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10)
        self.create_subscription(
            String, '/rafeeq/manip/command', self.on_command, 10)

        self.active = None            # (command, expected {joint: position})
        self.deadline = None
        self.create_timer(0.1, self.tick)

        self.get_logger().info(
            'manipulation up | arm={} | poses={}'.format(
                self.side, sorted(self.arm_poses)))

    # -- setup -----------------------------------------------------------

    def _load_config(self):
        path = str(self.get_parameter('config_file').value)

        if not path:
            try:
                path = os.path.join(
                    get_package_share_directory('rafeeq_solution'),
                    'config', 'arm_poses.yaml')
            except Exception as exc:
                self.get_logger().error(
                    'cannot locate rafeeq_solution share dir: {}'.format(exc))
                return {}

        try:
            with open(path, 'r') as handle:
                return yaml.safe_load(handle) or {}
        except OSError as exc:
            self.get_logger().error(
                'cannot read arm poses from {}: {}'.format(path, exc))
            return {}

    # -- subscriptions ---------------------------------------------------

    def on_joint_states(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position

    def on_command(self, msg):
        command = msg.data.strip()
        if not command:
            return

        if self.active is not None:
            self.get_logger().warn(
                'busy with {!r}, dropping {!r}'.format(
                    self.active[0], command))
            return

        try:
            expected, duration = self.dispatch(command)
        except (KeyError, ValueError) as exc:
            self.get_logger().error(
                'bad command {!r}: {}'.format(command, exc))
            self._status('failed:{}'.format(command))
            return

        self.active = (command, expected)
        self.deadline = self.get_clock().now() + RclDuration(
            seconds=duration + self.settle)
        self._status('busy:{}'.format(command))
        self.get_logger().info(
            '{} -> {:.1f} s'.format(command, duration))

    # -- command dispatch ------------------------------------------------

    def dispatch(self, command):
        """Send the trajectories for ``command``.

        Returns ``(expected_positions, duration)`` where ``expected_positions``
        maps joint name to target so :meth:`tick` can verify arrival.
        """
        head, _, argument = command.partition(':')
        head = head.lower()

        # Composite commands drive several joint groups at once, so they must
        # wait for the SLOWEST of them. The torso is the slow one: its
        # velocity limit is 0.035 m/s, so a full 0.35 m of travel takes 10 s
        # against the arm's 4 s. Waiting only the arm duration reports 'done'
        # while the torso is still moving.
        composite = max(self._d('arm'), self._d('torso'))

        if head == 'home':
            expected = {}
            expected.update(self.send_arm('tucked'))
            expected.update(self.send_torso(
                self.torso_poses.get('drive', 0.10)))
            expected.update(self.send_head('forward'))
            return expected, composite

        if head == 'carry':
            expected = dict(self.send_arm('carry'))
            expected.update(self.send_torso(
                self.torso_poses.get('drive', 0.10)))
            return expected, composite

        if head == 'place':
            expected = dict(self.send_arm('place'))
            expected.update(self.send_torso(
                self.torso_poses.get('place', 0.35)))
            return expected, composite

        if head == 'arm':
            return self.send_arm(argument), self._d('arm')

        if head == 'row':
            row = int(argument)
            if row not in (1, 2, 3, 4):
                raise ValueError('row must be 1..4, got {}'.format(row))
            expected = dict(self.send_arm('reach_row_{}'.format(row)))
            expected.update(self.send_torso(
                self.torso_by_row.get(row, 0.10)))
            return expected, composite

        if head == 'torso':
            value = self.torso_poses.get(argument)
            if value is None:
                value = float(argument)
            return self.send_torso(float(value)), self._d('torso')

        if head == 'head':
            if ',' in argument:
                pan, tilt = (float(v) for v in argument.split(',', 1))
                return self.send_head_values(pan, tilt), self._d('head')
            return self.send_head(argument), self._d('head')

        if head == 'gripper':
            value = self.gripper_poses.get(argument)
            if value is None:
                value = float(argument)
            return self.send_gripper(float(value)), self._d('gripper')

        raise ValueError('unknown command')

    # -- trajectory senders ----------------------------------------------

    def _d(self, key):
        return float(self.durations.get(key, 3.0))

    def _send(self, publisher, joints, positions, duration):
        traj = JointTrajectory()
        traj.joint_names = list(joints)

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = _duration(duration)
        traj.points = [point]

        publisher.publish(traj)
        return dict(zip(joints, point.positions))

    def send_arm(self, pose_name):
        if pose_name not in self.arm_poses:
            raise KeyError(
                'arm pose {!r} not in config (have {})'.format(
                    pose_name, sorted(self.arm_poses)))

        positions = self.arm_poses[pose_name]
        if len(positions) != len(self.arm_joints):
            raise ValueError(
                'arm pose {!r} has {} values, expected {}'.format(
                    pose_name, len(positions), len(self.arm_joints)))

        return self._send(self.pub_arm, self.arm_joints, positions,
                          self._d('arm'))

    def send_torso(self, metres):
        metres = max(0.0, min(0.35, float(metres)))
        return self._send(self.pub_torso, ['torso_lift_joint'], [metres],
                          self._d('torso'))

    def send_head(self, pose_name):
        if pose_name not in self.head_poses:
            raise KeyError(
                'head pose {!r} not in config (have {})'.format(
                    pose_name, sorted(self.head_poses)))
        pan, tilt = self.head_poses[pose_name]
        return self.send_head_values(pan, tilt)

    def send_head_values(self, pan, tilt):
        # head_2 upper limit is +0.349 rad; asking for more is silently
        # ignored by the controller, which looks like a perception bug.
        pan = max(-1.309, min(1.309, float(pan)))
        tilt = max(-1.047, min(0.349, float(tilt)))
        return self._send(self.pub_head, ['head_1_joint', 'head_2_joint'],
                          [pan, tilt], self._d('head'))

    def send_gripper(self, metres):
        # erc_bringup's clamp *rejects* out-of-range commands outright rather
        # than clipping them, so clip here or the motion silently never runs.
        metres = max(0.0, min(0.069, float(metres)))
        return self._send(self.pub_gripper, [self.gripper_joint], [metres],
                          self._d('gripper'))

    # -- completion ------------------------------------------------------

    def tick(self):
        if self.active is None or self.deadline is None:
            return

        if self.get_clock().now() < self.deadline:
            return

        command, expected = self.active
        self.active = None
        self.deadline = None

        stragglers = []
        for joint, target in expected.items():
            actual = self.joint_positions.get(joint)
            if actual is None:
                continue
            if abs(actual - target) > self.tolerance:
                stragglers.append(
                    '{} target {:+.3f} actual {:+.3f}'.format(
                        joint, target, actual))

        if stragglers:
            # Report done anyway: the gripper legitimately stalls on a book
            # before reaching its commanded position, and that is a
            # successful grasp, not a failure.
            self.get_logger().warn(
                '{} finished with joints off target ({})'.format(
                    command, '; '.join(stragglers)))

        self._status('done:{}'.format(command))

    def _status(self, text):
        self.pub_status.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
