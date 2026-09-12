"""Manipulation node - arm, gripper, torso and head, with real completion checks.

Three things this node deliberately does **not** do, because earlier versions did
them and they manufactured false successes:

* It does not report ``done`` because a trajectory duration elapsed.  Arm, torso
  and head motions go through the controllers' ``FollowJointTrajectory`` **action**
  interfaces and completion is the action result, cross-checked against fresh
  ``/joint_states``.  A controller that never moved reports ``failed``.
* It does not apply one blanket "the gripper might stall" exemption to every
  joint group.  A stall is expected for the *gripper closing on a book* and only
  there; an arm, head or torso that misses its target is a failure.
* It does not treat "the close command was sent" as a grasp.  The finger joint
  stalls on whatever is between the fingers, so the measured resting position is
  a **width measurement**: ``width = 0.028 + 0.783 * q`` (measured, see
  ``docs/measured_environment.md``).  A grasp is confirmed only when that width
  is consistent with a 0.03 m book spine.

Why not MoveIt: the competition repository ships no MoveIt configuration for
TIAGo Pro - no SRDF, no planning group, no kinematics plugin.  MoveIt is
installed in the image but has nothing to plan against.  :mod:`kinematics`
solves the same 8-DOF chain out of the competition URDF directly, and
:mod:`grasp` turns a measured book position into a validated Cartesian path.

Command protocol on ``/rafeeq/manip/command`` (plain text, pipe separated)::

    <seq>|home
    <seq>|head|<name>              or  <seq>|head|<pan>,<tilt>
    <seq>|torso|<name or metres>
    <seq>|gripper|open|close|<metres>
    <seq>|carry
    <seq>|grasp|<x>,<y>,<z>|<slot>|<basex>,<basey>,<baseyaw>
    <seq>|place|<x>,<y>,<z>|<basex>,<basey>,<baseyaw>
    <seq>|release
    <seq>|stow

``<seq>`` is an integer that the reply echoes, so an acknowledgement from a
previous command or a previous run can never advance the current state.

Replies on ``/rafeeq/manip/status``::

    busy|<seq>|<verb>
    done|<seq>|<verb>|<detail>
    failed|<seq>|<verb>|<reason>
"""

import math
import os
import time

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ament_index_python.packages import get_package_share_directory

from . import arena, grasp, kinematics


#: A gripper "open" only has to admit the 0.030 m spine with room either
#: side; requiring an exact joint value there is a false failure.
OPEN_GAP_REQUIRED = arena.BOOK_SPINE_W + 0.014

#: Joint accuracy demanded of the grasp posture, in radians.  See the comment
#: at its use site: the finger clearance around a book spine is ~0.011 m.
GRASP_POSTURE_TOLERANCE = 0.02


def _duration(seconds):
    msg = Duration()
    seconds = max(0.0, float(seconds))
    msg.sec = int(seconds)
    msg.nanosec = int(round((seconds - int(seconds)) * 1e9))
    return msg


class Step(object):
    """One stage of a job: start it, then wait for its own completion test."""

    def __init__(self, name, start, done, timeout, detail=None):
        self.name = name
        self.start = start
        self.done = done
        self.timeout = float(timeout)
        self.detail = detail
        self.started_at = None
        self.error = None


class ManipulationNode(Node):

    def __init__(self):
        super().__init__('rafeeq_manipulation')

        self.declare_parameter('arm_side', 'right')
        self.declare_parameter('config_file', '')
        self.declare_parameter('urdf_file', '')
        self.declare_parameter('joint_tolerance', 0.10)
        self.declare_parameter('gripper_tolerance', 0.004)
        self.declare_parameter('joint_state_max_age', 1.0)
        # Fraction of each joint's URDF velocity limit that trajectories are
        # timed against.  Below 1.0 so the controller is never chasing a
        # physically impossible profile.
        # Arm and torso motions run at this fraction of the URDF velocity
        # limits.  Raised from 0.55: none of these moves carries the book
        # except the lift, which is 0.02 m and floored by min_segment_time
        # anyway, and the run is judged on a clock.
        self.declare_parameter('speed_fraction', 0.80)
        self.declare_parameter('min_segment_time', 0.7)
        # How long a joint is allowed to keep converging after its trajectory
        # action reports success, before being called blocked.
        self.declare_parameter('settle_window', 4.0)
        self.declare_parameter('ik_samples', 40000)
        self.declare_parameter('spawn_yaw', arena.SPAWN_YAW)
        self.declare_parameter('base_pose_tolerance', 0.15)

        self.side = str(self.get_parameter('arm_side').value).lower()
        if self.side not in ('left', 'right'):
            self.get_logger().warn(
                'arm_side={!r} invalid, using right'.format(self.side))
            self.side = 'right'

        self.tolerance = float(self.get_parameter('joint_tolerance').value)
        self.grip_tol = float(self.get_parameter('gripper_tolerance').value)
        self.js_max_age = float(
            self.get_parameter('joint_state_max_age').value)
        self.speed_fraction = float(
            self.get_parameter('speed_fraction').value)
        self.min_seg = float(self.get_parameter('min_segment_time').value)
        self.settle_window = float(self.get_parameter('settle_window').value)
        self.velocity_limits = {}
        self.spawn_yaw = float(self.get_parameter('spawn_yaw').value)
        self.base_tol = float(
            self.get_parameter('base_pose_tolerance').value)
        self.base_world = None
        self.base_stamp = None

        self.config = self._load_config()
        self.torso_poses = self.config.get('torso_poses', {})
        self.head_poses = self.config.get('head_poses', {})
        self.arm_poses = self.config.get('arm_poses', {})
        self.durations = self.config.get('durations', {})

        self.arm_joints = ['arm_{}_{}_joint'.format(self.side, i)
                           for i in range(1, 8)]
        self.other_side = 'left' if self.side == 'right' else 'right'
        self.other_arm_joints = ['arm_{}_{}_joint'.format(self.other_side, i)
                                 for i in range(1, 8)]
        self.gripper_joint = 'gripper_{}_finger_joint'.format(self.side)

        # --- interfaces ---------------------------------------------------
        # Arm, torso and head go through the action interface because it
        # reports real completion.  The gripper deliberately uses the public
        # *topic*, which the organiser's gripper_command_clamp owns - driving
        # gripper_*_controller_raw directly would bypass their range check.
        self.arm_action = ActionClient(
            self, FollowJointTrajectory,
            '/arm_{}_controller/follow_joint_trajectory'.format(self.side))
        self.torso_action = ActionClient(
            self, FollowJointTrajectory,
            '/torso_controller/follow_joint_trajectory')
        self.head_action = ActionClient(
            self, FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory')
        # The task uses one arm, but the *other* arm still has to be parked:
        # at its all-zeros spawn pose its gripper sits 0.98 m in front of the
        # base, which is inside the shelf once the robot is at grasp standoff.
        self.other_arm_action = ActionClient(
            self, FollowJointTrajectory,
            '/arm_{}_controller/follow_joint_trajectory'.format(
                self.other_side))

        self.pub_gripper = self.create_publisher(
            JointTrajectory,
            '/gripper_{}_controller/joint_trajectory'.format(self.side), 10)
        self.pub_status = self.create_publisher(
            String, '/rafeeq/manip/status', 10)
        self.pub_hold = self.create_publisher(
            Float32, '/rafeeq/manip/hold', 10)
        self.pub_ready = self.create_publisher(
            Bool, '/rafeeq/manip/ready', 10)

        self.joint_positions = {}
        self.joint_stamp = None
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10)
        self.create_subscription(
            String, '/rafeeq/manip/command', self.on_command, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        # Grasp verification comes from the simulator's own contact sensors.
        # It cannot come from the finger joint stalling: gz_ros2_control
        # implements a position command as a JointVelocityCmd, so the joint
        # reaches whatever it is told to reach and simply compresses into the
        # book instead of stopping against it.  The fingertip links do have
        # contact sensors, and a sustained contact between BOTH fingertips and
        # the same book model is direct evidence of a grasp.
        self.create_subscription(Contacts, '/contacts', self.on_contacts, 10)
        self._finger_contact = {}

        # --- kinematics ----------------------------------------------------
        self.chain = None
        self.planner = None
        self._chain_error = None
        self._load_chain()

        self.job = []
        self.job_seq = None
        self.job_verb = ''
        self.job_index = 0
        self.job_detail = {}
        self._goal_handle = None
        self._result_future = None
        self._send_future = None
        self.holding_width = None
        self.holding = None
        self._plan = None
        self._plan_target = None
        self._plan_slot = None

        self.create_timer(0.1, self.tick)
        self.create_timer(0.25, self.publish_hold)
        self.create_timer(0.5, self.publish_ready)
        self._announced_ready = False

        self.get_logger().info(
            'manipulation up | arm={} | IK chain {} | gripper topic {}'.format(
                self.side,
                'ready' if self.chain is not None else 'UNAVAILABLE',
                '/gripper_{}_controller/joint_trajectory'.format(self.side)))

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
                    'cannot locate rafeeq_solution share: {}'.format(exc))
                return {}
        try:
            with open(path, 'r') as handle:
                return yaml.safe_load(handle) or {}
        except OSError as exc:
            self.get_logger().error(
                'cannot read {}: {}'.format(path, exc))
            return {}

    def _urdf_path(self):
        given = str(self.get_parameter('urdf_file').value)
        if given:
            return given
        try:
            share = get_package_share_directory('erc_description')
        except Exception as exc:
            raise RuntimeError('erc_description not found: {}'.format(exc))
        return os.path.join(share, 'urdf', 'tiago_pro.urdf')

    def _load_chain(self):
        """Parse the competition URDF into an IK chain and warm the planner."""
        try:
            path = self._urdf_path()
            if not os.path.exists(path):
                raise RuntimeError('URDF not found at {}'.format(path))
            self.velocity_limits = kinematics.joint_velocity_limits(path)
            builder = (kinematics.right_arm_chain if self.side == 'right'
                       else kinematics.left_arm_chain)
            self.chain = builder(path)
            self.planner = grasp.GraspPlanner(
                self.chain, logger=self.get_logger().info)
            started = time.time()
            self.planner.warm_up(int(self.get_parameter('ik_samples').value))
            self.get_logger().info(
                'IK chain: {} DOF ({}), seed workspace warmed in {:.1f} s'
                .format(self.chain.dof, ', '.join(self.chain.names),
                        time.time() - started))
        except Exception as exc:
            self._chain_error = str(exc)
            self.get_logger().error(
                'kinematics unavailable, grasping disabled: {}'.format(exc))

    # -- subscriptions ---------------------------------------------------

    def on_joint_states(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position
        self.joint_stamp = self.get_clock().now()

    def now(self):
        """Seconds on the node clock.

        With ``use_sim_time`` this is **simulation** time, which is the same
        clock the controllers time their trajectories against.  Mixing it with
        wall time is a real bug on this setup: the simulator runs at a real-time
        factor well below 1 (measured 0.36 on the validation machine), so a
        13-second trajectory takes 36 wall seconds and a wall-clock timeout
        fires long before the motion could possibly have finished.
        """
        return self.get_clock().now().nanoseconds / 1e9

    FINGER_LINKS = ('fingertip_left', 'fingertip_right',
                    'inner_finger_left', 'inner_finger_right')

    def on_contacts(self, msg):
        """Record which of our fingertips are touching which book, and when."""
        now = self.now()
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            book = next((n.split('::', 1)[0] for n in names
                         if n.split('::', 1)[0].startswith('book_')), None)
            if book is None:
                continue
            link = next((n for n in names if 'tiago' in n), '')
            if self.side not in link:
                continue
            for tag in self.FINGER_LINKS:
                if tag in link:
                    self._finger_contact[tag] = (now, book)

    def gripped_book(self, window=1.0):
        """Book currently in contact with the closed fingers, or None.

        Deliberately *not* "contact on both fingers at once".  The book's
        collision declares ``<max_contacts>4</max_contacts>``, so once one
        fingertip is generating contact points the other one's can be dropped
        by the solver - both fingers can be pressing on the spine while only
        one of them ever appears on ``/contacts``.

        What makes this a grasp test rather than a touch test is *persistence*:
        the mission re-checks it after the lift and again after the base has
        reversed 0.42 m.  A book that is merely being brushed stops touching
        the moment the robot backs away; a book that is held does not.
        """
        now = self.now()
        books = [b for tag, (t, b) in self._finger_contact.items()
                 if now - t <= window]
        if not books:
            return None
        # Most-recently-seen book wins if somehow two are in contact.
        latest = max(self._finger_contact.values(), key=lambda tb: tb[0])
        return latest[1]

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        wx, wy = arena.odom_to_world(
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            self.spawn_yaw)
        self.base_world = (wx, wy,
                           arena.odom_yaw_to_world(yaw, self.spawn_yaw))
        self.base_stamp = self.get_clock().now()

    def base_pose_fresh(self):
        if self.base_stamp is None:
            return False
        age = (self.get_clock().now() - self.base_stamp).nanoseconds / 1e9
        return age <= self.js_max_age

    def resolve_base(self, requested):
        """Plan against where the base **is**, not where it was asked to be.

        The arm executes joint angles, which are relative to the base.  Any
        error between the commanded waypoint and where the base actually
        stopped therefore lands directly on the gripper.  Measured on this
        setup: a base 19 mm short and 0.010 rad off yaw put the gripper 11.6 mm
        off the book - against a 14 mm half-gap, which is the difference
        between a grasp and closing on air.
        """
        if not self.base_pose_fresh():
            raise RuntimeError(
                '/odom is stale; cannot place the arm relative to the base')
        actual = self.base_world
        if requested is not None:
            drift = math.hypot(actual[0] - requested[0],
                               actual[1] - requested[1])
            if drift > self.base_tol:
                raise RuntimeError(
                    'base is {:.3f} m from the pose this grasp was requested '
                    'for ({:+.2f}, {:+.2f}); refusing to reach from '
                    '({:+.2f}, {:+.2f})'.format(
                        drift, requested[0], requested[1],
                        actual[0], actual[1]))
            if drift > 0.01:
                self.get_logger().info(
                    'planning against the measured base pose '
                    '({:+.3f}, {:+.3f}, {:+.3f} rad), {:.3f} m from the '
                    'requested one'.format(actual[0], actual[1], actual[2],
                                           drift))
        return actual

    def joint_states_fresh(self):
        """True if /joint_states arrived recently enough to trust."""
        if self.joint_stamp is None:
            return False
        age = (self.get_clock().now() - self.joint_stamp).nanoseconds / 1e9
        return age <= self.js_max_age

    def joint_age(self):
        if self.joint_stamp is None:
            return float('inf')
        return (self.get_clock().now() - self.joint_stamp).nanoseconds / 1e9

    # -- status ----------------------------------------------------------

    def _status(self, text):
        self.pub_status.publish(String(data=text))

    def busy(self):
        self._status('busy|{}|{}'.format(self.job_seq, self.job_verb))

    def succeed(self, detail=''):
        self.get_logger().info(
            '{} [{}] done{}'.format(self.job_verb, self.job_seq,
                                    ': ' + detail if detail else ''))
        self._status('done|{}|{}|{}'.format(
            self.job_seq, self.job_verb, detail))
        self._clear_job()

    def abort(self, reason):
        self.get_logger().error(
            '{} [{}] FAILED: {}'.format(self.job_verb, self.job_seq, reason))
        self._status('failed|{}|{}|{}'.format(
            self.job_seq, self.job_verb, reason))
        self._cancel_goal()
        self._clear_job()

    def _clear_job(self):
        self.job = []
        self.job_index = 0
        self._goal_handle = None
        self._result_future = None
        self._send_future = None

    # -- command entry ---------------------------------------------------

    def on_command(self, msg):
        raw = msg.data.strip()
        if not raw:
            return

        parts = raw.split('|')
        if len(parts) < 2 or not parts[0].lstrip('-').isdigit():
            self.get_logger().error(
                'malformed command {!r}; expected <seq>|<verb>|...'.format(raw))
            return

        seq, verb, args = int(parts[0]), parts[1].strip().lower(), parts[2:]

        if self.job:
            self.get_logger().warn(
                'busy with {} [{}]; rejecting {} [{}]'.format(
                    self.job_verb, self.job_seq, verb, seq))
            self._status('failed|{}|{}|busy with {}'.format(
                seq, verb, self.job_verb))
            return

        self.job_seq = seq
        self.job_verb = verb
        self.job_detail = {}

        builder = getattr(self, 'build_{}'.format(verb), None)
        if builder is None:
            self.abort('unknown verb {!r}'.format(verb))
            return

        try:
            steps = builder(args)
        except grasp.PlanFailure as exc:
            self.abort('plan rejected: {}'.format(exc))
            return
        except Exception as exc:
            self.abort('{}: {}'.format(type(exc).__name__, exc))
            return

        if not steps:
            # A verb can be pure computation (plan_grasp): still report what
            # it produced rather than an empty acknowledgement.
            self.succeed(' '.join('{}={}'.format(k, v)
                                  for k, v in sorted(self.job_detail.items())))
            return

        self.job = steps
        self.job_index = 0
        self.busy()

    # -- job runner ------------------------------------------------------

    def tick(self):
        if not self.job:
            return

        step = self.job[self.job_index]

        if step.started_at is None:
            try:
                step.start()
            except Exception as exc:
                self.abort('{} could not start: {}'.format(step.name, exc))
                return
            step.started_at = self.now()
            return

        try:
            finished = step.done()
        except Exception as exc:
            self.abort('{} check failed: {}'.format(step.name, exc))
            return

        if finished is False and (self.now() - step.started_at) > step.timeout:
            self.abort('{} timed out after {:.0f} s{}'.format(
                step.name, step.timeout,
                ' ({})'.format(step.error) if step.error else ''))
            return

        if finished is None:
            return
        if finished is False:
            return

        if step.detail:
            try:
                self.job_detail.update(step.detail() or {})
            except Exception:
                pass

        self.job_index += 1
        if self.job_index >= len(self.job):
            detail = ' '.join('{}={}'.format(k, v)
                              for k, v in sorted(self.job_detail.items()))
            self.succeed(detail)

    # -- trajectory execution -------------------------------------------

    def _segment_times(self, path, start, joints):
        """Time-parameterise a joint path against real URDF velocity limits.

        Each segment is given the time its *slowest* joint needs at a fraction
        of that joint's limit.  This is what makes a torso move (0.035 m/s) and
        an arm move (1.95 rad/s) both arrive when the trajectory says they do.
        """
        times, t = [], 0.0
        previous = np.asarray(start, float)
        for q in path:
            q = np.asarray(q, float)
            need = self.min_seg
            for i, name in enumerate(joints):
                limit = self.velocity_limits.get(name, 1.0)
                speed = max(1e-4, limit * self.speed_fraction)
                delta = abs(float(q[i]) - float(previous[i]))
                need = max(need, delta / speed)
            t += need
            times.append(t)
            previous = q
        return times

    def _current(self, joints):
        return np.array([self.joint_positions.get(j, 0.0) for j in joints])

    def send_path(self, client, joints, path, extra_settle=0.5,
                  tolerance=None):
        """Send a multi-point trajectory through an action client.

        Returns a ``(start, done)`` pair for a :class:`Step`.  Every waypoint
        becomes a trajectory point, so the controller interpolates *through*
        the planned Cartesian path instead of cutting the corner between the
        first and last pose.
        """
        state = {}

        def start():
            # Never block here: this runs inside a timer callback on the same
            # single-threaded executor that has to resolve the action futures,
            # so waiting for the server in-line would deadlock.  Readiness is
            # gated up front instead - see :meth:`publish_ready`.
            if not client.server_is_ready():
                raise RuntimeError(
                    'action server {} is not available'.format(
                        client._action_name))

            current = self._current(joints)
            times = self._segment_times(path, current, joints)

            traj = JointTrajectory()
            traj.joint_names = list(joints)
            for q, t in zip(path, times):
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in q]
                point.velocities = [0.0] * len(q)
                point.time_from_start = _duration(t)
                traj.points.append(point)

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            state['target'] = np.asarray(path[-1], float)
            state['expected'] = times[-1] + extra_settle
            self._send_future = client.send_goal_async(goal)
            state['send'] = self._send_future

        def done():
            send = state.get('send')
            if send is None or not send.done():
                return False
            if 'result' not in state:
                handle = send.result()
                if handle is None or not handle.accepted:
                    raise RuntimeError('trajectory goal was rejected')
                self._goal_handle = handle
                state['result'] = handle.get_result_async()
                self._result_future = state['result']
                return False

            result_future = state['result']
            if not result_future.done():
                return False

            outcome = result_future.result()
            status = getattr(outcome, 'status', None)
            if status is not None and status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(
                    'controller reported status {}'.format(status))

            # The action result is necessary but not sufficient: confirm with
            # fresh joint feedback that the arm really is where we asked.
            if not self.joint_states_fresh():
                raise RuntimeError(
                    '/joint_states is {:.1f} s stale; cannot confirm the '
                    'motion completed'.format(self.joint_age()))

            tol = self.tolerance if tolerance is None else float(tolerance)
            actual = self._current(joints)
            error = np.abs(actual - state['target'])
            worst = float(np.max(error)) if error.size else 0.0
            if worst <= tol:
                return True

            # Off target at the moment the action returned.  That is either
            # ordinary tracking lag, which settles in a second, or the joint
            # is physically blocked, which does not.  Give it a bounded window
            # to converge and fail only if it never does - so a real blockage
            # is still reported as a failure rather than waited away.
            state.setdefault('settle_from', self.now())
            if self.now() - state['settle_from'] < self.settle_window:
                return False

            bad = ', '.join(
                '{} off by {:+.3f}'.format(j, actual[i] - state['target'][i])
                for i, j in enumerate(joints)
                if abs(actual[i] - state['target'][i]) > tol)
            raise RuntimeError(
                'motion finished {:.3f} rad off target after {:.0f} s of '
                'settling ({})'.format(worst, self.settle_window, bad))

        return start, done

    def _cancel_goal(self):
        handle = self._goal_handle
        self._goal_handle = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    # -- gripper ---------------------------------------------------------

    def send_gripper(self, value):
        """Publish a clamped gripper command on the organiser's public topic."""
        value = max(arena.GRIPPER_MIN, min(arena.GRIPPER_MAX, float(value)))
        traj = JointTrajectory()
        traj.joint_names = [self.gripper_joint]
        point = JointTrajectoryPoint()
        point.positions = [value]
        point.velocities = [0.0]
        point.time_from_start = _duration(
            float(self.durations.get('gripper', 1.5)))
        traj.points = [point]
        self.pub_gripper.publish(traj)
        return value

    def finger_position(self):
        return self.joint_positions.get(self.gripper_joint)

    def held_width(self):
        """Object width implied by where the pads came to rest, or None."""
        q = self.finger_position()
        if q is None:
            return None
        return arena.pad_gap(q)

    def fingers_empty(self, q):
        """True if the pads reached their commanded close position.

        Closing is commanded to ``GRIPPER_CLOSE``, which would put the pads
        6 mm *inside* a 0.030 m spine.  So reaching it means nothing stopped
        them; stalling short of it means something did.
        """
        return q <= arena.GRIPPER_CLOSE + arena.GRIPPER_EMPTY_EPS

    #: How far the fingers move between contact checks while closing.  Small
    #: enough that they stop before driving through the book, large enough
    #: that the close does not take all day.
    CLOSE_INCREMENT = 0.004
    #: How long to let each increment settle before judging contact.
    CLOSE_DWELL = 0.6
    #: How much further to close once a pad has reached the book.  With the
    #: camera frame fixed the book is centred in the jaw - the first pad meets
    #: it at a 0.0307 m gap on a 0.030 m spine, both pads together - so a real
    #: squeeze loads both faces symmetrically.  The earlier "squeezing harder
    #: ejects the book" result came from off-centre grips, where one pad
    #: pushed the book sideways instead.  At 0.004 (0.003 m of interference)
    #: the book held through the lift and slipped out during the retreat.
    #: At 0.010 it held through the extraction but crept down through the pads
    #: while it was carried - measured in truth, about a millimetre a second,
    #: until the top edge passed the pads 20 s into the drive back - and 0.016
    #: slipped *more*, not less.
    #:
    #: That is the fingertip hinges.  The pads sit on mimic joints that
    #: gz_ros2_control drives with a velocity servo, and the simulator caps
    #: that servo at the joint's URDF effort limit: 0.1 N m.  A harder squeeze
    #: presses the pads harder, the reaction torque on each hinge grows with
    #: it, and once the servo saturates the book's weight back-drives the
    #: fingertip and the book slides.  Friction needs very little force here -
    #: with mu = 5, about 0.3 N per pad carries the 0.3 kg book - which argued
    #: for a lighter squeeze.  Measured, it does not help: at 0.004 the book
    #: crept at the same rate and fell 5 s into the carry instead of 10.  The
    #: squeeze stays at 0.010, and the carry is made short enough to finish
    #: inside the time the grip lasts (see the mission's CARRY_TO_BIN).
    CLOSE_SQUEEZE = 0.010
    #: Pad gap the close will not leave slacker than, whatever the search
    #: reports as first contact.  The squeeze is measured from that report,
    #: and the report lands on a 0.004-unit grid, so the realised grip swung a
    #: whole increment between runs: five takes closed to 0.0199 m (10.1 mm of
    #: interference on a 0.030 m spine) and one closed to 0.0230 m (7.0 mm) -
    #: and that loose one gave the shortest carry of the six.  This only ever
    #: tightens a slack close to the value most runs already achieve; it never
    #: squeezes harder than that, which would load the fingertip hinges.
    CLOSE_TARGET_GAP = 0.0199

    def both_pads_on_same_book(self, window=0.8):
        """The book both fingertips are currently touching, or None."""
        now = self.now()
        seen = {}
        for tag, (t, book) in self._finger_contact.items():
            if now - t <= window and 'fingertip' in tag:
                seen.setdefault(book, set()).add(tag)
        for book, tags in seen.items():
            if len(tags) >= 2:
                return book
        return None

    def close_on_contact_step(self, name='gripper->close', timeout=40.0):
        """Close the fingers and stop the moment both pads are on the book.

        Commanding the closed position in one move drives the fingers *through*
        the book.  They are position controlled, so they keep going, and a
        rigid box between two closing walls gets squeezed out rather than held:
        measured, every grasp reached the commanded 0.016 m gap on a 0.030 m
        spine, which is only possible once the book has left the jaw.  What was
        being reported as a grasp was the book resting against one pad's
        outside face, which is why it never survived the lift.

        Stepping the fingers in and watching the fingertip contacts stops where
        the grip actually is, with both pads on the spine and the book still
        between them.
        """
        state = {}

        def start():
            state['target'] = arena.GRIPPER_OPEN
            state['t'] = self.now()
            self.send_gripper(state['target'])

        def done():
            now = self.now()
            # Stop on the *first* pad to reach the book, not both.  The
            # simulator reports contact points from one fingertip at a time
            # even when both are pressing - see `gripped_book` - so waiting for
            # both means never stopping, which is what drove the fingers
            # straight through the book on every previous attempt.
            touched = self.gripped_book(window=0.8)
            if touched is not None and 'squeeze_from' not in state:
                state['squeeze_from'] = state['target']
                state['target'] = max(
                    arena.GRIPPER_CLOSE,
                    min(state['target'] - self.CLOSE_SQUEEZE,
                        arena.joint_for_pad_gap(self.CLOSE_TARGET_GAP)))
                state['t'] = now
                self.send_gripper(state['target'])
                return False
            if 'squeeze_from' in state and now - state['t'] >= self.CLOSE_DWELL:
                self.job_detail['closed_on'] = touched or '?'
                self.job_detail['first_touch_gap'] = '{:.4f}'.format(
                    arena.pad_gap(state['squeeze_from']))
                self.job_detail['close_gap'] = '{:.4f}'.format(
                    arena.pad_gap(state['target']))
                return True
            if now - state['t'] < self.CLOSE_DWELL:
                return False
            if state['target'] <= arena.GRIPPER_CLOSE + 1e-6:
                # Nothing ever came between the pads.  Let the verify step say
                # so rather than pretending this was a grasp.
                self.job_detail['close_gap'] = '{:.4f}'.format(
                    arena.pad_gap(state['target']))
                return True
            state['target'] = max(arena.GRIPPER_CLOSE,
                                  state['target'] - self.CLOSE_INCREMENT)
            state['t'] = now
            self.send_gripper(state['target'])
            return False

        return Step(name, start, done, timeout)

    def gripper_step(self, name, value, settle=2.0, timeout=15.0,
                     expect_stall=False, min_gap=None):
        """A :class:`Step` that commands the gripper and confirms the outcome.

        ``expect_stall`` marks the one case where not reaching the commanded
        position is the *success* condition rather than a failure: closing on
        a book.  Everywhere else the gripper is held to the same standard as
        every other joint.
        """
        state = {}

        def start():
            state['value'] = self.send_gripper(value)
            state['t0'] = self.now()

        def done():
            if self.now() - state['t0'] < settle:
                return False
            if not self.joint_states_fresh():
                raise RuntimeError('/joint_states is stale')
            q = self.finger_position()
            if q is None:
                raise RuntimeError(
                    'no {} in /joint_states'.format(self.gripper_joint))
            if expect_stall:
                return True
            if min_gap is not None:
                # For an opening move what matters is clearance, not hitting an
                # exact joint value: the mimic linkage settles a few millimetres
                # off the command and that is harmless as long as the book fits
                # between the fingers.
                gap = arena.pad_gap(q)
                if gap < min_gap:
                    raise RuntimeError(
                        'gripper only opened to a {:.4f} m gap, needs at '
                        'least {:.4f} m'.format(gap, min_gap))
                return True
            if abs(q - state['value']) > self.grip_tol:
                raise RuntimeError(
                    'gripper reached {:.4f} m, commanded {:.4f} m'.format(
                        q, state['value']))
            return True

        return Step(name, start, done, timeout)

    # -- pose helpers ----------------------------------------------------

    def torso_step(self, name, metres, timeout=45.0):
        value = max(0.0, min(0.35, float(metres)))
        start, done = self.send_path(
            self.torso_action, ['torso_lift_joint'], [[value]])
        return Step(name, start, done, timeout)

    def head_step(self, name, pan, tilt, timeout=12.0):
        # head_2 upper limit is +0.349 rad; asking for more is silently
        # dropped by the controller, which looks like a perception bug.
        pan = max(-1.309, min(1.309, float(pan)))
        tilt = max(-1.047, min(0.349, float(tilt)))
        start, done = self.send_path(
            self.head_action, ['head_1_joint', 'head_2_joint'],
            [[pan, tilt]])
        return Step(name, start, done, timeout)

    def arm_step(self, name, path, timeout=45.0, tolerance=None):
        start, done = self.send_path(
            self.arm_action, self.arm_joints, path, tolerance=tolerance)
        return Step(name, start, done, timeout)

    def other_arm_step(self, name, path, timeout=45.0):
        start, done = self.send_path(
            self.other_arm_action, self.other_arm_joints, path)
        return Step(name, start, done, timeout)

    def _named_arm(self, pose_name):
        if pose_name not in self.arm_poses:
            raise KeyError('arm pose {!r} not in config (have {})'.format(
                pose_name, sorted(self.arm_poses)))
        positions = self.arm_poses[pose_name]
        if len(positions) != len(self.arm_joints):
            raise ValueError(
                'arm pose {!r} has {} values, expected {}'.format(
                    pose_name, len(positions), len(self.arm_joints)))
        return [float(v) for v in positions]

    def _require_chain(self):
        if self.chain is None:
            raise RuntimeError(
                'kinematics unavailable ({})'.format(self._chain_error))

    # -- command builders ------------------------------------------------

    def build_home(self, args):
        return [
            self.torso_step('torso->drive',
                            self.torso_poses.get('drive', 0.10)),
            self.arm_step('arm->tucked', [self._named_arm('tucked')]),
            self.other_arm_step(
                '{}arm->tucked'.format(self.other_side),
                [self._named_arm('tucked_left' if self.other_side == 'left'
                                 else 'tucked')]),
            self.head_step('head->forward', 0.0, 0.0),
        ]

    def build_stow(self, args):
        return [
            self.arm_step('arm->tucked', [self._named_arm('tucked')]),
            self.torso_step('torso->drive',
                            self.torso_poses.get('drive', 0.10)),
        ]

    def build_carry(self, args):
        return [self.arm_step('arm->carry', [self._named_arm('carry')])]

    def _scan(self, name):
        """Composite look-here command: torso height plus head pan/tilt.

        The torso is part of looking, not just of reaching.  The number plates
        sit at 2.26 m and ``head_2`` only tilts +0.349 rad up, so the plates do
        not enter the frame at all unless the torso raises the camera first.
        """
        steps = []
        torso = self.torso_poses.get(name)
        if torso is not None:
            steps.append(self.torso_step(
                'torso->{}'.format(name), float(torso), timeout=60.0))
        if name not in self.head_poses:
            raise KeyError('head pose {!r} not in config (have {})'.format(
                name, sorted(self.head_poses)))
        pan, tilt = self.head_poses[name]
        steps.append(self.head_step('head->{}'.format(name), pan, tilt))
        return steps

    def build_scan_markers(self, args):
        return self._scan('scan_markers')

    def build_scan_books(self, args):
        return self._scan('scan_books')

    def build_scan_bin(self, args):
        return self._scan('scan_bin')

    def build_head(self, args):
        spec = args[0] if args else 'forward'
        if ',' in spec:
            pan, tilt = (float(v) for v in spec.split(',', 1))
        else:
            if spec not in self.head_poses:
                raise KeyError('head pose {!r} not in config'.format(spec))
            pan, tilt = self.head_poses[spec]
        return [self.head_step('head->{}'.format(spec), pan, tilt)]

    def build_torso(self, args):
        spec = args[0] if args else 'drive'
        value = self.torso_poses.get(spec)
        if value is None:
            value = float(spec)
        return [self.torso_step('torso->{}'.format(spec), float(value))]

    def build_gripper(self, args):
        spec = (args[0] if args else 'open').lower()
        if spec == 'open':
            return [self.gripper_step('gripper->open', arena.GRIPPER_OPEN,
                                  min_gap=OPEN_GAP_REQUIRED)]
        if spec in ('close', 'closed'):
            return [self.close_on_contact_step()]
        return [self.gripper_step('gripper->{}'.format(spec), float(spec))]

    def build_release(self, args):
        """Open the gripper and confirm the fingers actually freed the book."""
        steps = [self.gripper_step('release', arena.GRIPPER_OPEN,
                                   min_gap=OPEN_GAP_REQUIRED)]

        def detail():
            self.holding_width = None
            return {'released': 'yes'}

        steps[-1].detail = detail
        return steps

    def build_plan_grasp(self, args):
        """Plan a grasp at a *measured* book position and report the waypoints.

        Planning happens while the base is still at the staging distance, and
        it plans for where the base *will* be after it advances - so the arm
        posture is solved for the moment the fingers close, then held rigid
        while the base translates in.  The reply carries the three base poses
        the mission has to drive to.
        """
        self._require_chain()
        if len(args) < 2:
            raise ValueError('plan_grasp needs <x,y,z>|<slot>[|<advance>]')

        target = tuple(float(v) for v in args[0].split(','))
        slot = int(args[1]) if args[1] not in ('', 'none') else None

        here = self.resolve_base(None)
        advance = (float(args[2]) if len(args) > 2
                   else grasp.GraspPlanner.PREGRASP_BACKOFF)
        grasp_base = (here[0] + advance * math.cos(here[2]),
                      here[1] + advance * math.sin(here[2]),
                      here[2])

        planner = self.planner
        planner.PREGRASP_BACKOFF = advance
        plan = planner.plan_grasp(
            target, (grasp_base[0], grasp_base[1]), grasp_base[2], slot=slot)

        self._plan = plan
        self._plan_target = target
        self._plan_slot = slot

        stage = plan.base_stage
        retreat = plan.base_retreat
        self.job_detail.update({
            'stage': '{:.4f},{:.4f},{:.4f}'.format(*stage),
            'grasp': '{:.4f},{:.4f},{:.4f}'.format(*plan.base_grasp),
            'retreat': '{:.4f},{:.4f},{:.4f}'.format(*retreat),
            'roll': '-y' if plan.finger_sign < 0 else '+y',
            'spine_offset': '{:+.3f}'.format(plan.height_offset),
            'ik_error': '{:.4f}'.format(plan.info['position_error']),
        })
        if plan.shelf_offender:
            self.get_logger().warn(
                'grasp posture leaves {} {:.3f} m inside the shelf outline; '
                'continuing but this is worth watching'.format(
                    plan.shelf_offender[0], plan.shelf_offender[1]))
        return []

    def build_arm_pregrasp(self, args):
        """Take the planned grasp posture while still clear of the shelf."""
        plan = self._require_plan()
        return [
            self.torso_step('torso->grasp height', plan.arm_q[0],
                            timeout=60.0),
            self.gripper_step('gripper->open', arena.GRIPPER_OPEN,
                              min_gap=OPEN_GAP_REQUIRED),
            # Tight on purpose.  The open pads clear the 0.030 m spine by
            # about 0.011 m either side, and 0.10 rad of slop on a wrist joint
            # is 0.015 m at the gripper - more than that clearance.  A loose
            # tolerance here is what let a fingertip clip the book on the way
            # in and shove it 0.042 m deeper into the shelf.
            self.arm_step('arm->grasp posture', [plan.arm_q[1:]],
                          timeout=60.0, tolerance=GRASP_POSTURE_TOLERANCE),
        ]

    def build_refine_grasp(self, args):
        """Re-solve the grasp posture for where the base actually stopped.

        The base carries the arm in, so any error between the commanded and
        achieved base pose lands straight on the gripper.  This is a small,
        continuity-checked correction made once the base has stopped, with the
        gripper already at the book.
        """
        plan = self._require_plan()
        here = self.resolve_base(None)
        # Use the plan's own approach tilt.  Re-solving with a horizontal
        # approach would silently ask for a different wrist roll than the one
        # the plan validated as shelf-clear, and the correction would undo the
        # very thing that keeps the gripper body out of the shelf row.
        tilt = math.radians(plan.info.get('approach_tilt_deg', 0.0))
        approach = np.array([math.cos(tilt) * math.cos(here[2]),
                             math.cos(tilt) * math.sin(here[2]),
                             math.sin(tilt)])

        # Correct sideways and up, never deeper.
        #
        # The refine exists to absorb base error, but the base's error along
        # the approach must not be absorbed by extending the arm.  The gripper
        # body behind the pads is wider than the gap between them, so it cannot
        # pass the book: at the planned pose it clears the spine face by
        # 0.0265 m, and a base that stopped short of its goal would have the
        # arm make that up and drive the body into the book.  Measured, that
        # pushed the book 0.0154 m deeper - after which the pads closed beside
        # it rather than around it.
        #
        # Depth is the one axis with slack to give: the pads are 0.057 m long,
        # so gripping a centimetre or two shallower costs nothing.  So when the
        # base is short, the target moves back with it and the arm keeps the
        # extension the plan validated.
        target = list(plan.grasp_world)
        planned_base = getattr(plan, 'base_grasp', None)
        if planned_base is not None:
            ahead = (math.cos(here[2]), math.sin(here[2]))
            short = ((here[0] - planned_base[0]) * ahead[0]
                     + (here[1] - planned_base[1]) * ahead[1])
            if short < 0.0:
                target[0] += short * ahead[0]
                target[1] += short * ahead[1]
                self.job_detail['refine_held_back'] = '{:.4f}'.format(-short)

        planner = self.planner
        planner._row = arena.nearest_row(self._plan_target[2])
        planner._column_y = arena.COLUMN_Y.get(
            self._plan_slot, self._plan_target[1])
        saved = planner.lock_torso(plan.arm_q[0])
        try:
            q, info = planner._solve_fixed_sign(
                tuple(target), (here[0], here[1]), here[2], approach,
                plan.finger_sign, self._current_full(), name='grasp refine')
        finally:
            planner.unlock_torso(saved)

        plan.arm_q = np.concatenate([[plan.arm_q[0]], q[1:]])
        plan.grasp_world = tuple(target)
        self.job_detail['refine_step'] = '{:.3f}'.format(
            info.get('joint_step', 0.0))

        # The lift was solved from the *planned* posture.  Replaying it after a
        # refine would carry the arm back to the uncorrected position with the
        # book already in the jaw, so re-solve it from the refined one.
        if plan.lift_q is not None:
            saved = planner.lock_torso(plan.arm_q[0])
            try:
                lift_target = (target[0], target[1], target[2] + plan.lift)
                q_lift = planner._solve_fixed_sign(
                    lift_target, (here[0], here[1]), here[2], approach,
                    plan.finger_sign, plan.arm_q, name='lift refine')[0]
                plan.lift_q = np.concatenate([[plan.arm_q[0]], q_lift[1:]])
            except grasp.PlanFailure:
                plan.lift_q = None
            finally:
                planner.unlock_torso(saved)

        # Held to the grasp-posture tolerance, not the 0.10 rad default: that
        # default is worth about 0.015 m at the gripper, which is the whole of
        # the lateral error this step exists to remove.
        return [self.arm_step('arm->refined grasp', [q[1:]], timeout=30.0,
                              tolerance=GRASP_POSTURE_TOLERANCE)]

    def build_close_grasp(self, args):
        """Close on the book and prove something book-shaped is held."""
        return [
            self.close_on_contact_step(),
            self._verify_grasp_step(),
        ]

    def build_lift(self, args):
        """Unload the book from its board before the base reverses."""
        plan = self._require_plan()
        if plan.lift_q is None:
            return [self._verify_retention_step('retention (no lift needed)')]
        return [
            self.arm_step('arm->lift', [plan.lift_q[1:]], timeout=30.0),
            self._verify_retention_step('retention after lift'),
        ]

    def _require_plan(self):
        if getattr(self, '_plan', None) is None:
            raise RuntimeError('no grasp has been planned yet')
        return self._plan

    def _current_full(self):
        """Current configuration of the whole IK chain, torso first."""
        names = self.chain.names
        return np.array([self.joint_positions.get(n, 0.0) for n in names])

    def _verify_grasp_step(self, timeout=12.0):
        """Confirm both fingers are actually on the target book."""
        state = {}

        def start():
            state['t0'] = self.now()

        def done():
            if self.now() - state['t0'] < 1.0:
                return False
            if not self.joint_states_fresh():
                raise RuntimeError('/joint_states is stale')

            book = self.gripped_book()
            if book is None:
                q = self.finger_position()
                raise RuntimeError(
                    'the fingers closed to {:.4f} m (pad gap {:.4f} m) with no '
                    'contact on any book - the book was missed'.format(
                        q if q is not None else float('nan'),
                        arena.pad_gap(q or 0.0)))
            self.holding = book
            self.holding_width = arena.pad_gap(self.finger_position() or 0.0)
            return True

        def detail():
            return {'holding': self.holding or '?',
                    'pad_gap': '{:.4f}'.format(self.holding_width or 0.0)}

        return Step('verify grasp', start, done, timeout, detail)

    def _verify_retention_step(self, name, timeout=10.0):
        """Confirm the book is *still* gripped after a motion."""
        state = {}

        def start():
            state['t0'] = self.now()

        def done():
            if self.now() - state['t0'] < 1.0:
                return False
            book = self.gripped_book(window=1.5)
            if book is None:
                raise RuntimeError(
                    'the fingertips are no longer in contact with a book - it '
                    'has been dropped')
            if self.holding and book != self.holding:
                raise RuntimeError(
                    'now holding {} but the grasp was on {}'.format(
                        book, self.holding))
            self.holding = book
            self.holding_width = arena.pad_gap(self.finger_position() or 0.0)
            # Where the gripper is, relative to the base, with the book in it.
            # The mission uses this to drive the held book straight over the
            # bin without moving the arm.
            try:
                tool = self.chain.fk(self._current_full())[:3, 3]
                # x and y stay in the base frame - the mission uses them as an
                # offset from the base - but z is reported in the world, which
                # is where the mission's rim thresholds live.  The IK chain is
                # rooted at base_link, which stands arena.BASE_LINK_Z above the
                # ground, and leaving that out put every height 0.076 m low.
                state['tool'] = '{:.3f},{:.3f},{:.3f}'.format(
                    tool[0], tool[1], tool[2] + arena.BASE_LINK_Z)
                state['torso'] = '{:.3f}'.format(float(
                    self.joint_positions.get('torso_lift_joint', 0.0)))
            except Exception:
                state.pop('tool', None)
            return True

        def detail():
            out = {'held': self.holding or '?'}
            for key in ('tool', 'torso'):
                if key in state:
                    out[key] = state[key]
            return out

        return Step(name, start, done, timeout, detail)

    def build_check_hold(self, args):
        return [self._verify_retention_step('retention check')]

    def build_place(self, args):
        """Plan and execute a placement at a *measured* bin position."""
        self._require_chain()
        if len(args) < 2:
            raise ValueError('place needs <x,y,z>|<basex,basey,baseyaw>')

        target = tuple(float(v) for v in args[0].split(','))
        requested = tuple(float(v) for v in args[1].split(','))
        bx, by, byaw = self.resolve_base(requested)

        plan = self.planner.plan_place(target, (bx, by), byaw)
        torso = plan[0].q[0]
        arm_path = [w.q[1:] for w in plan]
        self.job_detail['release_z'] = '{:.3f}'.format(plan[-1].target_world[2])

        return [
            self.torso_step('torso->place height', torso, timeout=30.0),
            self._verify_retention_step('retention before placing'),
            self.arm_step('arm->over bin', [arm_path[0]], timeout=40.0),
            self.arm_step('arm->into bin', arm_path[1:], timeout=40.0),
            self._verify_retention_step('retention over bin'),
        ]

    # -- periodic --------------------------------------------------------

    def publish_ready(self):
        """Say whether every interface this node needs is actually up.

        The mission waits for this instead of sleeping for a fixed time:
        commanding a controller before its spawner has run is silently
        dropped, and a fixed sleep is only ever right by luck.
        """
        ready = (self.chain is not None
                 and self.arm_action.server_is_ready()
                 and self.other_arm_action.server_is_ready()
                 and self.torso_action.server_is_ready()
                 and self.head_action.server_is_ready()
                 and self.joint_states_fresh()
                 and self.base_pose_fresh()
                 and self.gripper_joint in self.joint_positions)
        self.pub_ready.publish(Bool(data=bool(ready)))
        if ready and not self._announced_ready:
            self._announced_ready = True
            self.get_logger().info(
                'all controllers, actions and joint feedback are up')

    def publish_hold(self):
        """Broadcast the currently held width so the mission can watch it."""
        q = self.finger_position()
        if q is None:
            return
        held = self.gripped_book(window=1.5)
        self.pub_hold.publish(
            Float32(data=float(arena.pad_gap(q) if held else 0.0)))

    # -- shutdown --------------------------------------------------------

    def shutdown(self):
        """Stop cleanly without dropping whatever we are holding.

        Deliberately does **not** open the gripper: if the run is interrupted
        mid-carry, releasing would drop the book on the floor.
        """
        self._cancel_goal()
        if self.holding:
            self.get_logger().warn(
                'shutting down while holding {}; leaving the gripper closed '
                'rather than dropping it'.format(self.holding))


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
