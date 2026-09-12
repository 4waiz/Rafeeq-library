"""Navigation node - holonomic waypoint driving on odometry.

Why not Nav2: the competition image installs Nav2 but ``simulation.launch.py``
never brings it up.  There is no map server, no AMCL, no costmaps and no ``map``
frame - only ``odom -> base_footprint`` from the bridged ``/odom_tf``.  Standing
all of that up is a project in itself and the arena does not need it: the walls,
shelf, table and bin never move.  What *is* randomised - which plate is over
which column, which row holds the target colour, and where in the column the
book sits - is handled by measuring it, not by mapping.

So this drives waypoints directly.  The base is a mecanum holonomic drive, which
means x, y and yaw are independent: the robot can strafe along the shelf without
ever turning away from it, and can carry the arm straight into a shelf row on a
pure translation.  That last property is what the grasp depends on.

Two safety properties that a front-only check does not give you:

* **The obstacle check is direction-aware and uses both lidars.**  The robot is
  holonomic, so it can drive sideways and backwards into things.  The front
  lidar alone covers about 270 degrees from one corner; the pair covers the
  full circle.  Every commanded direction is checked against the returns that
  actually lie in that direction.
* **Stale data stops the robot.**  If odometry or the scans go quiet the base is
  commanded to zero rather than continuing to integrate against a frozen pose.

Goals arrive on ``/rafeeq/nav/goal`` as a ``PoseStamped``.  ``frame_id`` may be
``world`` (converted through the known spawn pose) or ``odom`` (used as-is).
Status is reported on ``/rafeeq/nav/status`` as ``moving``, ``reached``,
``blocked`` or ``failed``.  ``blocked`` is deliberately distinct from
``reached``: an earlier version conflated them, so the mission could believe it
had arrived at a grasp pose it had never reached.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, \
    ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String

from . import arena

LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def yaw_from_quaternion(q):
    """Extract the yaw angle from a geometry_msgs Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def clamp(value, limit):
    return max(-limit, min(limit, value))


class ScanSector(object):
    """Closest return per angular sector of one lidar, in the base frame."""

    def __init__(self, sectors=36):
        self.sectors = int(sectors)
        self.ranges = [float('inf')] * self.sectors
        self.stamp = None

    def index(self, angle):
        wrapped = (angle + math.pi) % (2.0 * math.pi)
        return min(self.sectors - 1,
                   int(wrapped / (2.0 * math.pi) * self.sectors))

    def update(self, msg, mount_yaw, stamp):
        fresh = [float('inf')] * self.sectors
        angle = msg.angle_min
        lo, hi = msg.range_min, msg.range_max
        for distance in msg.ranges:
            if lo < distance < hi:
                i = self.index(angle + mount_yaw)
                if distance < fresh[i]:
                    fresh[i] = distance
            angle += msg.angle_increment
        self.ranges = fresh
        self.stamp = stamp

    def clearance(self, heading, half_width):
        """Closest return within ``half_width`` of ``heading``."""
        best = float('inf')
        step = 2.0 * math.pi / self.sectors
        for i, distance in enumerate(self.ranges):
            if distance == float('inf'):
                continue
            centre = -math.pi + (i + 0.5) * step
            if abs(arena.normalize_angle(centre - heading)) <= half_width:
                best = min(best, distance)
        return best


def _ramp_up(current, target, max_step):
    """Rate-limit a command's growth only, never its decay.

    ``arena.ramp`` limits both directions, which is wrong for the legs carried
    with a book in the jaw: a ramp that also governs slowing down cannot stop
    in time from a high speed cap - at 0.08 m/s^2 from 0.45 m/s that is 1.26 m
    of coasting - while the proportional controller's own approach to a goal is
    gradual and needs no smoothing.  A reversal still ramps: crossing zero is
    as much of a jolt to a hanging book as speeding up.
    """
    if target * current >= 0.0 and abs(target) <= abs(current):
        return target
    return arena.ramp(current, target, max_step)


class NavigationNode(Node):

    def __init__(self):
        super().__init__('rafeeq_navigation')

        self.declare_parameter('spawn_yaw', arena.SPAWN_YAW)
        # Ordinary legs - nothing is held during them, and the run is
        # judged on a clock.  The grasp approach is capped separately below,
        # because that one is a millimetre leg.
        self.declare_parameter('max_linear', 0.50)
        self.declare_parameter('max_angular', 0.90)
        #: The precise (grasp-approach) leg keeps the old, slower cap: it
        #: stops to 0.006 m and arriving fast means overshooting.
        self.declare_parameter('precise_max_linear', 0.30)
        self.declare_parameter('kp_linear', 1.1)
        self.declare_parameter('kp_angular', 1.6)
        self.declare_parameter('position_tolerance', 0.02)
        # What the grasp approach asks for instead.  It is the only leg that
        # needs this: the gripper body clears the book's spine face by 0.0265 m
        # and the pads clear the spine by 0.0128 m, so a base that stops 0.02 m
        # out drives the body into the book or closes the pads beside it.  The
        # rest of the run does not care, and holding every leg to millimetres
        # makes long traverses fight the obstacle checker instead of arriving.
        self.declare_parameter('precise_position_tolerance', 0.006)
        self.declare_parameter('precise_lateral_tolerance', 0.008)
        # Tolerance *across* the goal heading, which is a much tighter
        # constraint than along it.  Driving into a shelf row, an error along
        # the approach only changes how deep the pads grip - they span 0.057 m,
        # so 0.02 m is nothing.  An error across it puts a fingertip into the
        # book's spine: the pads clear it by a measured 0.0128 m per side at
        # full opening, which is *less* than the 0.02 m radial tolerance, so
        # the base could legitimately "arrive" with a finger already touching
        # the book, then drag it 0.047 m deeper as it finished advancing.  That
        # is what put the book out of the jaw before the gripper ever closed.
        # Ordinary goals are as forgiving across the heading as along it.
        # Only the grasp approach needs better, and it asks for it per goal -
        # holding every leg to millimetres made a long traverse fight the
        # obstacle checker instead of arriving.
        self.declare_parameter('lateral_tolerance', 0.02)
        self.declare_parameter('yaw_tolerance', 0.02)
        # What the carry to the bin asks for instead.  It does not need
        # millimetres: it needs to stop, because the book is only held for so
        # long.  The bin's mouth is 0.56 x 0.31 m and the release is a drop
        # into it, so 0.06 m and 0.08 rad (0.048 m of lateral error at the
        # gripper) is plenty - and it skips the slow proportional creep over
        # the last 0.1 m, which cost 1-2 s of a carry that was finishing
        # within a second or two of the book letting go.
        # 0.06, not 0.10: this tolerance governs the retreat too, and the
        # book's rear is measured leaving the board between 0.233 m and
        # 0.247 m of retreat.  At 0.10 the leg could finish at 0.24 m - inside
        # that band - and the drive then dragged a book that was still resting
        # on the shelf.  The one take that stopped at 0.221 m gave the
        # shortest carry on record.
        self.declare_parameter('carry_position_tolerance', 0.06)
        # 0.04 rather than 0.08 rad: at a gripper 0.555 m ahead of the base
        # that halves the sideways error at the release, and it costs no time
        # because the base turns while it translates.
        self.declare_parameter('carry_yaw_tolerance', 0.04)
        self.declare_parameter('approach_slowdown', 0.35)
        self.declare_parameter('obstacle_stop_distance', 0.42)
        self.declare_parameter('obstacle_half_angle_deg', 35.0)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('goal_timeout', 90.0)
        self.declare_parameter('data_max_age', 1.0)
        # The shelf is an obstacle we deliberately drive up to, so the plain
        # stop rule has to be relaxed once we are close to the goal - but only
        # then, and only that close.
        self.declare_parameter('ignore_obstacles_within', 0.45)
        # What the legs driven with a book in the jaw ask for instead.  The
        # pinch holds the book against gravity but has almost no stiffness
        # about its own axis, and the ordinary controller steps straight to
        # 0.6 rad/s: at the start of the 180 degree turn back to the bin, with
        # the gripper 0.8 m out, that is a 0.45 m/s sideways jerk at the pads,
        # and it threw out a book that had survived the whole extraction.
        # Speeds are capped and every component is ramped instead.
        self.declare_parameter('gentle_max_linear', 0.12)
        self.declare_parameter('gentle_max_angular', 0.25)
        self.declare_parameter('gentle_linear_accel', 0.15)
        self.declare_parameter('gentle_angular_accel', 0.25)
        # The carry to the bin: a high top speed, but the gentlest rates of
        # change.  Measured live, what shakes a hanging book out of the pads is
        # acceleration, not speed - and the honest measure is how far the book
        # survives, not how long:
        #
        #     0.30 m/s^2, cap 0.45   dropped after 0.92 m
        #     0.15 m/s^2, cap 0.12   dropped after 0.95 m
        #     0.15 m/s^2, cap 0.45   dropped after 1.75 m
        #
        # Halving the acceleration nearly doubled the distance, so it is
        # halved again here; the carry needs 2.3 m.  The ramp only limits
        # speeding *up* (see control), so slowing down is left to the
        # proportional controller, which is gradual anyway.
        self.declare_parameter('smooth_linear_accel', 0.08)
        self.declare_parameter('smooth_angular_accel', 0.12)
        # Faster than an ordinary leg: the book is only held for so long.
        self.declare_parameter('smooth_max_linear', 0.45)
        #: The carry's own turn rate.  It must NOT track max_angular: raising
        #: that to 0.90 rad/s for the ordinary legs also sped up the turn made
        #: with the book hanging from the pads, and the book came out after
        #: 3.4 s instead of 8.8 s on the run before it.
        self.declare_parameter('smooth_max_angular', 0.60)
        #: The carry approaches its goal harder than an ordinary leg.  With the
        #: ordinary gain the last 0.2 m is crawled at 0.05-0.12 m/s, about
        #: 2.5 s of a drive that the book beat by a second.  Overshooting is
        #: harmless here: the aim point is deliberately short of the bin's
        #: middle (mission_node.CARRY_NEAR_OFFSET, 0.05 m), so drifting past
        #: it lands the book more centrally.
        self.declare_parameter('smooth_kp_linear', 2.0)

        self.spawn_yaw = float(self.get_parameter('spawn_yaw').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.precise_max_linear = float(
            self.get_parameter('precise_max_linear').value)
        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.pos_tol = float(self.get_parameter('position_tolerance').value)
        self.lat_tol = float(self.get_parameter('lateral_tolerance').value)
        self.precise_pos_tol = float(
            self.get_parameter('precise_position_tolerance').value)
        self.precise_lat_tol = float(
            self.get_parameter('precise_lateral_tolerance').value)
        #: Set by /rafeeq/nav/precise for the next goal only, then cleared.
        self.precise_next = False
        self.gentle_max_linear = float(
            self.get_parameter('gentle_max_linear').value)
        self.gentle_max_angular = float(
            self.get_parameter('gentle_max_angular').value)
        self.gentle_linear_accel = float(
            self.get_parameter('gentle_linear_accel').value)
        self.gentle_angular_accel = float(
            self.get_parameter('gentle_angular_accel').value)
        self.control_period = float(
            self.get_parameter('control_period').value)
        #: Set by /rafeeq/nav/gentle for the next goal only, then cleared.
        self.gentle_next = False
        self.goal_gentle = False
        #: Last command sent, which a gentle goal ramps from.
        self.last_cmd = (0.0, 0.0, 0.0)
        self.yaw_tol = float(self.get_parameter('yaw_tolerance').value)
        self.carry_pos_tol = float(
            self.get_parameter('carry_position_tolerance').value)
        self.carry_yaw_tol = float(
            self.get_parameter('carry_yaw_tolerance').value)
        self.slowdown = float(self.get_parameter('approach_slowdown').value)
        self.stop_distance = float(
            self.get_parameter('obstacle_stop_distance').value)
        self.half_angle = math.radians(
            float(self.get_parameter('obstacle_half_angle_deg').value))
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.data_max_age = float(self.get_parameter('data_max_age').value)
        self.ignore_within = float(
            self.get_parameter('ignore_obstacles_within').value)

        self.pose = None              # (x, y, yaw) in odom
        self.pose_stamp = None
        self.goal = None
        self.goal_world = None
        self.goal_started = None
        self.goal_precise = False
        self.state = 'idle'

        # Mount yaw of each lidar in the base frame, from the measured TF:
        # front at (+0.275, -0.183) rotated -45 deg, rear at the opposite
        # corner rotated +135 deg.
        self.front = ScanSector()
        self.rear = ScanSector()

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(
            String, '/rafeeq/nav/status', 10)
        # Published for RViz and for the report: what we intended to drive and
        # what the base actually did.  With no Nav2 there is no /plan topic,
        # so these are the honest equivalent - and they are labelled as such.
        self.pub_plan = self.create_publisher(
            Path, '/rafeeq/nav/planned_path', LATCHED)
        self.pub_trail = self.create_publisher(
            Path, '/rafeeq/nav/executed_path', 10)

        self.planned = Path()
        self.planned.header.frame_id = 'odom'
        self.trail = Path()
        self.trail.header.frame_id = 'odom'

        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        # Asking for millimetre precision on the next goal only.  See
        # `precise_position_tolerance`.
        self.create_subscription(
            Bool, '/rafeeq/nav/precise',
            lambda m: setattr(self, 'precise_next', bool(m.data)), 10)
        # Asking for slow, ramped motion on the next goal only - the legs
        # driven with a book in the jaw.  See `gentle_max_angular`.
        self.create_subscription(
            Bool, '/rafeeq/nav/gentle',
            lambda m: setattr(self, 'gentle_next', bool(m.data)), 10)
        # Ordinary speeds, ramped, for the next goal only - the carry to the
        # bin.  See `smooth_linear_accel`.
        self.smooth_linear_accel = float(
            self.get_parameter('smooth_linear_accel').value)
        self.smooth_angular_accel = float(
            self.get_parameter('smooth_angular_accel').value)
        self.smooth_max_linear = float(
            self.get_parameter('smooth_max_linear').value)
        self.smooth_max_angular = float(
            self.get_parameter('smooth_max_angular').value)
        self.smooth_kp_linear = float(
            self.get_parameter('smooth_kp_linear').value)
        self.smooth_next = False
        self.goal_smooth = False
        self.create_subscription(
            Bool, '/rafeeq/nav/smooth',
            lambda m: setattr(self, 'smooth_next', bool(m.data)), 10)
        self.create_subscription(
            LaserScan, '/scan_front_raw', self.on_front,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan_rear_raw', self.on_rear,
            qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/rafeeq/nav/goal', self.on_goal, 10)
        self.create_subscription(
            Empty, '/rafeeq/nav/cancel', self.on_cancel, 10)

        self.create_timer(
            float(self.get_parameter('control_period').value), self.control)
        self.create_timer(0.5, self.publish_trail)

        self.get_logger().info(
            'navigation up | holonomic waypoint control on /odom, both lidars '
            'used for a full-circle clearance check, spawn_yaw={:.4f} rad'
            .format(self.spawn_yaw))

    # -- subscriptions ---------------------------------------------------

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))
        self.pose_stamp = self.now()

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.trail.poses.append(pose)
        if len(self.trail.poses) > 4000:
            del self.trail.poses[:1000]

    def on_front(self, msg):
        self.front.update(msg, -math.pi / 4.0, self.now())

    def on_rear(self, msg):
        self.rear.update(msg, 3.0 * math.pi / 4.0, self.now())

    def data_fresh(self):
        if self.pose_stamp is None:
            return False
        return (self.now() - self.pose_stamp) <= self.data_max_age

    def scans_fresh(self):
        stamps = [s for s in (self.front.stamp, self.rear.stamp)
                  if s is not None]
        if not stamps:
            return False
        return (self.now() - max(stamps)) <= self.data_max_age

    def clearance(self, heading):
        """Closest obstacle in a body-frame ``heading``, from both lidars."""
        return min(self.front.clearance(heading, self.half_angle),
                   self.rear.clearance(heading, self.half_angle))

    def on_goal(self, msg):
        yaw = yaw_from_quaternion(msg.pose.orientation)
        frame = (msg.header.frame_id or 'world').lower()

        if frame in ('world', 'map', ''):
            ox, oy = arena.world_to_odom(
                msg.pose.position.x, msg.pose.position.y, self.spawn_yaw)
            oyaw = arena.world_yaw_to_odom(yaw, self.spawn_yaw)
            self.goal_world = (msg.pose.position.x, msg.pose.position.y, yaw)
        else:
            ox, oy, oyaw = msg.pose.position.x, msg.pose.position.y, yaw
            self.goal_world = arena.odom_to_world(
                ox, oy, self.spawn_yaw) + (oyaw,)

        if not (arena.ARENA_X[0] <= self.goal_world[0] <= arena.ARENA_X[1]
                and arena.ARENA_Y[0] <= self.goal_world[1] <= arena.ARENA_Y[1]):
            self.get_logger().error(
                'refusing a goal outside the arena: world ({:+.2f}, {:+.2f})'
                .format(self.goal_world[0], self.goal_world[1]))
            self._status('failed')
            return

        self.goal = (ox, oy, oyaw)
        self.goal_precise = self.precise_next
        self.precise_next = False
        if self.goal_precise:
            self.get_logger().info(
                'this goal is the grasp approach: holding to {:.3f} m along '
                'and {:.3f} m across the heading'.format(
                    self.precise_pos_tol, self.precise_lat_tol))
        self.goal_gentle = self.gentle_next
        self.gentle_next = False
        self.goal_smooth = self.smooth_next and not self.goal_gentle
        self.smooth_next = False
        if self.goal_smooth:
            self.get_logger().info(
                'this goal carries the book to the bin: ordinary speeds, '
                'ramped at {:.2f} m/s^2 and {:.2f} rad/s^2'.format(
                    self.smooth_linear_accel, self.smooth_angular_accel))
        if self.goal_gentle:
            self.get_logger().info(
                'this goal carries the book: at most {:.2f} m/s and {:.2f} '
                'rad/s, ramped at {:.2f} m/s^2 and {:.2f} rad/s^2'.format(
                    self.gentle_max_linear, self.gentle_max_angular,
                    self.gentle_linear_accel, self.gentle_angular_accel))
        self.goal_started = self.now()
        # A goal arriving while the base is already driving keeps the ramp
        # state; one arriving from rest starts it from zero.
        self.started_from_rest = self.state != 'moving'
        self.state = 'moving'

        waypoint = PoseStamped()
        waypoint.header.frame_id = 'odom'
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.pose.position.x = ox
        waypoint.pose.position.y = oy
        waypoint.pose.orientation.z = math.sin(oyaw / 2.0)
        waypoint.pose.orientation.w = math.cos(oyaw / 2.0)
        self.planned.poses.append(waypoint)
        self.planned.header.stamp = waypoint.header.stamp
        self.pub_plan.publish(self.planned)

        self.get_logger().info(
            'goal: world ({:+.2f}, {:+.2f}, {:+.1f} deg)'.format(
                self.goal_world[0], self.goal_world[1],
                math.degrees(self.goal_world[2])))
        self._status('moving')

    def on_cancel(self, _msg):
        self.goal = None
        self.state = 'idle'
        self.pub_cmd.publish(Twist())
        self._status('idle')

    # -- control loop ----------------------------------------------------

    def control(self):
        if self.goal is None:
            return

        if not self.data_fresh():
            self.pub_cmd.publish(Twist())
            self.get_logger().error(
                'odometry is stale; stopping the base rather than driving '
                'blind', throttle_duration_sec=2.0)
            return

        x, y, yaw = self.pose
        gx, gy, gyaw = self.goal

        ex_odom, ey_odom = gx - x, gy - y
        eyaw = arena.normalize_angle(gyaw - yaw)
        distance = math.hypot(ex_odom, ey_odom)

        # Split the error into along- and across-heading components: they are
        # not equally forgiving.  See `lateral_tolerance`.
        cg, sg = math.cos(gyaw), math.sin(gyaw)
        along = ex_odom * cg + ey_odom * sg
        across = -ex_odom * sg + ey_odom * cg

        pos_tol = self.precise_pos_tol if self.goal_precise else self.pos_tol
        lat_tol = self.precise_lat_tol if self.goal_precise else self.lat_tol
        yaw_tol = self.yaw_tol
        if self.goal_smooth or self.goal_gentle:
            # Both legs carried with a book in the jaw: the retreat only has to
            # clear the shelf and the carry only has to reach the bin's mouth.
            # Converging to 0.02 m spends a second of proportional creep that
            # the book, which lets go after 6-11 s, cannot spare.
            pos_tol = max(pos_tol, self.carry_pos_tol)
            lat_tol = max(lat_tol, self.carry_pos_tol)
            yaw_tol = max(yaw_tol, self.carry_yaw_tol)

        if (abs(along) <= pos_tol and abs(across) <= lat_tol
                and abs(eyaw) <= yaw_tol):
            self.goal = None
            self.state = 'reached'
            self.pub_cmd.publish(Twist())
            self.get_logger().info(
                'reached world ({:+.3f}, {:+.3f}), {:.3f} m along and '
                '{:.3f} m across the heading, {:.1f} deg of residual '
                'error'.format(
                    self.goal_world[0], self.goal_world[1], abs(along),
                    abs(across), math.degrees(abs(eyaw))))
            self._status('reached')
            return

        elapsed = self.now() - self.goal_started
        if elapsed > self.goal_timeout:
            self.goal = None
            self.state = 'failed'
            self.pub_cmd.publish(Twist())
            self.get_logger().error(
                'goal timed out after {:.0f} s, still {:.2f} m short'.format(
                    elapsed, distance))
            self._status('failed')
            return

        # Rotate the odom-frame error into the body frame - this is what lets
        # the mecanum base strafe straight at a goal.
        cos_yaw, sin_yaw = math.cos(-yaw), math.sin(-yaw)
        ex = cos_yaw * ex_odom - sin_yaw * ey_odom
        ey = sin_yaw * ex_odom + cos_yaw * ey_odom

        scale = min(1.0, self.slowdown + distance)
        cap_linear = (self.precise_max_linear if self.goal_precise
                      else self.max_linear)
        kp_linear = (self.smooth_kp_linear if self.goal_smooth
                     else self.kp_linear)
        vx = clamp(kp_linear * ex, cap_linear) * scale
        vy = clamp(kp_linear * ey, cap_linear) * scale

        # Line up before going in, not while going in.
        #
        # Tightening the *arrival* tolerance constrains where the base stops,
        # not the path it takes to get there, and the path is what matters
        # here: driving the open gripper into a shelf row, the pads clear the
        # book's spine by a measured 0.0128 m on the tighter side.  A base that
        # wanders further than that sideways part-way in catches a fingertip on
        # the spine and drags the book deeper - measured at 0.047 m - so by the
        # time the fingers close the book is no longer between them.  It then
        # closes to its commanded 0.016 m gap on a 0.030 m book, touching it
        # with one pad's outside face, which cannot lift anything.
        #
        # So while the across-heading error is outside tolerance, the
        # along-heading velocity is suppressed: the base corrects sideways
        # first and only then advances.
        # Only on the grasp approach.  On a long traverse this deadlocks:
        # zeroing the forward component leaves the base strafing, the obstacle
        # checker vetoes the sideways direction, and the goal fails with "the
        # path is blocked" - which is exactly what it did to a trial that had
        # never failed that way before.
        if self.goal_precise and abs(across) > lat_tol:
            along_hat = (cos_yaw * math.cos(gyaw) - sin_yaw * math.sin(gyaw),
                         sin_yaw * math.cos(gyaw) + cos_yaw * math.sin(gyaw))
            along_speed = vx * along_hat[0] + vy * along_hat[1]
            if along_speed > 0.0:
                vx -= along_speed * along_hat[0]
                vy -= along_speed * along_hat[1]

        wz = clamp(self.kp_angular * eyaw, self.max_angular)

        # Carrying the book: cap every component and ramp it from the last
        # command instead of stepping.  Ramped before the obstacle veto, so a
        # veto still stops the base at once.  Each new goal ramps up from
        # standstill, which is where the previous goal left the base.
        #
        # The carry to the bin (`goal_smooth`) keeps the ordinary top speeds
        # and only ramps: it has to be quick, because the book only stays in
        # the pads for so long once it hangs free.
        if self.goal_gentle or self.goal_smooth:
            # Reset the ramp only when the base really is at rest.  Keying
            # this on the goal object meant a goal re-issued mid-drive threw
            # away the speed the base actually had: the command collapsed from
            # 0.45 m/s to 0.004 m/s in one tick, a ~9 m/s^2 spike, and then
            # took 5.6 s to climb back.
            if getattr(self, 'started_from_rest', True):
                self.last_cmd = (0.0, 0.0, 0.0)
                self.started_from_rest = False
            if self.goal_gentle:
                cap_v, cap_w = self.gentle_max_linear, self.gentle_max_angular
                dv = self.gentle_linear_accel * self.control_period
                dw = self.gentle_angular_accel * self.control_period
            else:
                cap_v, cap_w = self.smooth_max_linear, self.smooth_max_angular
                dv = self.smooth_linear_accel * self.control_period
                dw = self.smooth_angular_accel * self.control_period
            # Ramped in the odom frame, not the body frame.  The carry turns
            # 180 degrees while it drives, and a body-frame ramp spends the
            # whole leg chasing its own rotation: simulated over this
            # controller, that swerved the base 0.29-0.71 m off the straight
            # line and cost up to 2.7 s - out of a grip that lasts ten.
            # Capping the magnitude rather than each axis also stops a diagonal
            # leg running at sqrt(2) times the speed the log claims.
            c, s = math.cos(yaw), math.sin(yaw)
            ox, oy = c * vx - s * vy, s * vx + c * vy
            speed = math.hypot(ox, oy)
            if speed > cap_v:
                ox, oy = ox * cap_v / speed, oy * cap_v / speed
            # Only speeding up is rate-limited.  A ramp that also governs
            # slowing down cannot stop in time from a high cap - at 0.08 m/s^2
            # from 0.45 m/s that is 1.26 m of coasting, straight into the
            # table - and the proportional controller's own approach is
            # gradual, so there is nothing to smooth on the way down.
            ox = _ramp_up(self.last_cmd[0], ox, dv)
            oy = _ramp_up(self.last_cmd[1], oy, dv)
            wz = _ramp_up(self.last_cmd[2], clamp(wz, cap_w), dw)
            vx, vy = c * ox + s * oy, -s * ox + c * oy

        blocked = self._apply_clearance(vx, vy, distance)
        if blocked is not None:
            vx, vy = blocked

        cmd = Twist()
        cmd.linear.x, cmd.linear.y = vx, vy
        cmd.angular.z = wz
        self.pub_cmd.publish(cmd)
        cy, sy = math.cos(yaw), math.sin(yaw)
        self.last_cmd = (cy * vx - sy * vy, sy * vx + cy * vy, wz)
        self.last_cmd_goal = self.goal

    def _apply_clearance(self, vx, vy, distance):
        """Veto motion in any direction that is not clear.

        Returns the corrected ``(vx, vy)`` if anything was vetoed, else None.
        """
        speed = math.hypot(vx, vy)
        if speed < 1e-3:
            return None

        if not self.scans_fresh():
            self.get_logger().error(
                'laser scans are stale; refusing to translate',
                throttle_duration_sec=2.0)
            self._set_state('blocked')
            return (0.0, 0.0)

        # Close to the goal we are deliberately approaching something - the
        # shelf, the table - so the veto is lifted inside that radius only.
        if distance <= self.ignore_within:
            return None

        heading = math.atan2(vy, vx)
        clear = self.clearance(heading)
        if clear >= self.stop_distance:
            if self.state == 'blocked':
                self._set_state('moving')
            return None

        self.get_logger().warn(
            'obstacle {:.2f} m away at {:+.0f} deg in the body frame; holding '
            'that direction'.format(clear, math.degrees(heading)),
            throttle_duration_sec=2.0)
        self._set_state('blocked')
        return (0.0, 0.0)

    def _set_state(self, state):
        if self.state != state:
            self.state = state
            self._status(state)

    def publish_trail(self):
        if self.trail.poses:
            self.trail.header.stamp = self.get_clock().now().to_msg()
            self.pub_trail.publish(self.trail)

    def _status(self, text):
        self.pub_status.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
