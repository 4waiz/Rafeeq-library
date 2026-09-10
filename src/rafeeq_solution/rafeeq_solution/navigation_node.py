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
from std_msgs.msg import Empty, String

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


class NavigationNode(Node):

    def __init__(self):
        super().__init__('rafeeq_navigation')

        self.declare_parameter('spawn_yaw', arena.SPAWN_YAW)
        self.declare_parameter('max_linear', 0.32)
        self.declare_parameter('max_angular', 0.6)
        self.declare_parameter('kp_linear', 1.1)
        self.declare_parameter('kp_angular', 1.6)
        self.declare_parameter('position_tolerance', 0.02)
        # Tolerance *across* the goal heading, which is a much tighter
        # constraint than along it.  Driving into a shelf row, an error along
        # the approach only changes how deep the pads grip - they span 0.057 m,
        # so 0.02 m is nothing.  An error across it puts a fingertip into the
        # book's spine: the pads clear it by a measured 0.0128 m per side at
        # full opening, which is *less* than the 0.02 m radial tolerance, so
        # the base could legitimately "arrive" with a finger already touching
        # the book, then drag it 0.047 m deeper as it finished advancing.  That
        # is what put the book out of the jaw before the gripper ever closed.
        self.declare_parameter('lateral_tolerance', 0.008)
        self.declare_parameter('yaw_tolerance', 0.02)
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

        self.spawn_yaw = float(self.get_parameter('spawn_yaw').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.pos_tol = float(self.get_parameter('position_tolerance').value)
        self.lat_tol = float(self.get_parameter('lateral_tolerance').value)
        self.yaw_tol = float(self.get_parameter('yaw_tolerance').value)
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
        self.goal_started = self.now()
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

        if (abs(along) <= self.pos_tol and abs(across) <= self.lat_tol
                and abs(eyaw) <= self.yaw_tol):
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
        vx = clamp(self.kp_linear * ex, self.max_linear) * scale
        vy = clamp(self.kp_linear * ey, self.max_linear) * scale

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
        if abs(across) > self.lat_tol:
            along_hat = (cos_yaw * math.cos(gyaw) - sin_yaw * math.sin(gyaw),
                         sin_yaw * math.cos(gyaw) + cos_yaw * math.sin(gyaw))
            along_speed = vx * along_hat[0] + vy * along_hat[1]
            if along_speed > 0.0:
                vx -= along_speed * along_hat[0]
                vy -= along_speed * along_hat[1]

        blocked = self._apply_clearance(vx, vy, distance)
        if blocked is not None:
            vx, vy = blocked

        cmd = Twist()
        cmd.linear.x, cmd.linear.y = vx, vy
        cmd.angular.z = clamp(self.kp_angular * eyaw, self.max_angular)
        self.pub_cmd.publish(cmd)

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
