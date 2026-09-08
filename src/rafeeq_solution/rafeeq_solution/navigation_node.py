"""Navigation node - holonomic waypoint driving on odometry.

Why not Nav2: the competition image installs Nav2 but ``simulation.launch.py``
never brings it up. There is no map server, no AMCL, no costmaps and no
``map`` frame - only ``odom -> base_footprint`` from the bridged ``/odom_tf``.
Standing all of that up is a project in itself, and the arena does not need
it: the walls, shelf, table and bin never move, and the only randomised thing
in the whole world is which number plate sits above which column.

So we drive waypoints directly. The base is a mecanum holonomic drive, which
means we can command x, y and yaw independently and strafe sideways along the
shelf without ever turning away from it.

Goals arrive on ``/rafeeq/nav/goal`` as a ``PoseStamped``. ``frame_id`` may be
``world`` (converted through the known spawn pose) or ``odom`` (used as-is).
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

from . import arena


def yaw_from_quaternion(q):
    """Extract the yaw angle from a geometry_msgs Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def clamp(value, limit):
    return max(-limit, min(limit, value))


class NavigationNode(Node):

    def __init__(self):
        super().__init__('rafeeq_navigation')

        self.declare_parameter('spawn_yaw', arena.SPAWN_YAW)
        self.declare_parameter('max_linear', 0.35)
        self.declare_parameter('max_angular', 0.6)
        self.declare_parameter('kp_linear', 0.9)
        self.declare_parameter('kp_angular', 1.4)
        self.declare_parameter('position_tolerance', 0.08)
        self.declare_parameter('yaw_tolerance', 0.06)
        self.declare_parameter('approach_slowdown', 0.6)
        self.declare_parameter('obstacle_stop_distance', 0.35)
        self.declare_parameter('obstacle_sector_deg', 40.0)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('goal_timeout', 60.0)

        self.spawn_yaw = float(self.get_parameter('spawn_yaw').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.pos_tol = float(self.get_parameter('position_tolerance').value)
        self.yaw_tol = float(self.get_parameter('yaw_tolerance').value)
        self.slowdown = float(self.get_parameter('approach_slowdown').value)
        self.stop_distance = float(
            self.get_parameter('obstacle_stop_distance').value)
        self.sector = math.radians(
            float(self.get_parameter('obstacle_sector_deg').value))
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)

        self.pose = None              # (x, y, yaw) in odom
        self.goal = None              # (x, y, yaw) in odom
        self.goal_world = None        # for logging only
        self.goal_started = None
        self.front_clearance = float('inf')
        self.state = 'idle'

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(
            String, '/rafeeq/nav/status', 10)

        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(
            LaserScan, '/scan_front_raw', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/rafeeq/nav/goal', self.on_goal, 10)
        self.create_subscription(
            Empty, '/rafeeq/nav/cancel', self.on_cancel, 10)

        self.create_timer(
            float(self.get_parameter('control_period').value), self.control)

        self.get_logger().info(
            'navigation up | holonomic waypoint control on /odom, '
            'spawn_yaw={:.4f} rad'.format(self.spawn_yaw))

    # -- subscriptions ---------------------------------------------------

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))

    def on_scan(self, msg):
        """Track the closest return in a narrow forward sector.

        The front LiDAR spans about 270 degrees, so most of it looks sideways
        and backwards. Only the readings within +/- ``obstacle_sector_deg``
        of straight ahead can actually be hit by driving forward.
        """
        if not msg.ranges:
            return

        closest = float('inf')
        angle = msg.angle_min

        for distance in msg.ranges:
            if abs(arena.normalize_angle(angle)) <= self.sector / 2.0:
                if msg.range_min < distance < msg.range_max:
                    closest = min(closest, distance)
            angle += msg.angle_increment

        self.front_clearance = closest

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
            self.goal_world = arena.odom_to_world(ox, oy, self.spawn_yaw) + (
                oyaw,)

        self.goal = (ox, oy, oyaw)
        self.goal_started = self.get_clock().now()
        self.state = 'moving'

        self.get_logger().info(
            'goal: world ({:+.2f}, {:+.2f}, {:+.1f} deg) = '
            'odom ({:+.2f}, {:+.2f}, {:+.1f} deg)'.format(
                self.goal_world[0], self.goal_world[1],
                math.degrees(self.goal_world[2]),
                ox, oy, math.degrees(oyaw)))
        self._status('moving')

    def on_cancel(self, _msg):
        self.goal = None
        self.state = 'idle'
        self.pub_cmd.publish(Twist())
        self._status('idle')
        self.get_logger().info('goal cancelled')

    # -- control loop ----------------------------------------------------

    def control(self):
        if self.goal is None or self.pose is None:
            return

        x, y, yaw = self.pose
        gx, gy, gyaw = self.goal

        ex_odom = gx - x
        ey_odom = gy - y
        eyaw = arena.normalize_angle(gyaw - yaw)
        distance = math.hypot(ex_odom, ey_odom)

        if distance <= self.pos_tol and abs(eyaw) <= self.yaw_tol:
            self.goal = None
            self.state = 'reached'
            self.pub_cmd.publish(Twist())
            self.get_logger().info(
                'reached world ({:+.2f}, {:+.2f})'.format(
                    *self.goal_world[:2]))
            self._status('reached')
            return

        elapsed = (self.get_clock().now() - self.goal_started).nanoseconds / 1e9
        if elapsed > self.goal_timeout:
            self.goal = None
            self.state = 'failed'
            self.pub_cmd.publish(Twist())
            self.get_logger().error(
                'goal timed out after {:.0f} s, {:.2f} m short'.format(
                    elapsed, distance))
            self._status('failed')
            return

        # Rotate the odom-frame error into the robot body frame - this is the
        # step that makes the mecanum base able to strafe straight at a goal.
        cos_yaw, sin_yaw = math.cos(-yaw), math.sin(-yaw)
        ex = cos_yaw * ex_odom - sin_yaw * ey_odom
        ey = sin_yaw * ex_odom + cos_yaw * ey_odom

        # Ease off near the goal so we settle instead of oscillating.
        scale = min(1.0, self.slowdown + distance)

        cmd = Twist()
        cmd.linear.x = clamp(self.kp_linear * ex, self.max_linear) * scale
        cmd.linear.y = clamp(self.kp_linear * ey, self.max_linear) * scale
        cmd.angular.z = clamp(self.kp_angular * eyaw, self.max_angular)

        if cmd.linear.x > 0.0 and self.front_clearance < self.stop_distance:
            # Something is close ahead. Keep the strafe and the turn - only
            # forward motion is dangerous - and let the caller decide.
            cmd.linear.x = 0.0
            if self.state != 'blocked':
                self.state = 'blocked'
                self.get_logger().warn(
                    'forward blocked at {:.2f} m; strafing only'.format(
                        self.front_clearance))
                self._status('blocked')
        elif self.state == 'blocked':
            self.state = 'moving'
            self._status('moving')

        self.pub_cmd.publish(cmd)

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
