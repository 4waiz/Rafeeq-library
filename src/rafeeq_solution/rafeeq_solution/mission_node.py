"""Mission node - the high-level state machine.

Drives the whole run: read the shelf numbers, go to the right column, find
the right book, pick it, carry it back to the start zone and put it in the
red bin. Perception, navigation and manipulation are all told what to do from
here; none of them talk to each other.

Every state logs on entry and on exit, with the reason, so a failed run can
be diagnosed from the console log alone (and so the competition video has a
readable narration).

State flow::

    WAIT_SIM -> HOME -> FACE_SHELF -> SCAN_POSE -> DETECT_COLUMN
             -> NAV_COLUMN -> BOOK_POSE -> DETECT_BOOK -> PREGRASP
             -> APPROACH -> GRASP -> LIFT -> RETRACT -> CARRY
             -> NAV_START -> FACE_BIN -> DETECT_BIN -> PLACE
             -> RELEASE -> BACK_OFF -> DONE

Any state that times out is reported and, where a retry makes sense
(detection states), retried with a different head pose before giving up.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Empty, String

from . import arena

#: Head pan/tilt pairs tried in order when a detection state has to retry.
SWEEP = [(0.0, 0.32), (0.0, 0.20), (-0.35, 0.30), (0.35, 0.30), (0.0, 0.05)]


class MissionNode(Node):

    def __init__(self):
        super().__init__('rafeeq_mission')

        self.declare_parameter('shelf_column_number', 2)
        self.declare_parameter('book_colour', 'red')
        self.declare_parameter('spawn_yaw', arena.SPAWN_YAW)
        self.declare_parameter('book_scan_standoff', arena.BOOK_SCAN_STANDOFF)
        self.declare_parameter('pregrasp_standoff', arena.PREGRASP_STANDOFF)
        self.declare_parameter('grasp_standoff', arena.GRASP_STANDOFF)
        self.declare_parameter('bin_standoff', 0.55)
        self.declare_parameter('startup_delay', 20.0)
        self.declare_parameter('detect_timeout', 25.0)
        self.declare_parameter('nav_timeout', 75.0)
        self.declare_parameter('manip_timeout', 20.0)
        self.declare_parameter('auto_start', True)

        self.target_column = int(
            self.get_parameter('shelf_column_number').value)
        self.target_colour = str(
            self.get_parameter('book_colour').value).lower()
        self.spawn_yaw = float(self.get_parameter('spawn_yaw').value)
        self.book_scan_standoff = float(
            self.get_parameter('book_scan_standoff').value)
        self.pregrasp_standoff = float(
            self.get_parameter('pregrasp_standoff').value)
        self.grasp_standoff = float(self.get_parameter('grasp_standoff').value)
        self.bin_standoff = float(self.get_parameter('bin_standoff').value)
        self.startup_delay = float(self.get_parameter('startup_delay').value)

        self.timeouts = {
            'detect': float(self.get_parameter('detect_timeout').value),
            'nav': float(self.get_parameter('nav_timeout').value),
            'manip': float(self.get_parameter('manip_timeout').value),
        }

        self.pub_goal = self.create_publisher(
            PoseStamped, '/rafeeq/nav/goal', 10)
        self.pub_cancel = self.create_publisher(
            Empty, '/rafeeq/nav/cancel', 10)
        self.pub_mode = self.create_publisher(
            String, '/rafeeq/perception/mode', 10)
        self.pub_manip = self.create_publisher(
            String, '/rafeeq/manip/command', 10)
        self.pub_state = self.create_publisher(
            String, '/rafeeq/mission/state', 10)

        self.nav_status = 'idle'
        self.manip_status = ''
        self.perception_status = ''

        self.create_subscription(
            String, '/rafeeq/nav/status', self.on_nav, 10)
        self.create_subscription(
            String, '/rafeeq/manip/status', self.on_manip, 10)
        self.create_subscription(
            String, '/rafeeq/perception/status', self.on_perception, 10)
        self.create_subscription(
            String, '/rafeeq/mission/start', self.on_start, 10)

        self.column_slot = None
        self.book_row = None
        self.bin_offset_px = 0.0
        self.attempt = 0

        self.state = None
        self.entered = self.get_clock().now()

        self.create_timer(0.2, self.tick)

        if self.get_parameter('auto_start').value:
            self.transition('WAIT_SIM')
        else:
            self.transition('IDLE')

        self.get_logger().info(
            '=' * 62 + '\n'
            'RAFEEQ LIBRARY ROBOT - ERC 2026 Phase 1\n'
            'target: shelf column {}, {} book\n'.format(
                self.target_column, self.target_colour) + '=' * 62)

    # -- subscriptions ---------------------------------------------------

    def on_nav(self, msg):
        self.nav_status = msg.data

    def on_manip(self, msg):
        self.manip_status = msg.data

    def on_perception(self, msg):
        self.perception_status = msg.data

        if msg.data.startswith('column:done:'):
            self.column_slot = int(msg.data.split(':')[2])
        elif msg.data.startswith('book:done:'):
            self.book_row = int(msg.data.split(':')[2])
        elif msg.data.startswith('bin:done:'):
            self.bin_offset_px = float(msg.data.split(':')[2])

    def on_start(self, _msg):
        if self.state in (None, 'IDLE', 'DONE', 'FAILED'):
            self.transition('WAIT_SIM')

    # -- state machine plumbing ------------------------------------------

    def transition(self, state):
        self.state = state
        self.entered = self.get_clock().now()
        self.attempt = 0
        self.pub_state.publish(String(data=state))
        self.get_logger().info('[STATE] {}'.format(state))

        handler = getattr(self, 'enter_{}'.format(state.lower()), None)
        if handler is not None:
            handler()

    @property
    def elapsed(self):
        return (self.get_clock().now() - self.entered).nanoseconds / 1e9

    def tick(self):
        if self.state in (None, 'IDLE', 'DONE', 'FAILED'):
            return

        handler = getattr(self, 'check_{}'.format(self.state.lower()), None)
        if handler is None:
            return

        nxt = handler()
        if nxt is not None and nxt != self.state:
            self.transition(nxt)

    def fail(self, reason):
        self.get_logger().error(
            'MISSION FAILED in {}: {}'.format(self.state, reason))
        self.pub_cancel.publish(Empty())
        self.set_mode('idle')
        return 'FAILED'

    # -- command helpers -------------------------------------------------

    def set_mode(self, mode):
        self.perception_status = ''
        self.pub_mode.publish(String(data=mode))

    def manip(self, command):
        self.manip_status = ''
        self.pub_manip.publish(String(data=command))

    def goto(self, wx, wy, wyaw):
        self.nav_status = 'moving'
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(wx)
        goal.pose.position.y = float(wy)
        goal.pose.orientation.z = math.sin(wyaw / 2.0)
        goal.pose.orientation.w = math.cos(wyaw / 2.0)
        self.pub_goal.publish(goal)

    def manip_done(self, command):
        return self.manip_status == 'done:{}'.format(command)

    def nav_done(self):
        return self.nav_status == 'reached'

    def timed_out(self, kind):
        return self.elapsed > self.timeouts[kind]

    @property
    def column_y(self):
        return arena.COLUMN_Y.get(self.column_slot, 0.0)

    # -- states ----------------------------------------------------------

    def check_wait_sim(self):
        """Let the simulation finish spawning before touching anything.

        simulation.launch.py spawns the robot at t+3 s, books and markers at
        t+5 s, controllers at t+8 s and the depth cloud at t+18 s. Commanding
        a controller before its spawner has run is silently dropped.
        """
        if self.elapsed < self.startup_delay:
            return None
        self.get_logger().info(
            'simulation settled after {:.0f} s'.format(self.elapsed))
        return 'HOME'

    def enter_home(self):
        self.manip('home')

    def check_home(self):
        if self.manip_done('home'):
            return 'FACE_SHELF'
        if self.timed_out('manip'):
            self.get_logger().warn('home timed out, continuing anyway')
            return 'FACE_SHELF'
        return None

    def enter_face_shelf(self):
        # Spawn yaw is +90 deg in world, so the robot starts side-on to the
        # shelf. Turn to face +x without leaving the start zone.
        self.goto(arena.START_XY[0], arena.START_XY[1], 0.0)

    def check_face_shelf(self):
        if self.nav_done():
            return 'SCAN_POSE'
        if self.timed_out('nav'):
            return self.fail('could not face the shelf')
        return None

    def enter_scan_pose(self):
        self.manip('head:scan_markers')

    def check_scan_pose(self):
        if self.manip_done('head:scan_markers'):
            return 'RAISE_FOR_SCAN'
        if self.timed_out('manip'):
            return 'RAISE_FOR_SCAN'
        return None

    def enter_raise_for_scan(self):
        # Plates sit at 2.26 m but head_2 only tilts +0.349 rad up, so the
        # torso has to make up the difference or they never enter the frame.
        self.manip('torso:scan_markers')

    def check_raise_for_scan(self):
        if self.manip_done('torso:scan_markers') or self.timed_out('manip'):
            return 'DETECT_COLUMN'
        return None

    def enter_detect_column(self):
        self.set_mode('column')

    def check_detect_column(self):
        if self.column_slot is not None:
            self.get_logger().info(
                'column {} is shelf slot {} at world y={:+.2f} m'.format(
                    self.target_column, self.column_slot, self.column_y))
            return 'NAV_COLUMN'

        if not self.timed_out('detect'):
            return None

        # Retry with a different head pose before giving up - the plates are
        # near the top of the frame and a small tilt change matters a lot.
        self.attempt += 1
        if self.attempt >= len(SWEEP):
            return self.fail(
                'never saw number plate {} (last status: {!r})'.format(
                    self.target_column, self.perception_status))

        pan, tilt = SWEEP[self.attempt]
        self.get_logger().warn(
            'column not found, retry {} with head pan={:+.2f} tilt={:+.2f}'
            .format(self.attempt, pan, tilt))
        self.manip('head:{},{}'.format(pan, tilt))
        self.entered = self.get_clock().now()
        self.set_mode('column')
        return None

    def enter_nav_column(self):
        # Stop at the book-scan standoff, not the grasp standoff. All four
        # rows of a column span 1.24 m vertically and the camera only covers
        # that from about 1.17 m back, so driving straight up to the shelf
        # would leave rows out of frame and mis-rank the target book.
        self.goto(arena.SHELF_X - self.book_scan_standoff, self.column_y, 0.0)

    def check_nav_column(self):
        if self.nav_done():
            return 'BOOK_POSE'
        if self.timed_out('nav'):
            return self.fail('could not reach column {}'.format(
                self.column_slot))
        return None

    def enter_book_pose(self):
        self.manip('head:scan_books')

    def check_book_pose(self):
        if self.manip_done('head:scan_books') or self.timed_out('manip'):
            return 'DETECT_BOOK'
        return None

    def enter_detect_book(self):
        self.set_mode('book')

    def check_detect_book(self):
        if self.book_row is not None:
            self.get_logger().info(
                '{} book is in row {} (nominal height {:.2f} m)'.format(
                    self.target_colour, self.book_row,
                    arena.ROW_Z[self.book_row]))
            return 'PREGRASP'

        if not self.timed_out('detect'):
            return None

        self.attempt += 1
        if self.attempt > 3:
            return self.fail(
                'never found a {} book (last status: {!r})'.format(
                    self.target_colour, self.perception_status))

        tilt = -0.15 - 0.15 * self.attempt
        self.get_logger().warn(
            'book not found, retry {} with head tilt={:+.2f}'.format(
                self.attempt, tilt))
        self.manip('head:0.0,{}'.format(tilt))
        self.entered = self.get_clock().now()
        self.set_mode('book')
        return None

    def enter_pregrasp(self):
        self.manip('gripper:open')

    def check_pregrasp(self):
        if self.manip_done('gripper:open') or self.timed_out('manip'):
            return 'REACH'
        return None

    def enter_reach(self):
        self.manip('row:{}'.format(self.book_row))

    def check_reach(self):
        if self.manip_done('row:{}'.format(self.book_row)):
            return 'APPROACH'
        if self.timed_out('manip'):
            self.get_logger().warn('reach timed out, approaching anyway')
            return 'APPROACH'
        return None

    def enter_approach(self):
        self.goto(arena.SHELF_X - self.grasp_standoff, self.column_y, 0.0)

    def check_approach(self):
        # 'blocked' is expected here - the shelf is directly ahead and the
        # LiDAR sees it. Treat it as arrival rather than a failure.
        if self.nav_done() or self.nav_status == 'blocked':
            self.pub_cancel.publish(Empty())
            return 'GRASP'
        if self.timed_out('nav'):
            self.pub_cancel.publish(Empty())
            return 'GRASP'
        return None

    def enter_grasp(self):
        self.manip('gripper:closed')

    def check_grasp(self):
        if self.manip_done('gripper:closed') or self.timed_out('manip'):
            return 'RETRACT'
        return None

    def enter_retract(self):
        self.goto(arena.SHELF_X - self.pregrasp_standoff - 0.35,
                  self.column_y, 0.0)

    def check_retract(self):
        if self.nav_done():
            return 'CARRY'
        if self.timed_out('nav'):
            self.get_logger().warn('retract timed out, carrying anyway')
            return 'CARRY'
        return None

    def enter_carry(self):
        self.manip('carry')

    def check_carry(self):
        if self.manip_done('carry') or self.timed_out('manip'):
            return 'NAV_START'
        return None

    def enter_nav_start(self):
        # Face the bin (which is at world -1, 0) on arrival, so the camera is
        # already pointing at it when we start looking.
        self.goto(arena.START_XY[0], arena.START_XY[1], math.pi)

    def check_nav_start(self):
        if self.nav_done():
            return 'BIN_POSE'
        if self.timed_out('nav'):
            return self.fail('could not return to the start zone')
        return None

    def enter_bin_pose(self):
        self.manip('head:scan_bin')

    def check_bin_pose(self):
        if self.manip_done('head:scan_bin') or self.timed_out('manip'):
            return 'DETECT_BIN'
        return None

    def enter_detect_bin(self):
        self.set_mode('bin')

    def check_detect_bin(self):
        if self.perception_status.startswith('bin:done:'):
            self.get_logger().info(
                'bin found, {:+.0f} px off centre'.format(self.bin_offset_px))
            return 'NAV_BIN'

        if not self.timed_out('detect'):
            return None

        # The bin never moves, so a detection failure is not fatal - fall
        # back to its known pose and keep going.
        self.get_logger().warn(
            'bin not detected, using known pose {}'.format(arena.BIN_XYZ))
        return 'NAV_BIN'

    def enter_nav_bin(self):
        self.goto(arena.BIN_XYZ[0] + self.bin_standoff, arena.BIN_XYZ[1],
                  math.pi)

    def check_nav_bin(self):
        if self.nav_done() or self.nav_status == 'blocked':
            self.pub_cancel.publish(Empty())
            return 'PLACE'
        if self.timed_out('nav'):
            self.pub_cancel.publish(Empty())
            return 'PLACE'
        return None

    def enter_place(self):
        self.manip('place')

    def check_place(self):
        if self.manip_done('place') or self.timed_out('manip'):
            return 'RELEASE'
        return None

    def enter_release(self):
        self.manip('gripper:open')

    def check_release(self):
        if self.manip_done('gripper:open') or self.timed_out('manip'):
            return 'BACK_OFF'
        return None

    def enter_back_off(self):
        self.goto(arena.START_XY[0], arena.START_XY[1], math.pi)

    def check_back_off(self):
        if self.nav_done() or self.timed_out('nav'):
            return 'STOW'
        return None

    def enter_stow(self):
        self.manip('home')

    def check_stow(self):
        if self.manip_done('home') or self.timed_out('manip'):
            self.get_logger().info(
                '=' * 62 + '\n'
                'MISSION COMPLETE - column {} (slot {}), {} book from row {} '
                'delivered to the collection bin\n'.format(
                    self.target_column, self.column_slot,
                    self.target_colour, self.book_row) + '=' * 62)
            return 'DONE'
        return None


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
