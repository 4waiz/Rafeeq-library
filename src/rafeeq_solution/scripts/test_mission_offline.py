#!/usr/bin/env python3
"""Offline test for the mission state machine.

Stubs out rclpy and the message packages so ``mission_node`` can be driven on
a laptop with no ROS, no Gazebo and no robot. It then plays a full nominal run
and several failure runs, asserting the state sequence and the exact commands
that get published.

This catches the integration bugs that are otherwise only findable by burning
a five-minute simulation cycle: a state that waits on a status string nobody
sends, a typo'd command name, a transition that never fires.

Usage:
    python3 test_mission_offline.py
"""

import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..')))

PASS, FAIL = 0, 0


def check(name, condition, detail=''):
    global PASS, FAIL
    if condition:
        PASS += 1
        print('  PASS  {}'.format(name))
    else:
        FAIL += 1
        print('  FAIL  {}  {}'.format(name, detail))


# ---------------------------------------------------------------- stubs ----

class Msg:
    """A message stand-in: any keyword becomes an attribute."""

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


class Vec:
    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.w = 1.0


class Header:
    def __init__(self):
        self.frame_id = ''
        self.stamp = None


class Pose:
    def __init__(self):
        self.position = Vec()
        self.orientation = Vec()


class PoseStamped:
    def __init__(self):
        self.header = Header()
        self.pose = Pose()


class String:
    def __init__(self, data=''):
        self.data = data


class Empty:
    pass


class FakeDuration:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class FakeTime:
    def __init__(self, seconds):
        self.seconds = seconds

    def __sub__(self, other):
        return FakeDuration(int((self.seconds - other.seconds) * 1e9))

    def to_msg(self):
        return self.seconds


class FakeClock:
    def __init__(self):
        self.seconds = 0.0

    def now(self):
        return FakeTime(self.seconds)


class FakeLogger:
    def __init__(self, sink):
        self.sink = sink

    def _log(self, level, msg, **_kwargs):
        self.sink.append((level, str(msg)))

    def info(self, msg, **kw):
        self._log('info', msg, **kw)

    def warn(self, msg, **kw):
        self._log('warn', msg, **kw)

    def error(self, msg, **kw):
        self._log('error', msg, **kw)

    def debug(self, msg, **kw):
        self._log('debug', msg, **kw)


class FakePublisher:
    def __init__(self, topic, sink):
        self.topic = topic
        self.sink = sink

    def publish(self, msg):
        self.sink.append((self.topic, msg))


class FakeNode:
    """Enough of rclpy.node.Node for the mission node to run."""

    def __init__(self, name):
        self._name = name
        self._params = {}
        self._clock = FakeClock()
        self.published = []
        self.logs = []
        self.timers = []
        self.subscriptions = {}

    def declare_parameter(self, name, default):
        self._params[name] = default

    def get_parameter(self, name):
        return Msg(value=self._params[name])

    def set_param(self, name, value):
        self._params[name] = value

    def create_publisher(self, _type, topic, _qos):
        return FakePublisher(topic, self.published)

    def create_subscription(self, _type, topic, callback, _qos):
        self.subscriptions[topic] = callback
        return Msg()

    def create_timer(self, period, callback):
        self.timers.append((period, callback))
        return Msg()

    def get_logger(self):
        return FakeLogger(self.logs)

    def get_clock(self):
        return self._clock

    def destroy_node(self):
        pass


def install_stubs():
    """Put fake ROS modules into sys.modules before importing the node."""
    rclpy = types.ModuleType('rclpy')
    rclpy.ok = lambda: True
    rclpy.init = lambda **kw: None
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node: None

    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = FakeNode
    rclpy.node = node_mod

    duration_mod = types.ModuleType('rclpy.duration')
    duration_mod.Duration = FakeDuration
    rclpy.duration = duration_mod

    geometry = types.ModuleType('geometry_msgs')
    geometry_msg = types.ModuleType('geometry_msgs.msg')
    geometry_msg.PoseStamped = PoseStamped
    geometry.msg = geometry_msg

    std = types.ModuleType('std_msgs')
    std_msg = types.ModuleType('std_msgs.msg')
    std_msg.String = String
    std_msg.Empty = Empty
    std.msg = std_msg

    for name, module in [
            ('rclpy', rclpy), ('rclpy.node', node_mod),
            ('rclpy.duration', duration_mod),
            ('geometry_msgs', geometry), ('geometry_msgs.msg', geometry_msg),
            ('std_msgs', std), ('std_msgs.msg', std_msg)]:
        sys.modules[name] = module


install_stubs()

from rafeeq_solution import arena              # noqa: E402
from rafeeq_solution.mission_node import MissionNode   # noqa: E402


# ------------------------------------------------------------- harness ----

class Runner:
    """Drives a MissionNode by faking the rest of the system."""

    def __init__(self, column=2, colour='red'):
        self.node = MissionNode()
        self.node.set_param('shelf_column_number', column)
        self.node.set_param('book_colour', colour)
        self.node.target_column = column
        self.node.target_colour = colour
        self.visited = [self.node.state]

    @property
    def clock(self):
        return self.node.get_clock()

    def advance(self, seconds=0.2):
        self.clock.seconds += seconds
        self.node.tick()
        if self.node.state != self.visited[-1]:
            self.visited.append(self.node.state)

    def last(self, topic):
        for sent_topic, msg in reversed(self.node.published):
            if sent_topic == topic:
                return msg
        return None

    def sent_to(self, topic):
        return [m for t, m in self.node.published if t == topic]

    def reply_manip(self):
        """Answer whatever manipulation command was last issued."""
        msg = self.last('/rafeeq/manip/command')
        if msg is not None:
            self.node.on_manip(String(data='done:{}'.format(msg.data)))

    def reply_nav(self, status='reached'):
        self.node.on_nav(String(data=status))

    def reply_perception(self, status):
        self.node.on_perception(String(data=status))

    def run(self, until, limit=4000, auto=True, column_slot=2, row=3):
        """Advance until ``until`` is reached, answering every request."""
        for _ in range(limit):
            if self.node.state == until:
                return True

            state = self.node.state
            if auto:
                if state in ('HOME', 'SCAN_POSE', 'RAISE_FOR_SCAN',
                             'BOOK_POSE', 'PREGRASP', 'REACH', 'GRASP',
                             'CARRY', 'BIN_POSE', 'PLACE', 'RELEASE', 'STOW'):
                    self.reply_manip()
                elif state in ('FACE_SHELF', 'NAV_COLUMN', 'APPROACH',
                               'RETRACT', 'NAV_START', 'NAV_BIN', 'BACK_OFF'):
                    self.reply_nav()
                elif state == 'DETECT_COLUMN':
                    self.reply_perception(
                        'column:done:{}'.format(column_slot))
                elif state == 'DETECT_BOOK':
                    self.reply_perception('book:done:{}'.format(row))
                elif state == 'DETECT_BIN':
                    self.reply_perception('bin:done:12.0')

            self.advance()

        return False


NOMINAL = [
    'WAIT_SIM', 'HOME', 'FACE_SHELF', 'SCAN_POSE', 'RAISE_FOR_SCAN',
    'DETECT_COLUMN', 'NAV_COLUMN', 'BOOK_POSE', 'DETECT_BOOK', 'PREGRASP',
    'REACH', 'APPROACH', 'GRASP', 'RETRACT', 'CARRY', 'NAV_START',
    'BIN_POSE', 'DETECT_BIN', 'NAV_BIN', 'PLACE', 'RELEASE', 'BACK_OFF',
    'STOW', 'DONE',
]


def test_nominal():
    print('\n[nominal run]')
    runner = Runner(column=2, colour='red')
    reached = runner.run('DONE', column_slot=4, row=3)

    check('reaches DONE', reached, 'stuck in {}'.format(runner.node.state))
    check('visits every state in order', runner.visited == NOMINAL,
          '\n    expected {}\n    got      {}'.format(NOMINAL, runner.visited))

    # Every state must have a handler, or it would silently stall.
    for state in NOMINAL:
        if state in ('DONE',):
            continue
        check('{} has a check handler'.format(state),
              hasattr(runner.node, 'check_{}'.format(state.lower())))

    return runner


def test_navigation_goals():
    print('\n[navigation goals]')
    runner = Runner(column=2, colour='red')
    runner.run('DONE', column_slot=4, row=3)

    goals = [(g.pose.position.x, g.pose.position.y)
             for g in runner.sent_to('/rafeeq/nav/goal')]

    column_y = arena.COLUMN_Y[4]
    scan_x = arena.SHELF_X - arena.BOOK_SCAN_STANDOFF
    grasp_x = arena.SHELF_X - arena.GRASP_STANDOFF

    check('drives to the detected slot, not the requested number',
          any(abs(y - column_y) < 1e-6 for _, y in goals),
          'slot 4 is y={:+.1f}, goals were {}'.format(column_y, goals))

    check('book scan happens at {:.2f} m from the shelf face'.format(
        arena.BOOK_SCAN_STANDOFF),
        any(abs(x - scan_x) < 1e-6 for x, _ in goals),
        'want x={:.2f}, goals {}'.format(scan_x, goals))

    check('grasp approach is closer than the book scan',
          any(abs(x - grasp_x) < 1e-6 for x, _ in goals) and grasp_x > scan_x,
          'scan {:.2f} grasp {:.2f}'.format(scan_x, grasp_x))

    check('returns to the start zone',
          any(abs(x) < 1e-6 and abs(y) < 1e-6 for x, y in goals),
          str(goals))

    check('all goals are inside the arena walls',
          all(arena.ARENA_X[0] < x < arena.ARENA_X[1] and
              arena.ARENA_Y[0] < y < arena.ARENA_Y[1] for x, y in goals),
          str(goals))

    check('every goal is published in the world frame',
          all(g.header.frame_id == 'world'
              for g in runner.sent_to('/rafeeq/nav/goal')))


def test_manipulation_commands():
    print('\n[manipulation commands]')
    runner = Runner(column=2, colour='red')
    runner.run('DONE', column_slot=4, row=3)

    commands = [m.data for m in runner.sent_to('/rafeeq/manip/command')]

    check('reaches for the detected row', 'row:3' in commands, str(commands))
    check('opens the gripper before reaching',
          commands.index('gripper:open') < commands.index('row:3'),
          str(commands))
    check('closes the gripper after approaching',
          commands.index('gripper:closed') > commands.index('row:3'),
          str(commands))
    check('releases after placing',
          commands.index('place') < len(commands) - 1 - list(
              reversed(commands)).index('gripper:open'),
          str(commands))
    check('raises the torso before reading the plates',
          'torso:scan_markers' in commands, str(commands))
    check('stows the arm at the end', commands[-1] == 'home', str(commands))


def test_perception_modes():
    print('\n[perception modes]')
    runner = Runner(column=2, colour='red')
    runner.run('DONE', column_slot=4, row=3)

    modes = [m.data for m in runner.sent_to('/rafeeq/perception/mode')]
    ordered = [m for i, m in enumerate(modes) if i == 0 or m != modes[i - 1]]

    check('requests column, then book, then bin',
          ['column', 'book', 'bin'] == [
              m for m in ordered if m in ('column', 'book', 'bin')],
          str(ordered))


def test_column_not_found():
    print('\n[failure: column never detected]')
    runner = Runner(column=2, colour='red')

    for _ in range(6000):
        state = runner.node.state
        if state in ('FAILED', 'DONE'):
            break
        if state == 'DETECT_COLUMN':
            pass                        # never answer
        elif state in ('HOME', 'SCAN_POSE', 'RAISE_FOR_SCAN'):
            runner.reply_manip()
        elif state == 'FACE_SHELF':
            runner.reply_nav()
        runner.advance()

    check('gives up rather than hanging', runner.node.state == 'FAILED',
          'ended in {}'.format(runner.node.state))

    retries = [m.data for m in runner.sent_to('/rafeeq/manip/command')
               if m.data.startswith('head:')]
    check('sweeps the head while retrying', len(retries) >= 3,
          str(retries))


def test_bin_not_found_is_survivable():
    print('\n[failure: bin never detected]')
    runner = Runner(column=2, colour='red')

    for _ in range(8000):
        state = runner.node.state
        if state in ('FAILED', 'DONE'):
            break
        if state == 'DETECT_BIN':
            pass                        # never answer
        elif state in ('HOME', 'SCAN_POSE', 'RAISE_FOR_SCAN', 'BOOK_POSE',
                       'PREGRASP', 'REACH', 'GRASP', 'CARRY', 'BIN_POSE',
                       'PLACE', 'RELEASE', 'STOW'):
            runner.reply_manip()
        elif state in ('FACE_SHELF', 'NAV_COLUMN', 'APPROACH', 'RETRACT',
                       'NAV_START', 'NAV_BIN', 'BACK_OFF'):
            runner.reply_nav()
        elif state == 'DETECT_COLUMN':
            runner.reply_perception('column:done:4')
        elif state == 'DETECT_BOOK':
            runner.reply_perception('book:done:3')
        runner.advance()

    check('falls back to the known bin pose and still finishes',
          runner.node.state == 'DONE', 'ended in {}'.format(runner.node.state))


def test_blocked_counts_as_arrival():
    print('\n[shelf blocks the LiDAR during approach]')
    runner = Runner(column=2, colour='red')
    runner.run('APPROACH', column_slot=4, row=3)
    check('mission reaches APPROACH', runner.node.state == 'APPROACH',
          runner.node.state)

    runner.reply_nav('blocked')
    runner.advance()
    check('treats a blocked approach as arrival, not failure',
          runner.node.state == 'GRASP', runner.node.state)


def test_all_slots_and_rows():
    print('\n[every column and row combination]')
    ok = True
    for slot in (1, 2, 3, 4, 5):
        for row in (1, 2, 3, 4):
            runner = Runner(column=3, colour='blue')
            if not runner.run('DONE', column_slot=slot, row=row):
                ok = False
                print('    stuck: slot {} row {} in {}'.format(
                    slot, row, runner.node.state))
                continue
            goals = [(g.pose.position.x, g.pose.position.y)
                     for g in runner.sent_to('/rafeeq/nav/goal')]
            if not any(abs(y - arena.COLUMN_Y[slot]) < 1e-6 for _, y in goals):
                ok = False
                print('    wrong y for slot {}'.format(slot))
            commands = [m.data
                        for m in runner.sent_to('/rafeeq/manip/command')]
            if 'row:{}'.format(row) not in commands:
                ok = False
                print('    missing row:{} command'.format(row))

    check('all 20 slot/row combinations complete correctly', ok)


def test_frame_conversion():
    print('\n[world <-> odom frame conversion]')
    # The robot spawns at world (0, 0) with yaw +90 deg, and odom starts at
    # identity there, so the two frames are rotated by exactly that yaw.
    for wx, wy in [(3.0, 0.0), (0.0, 0.0), (-1.0, 0.0), (2.25, 2.0),
                   (1.5, -2.0)]:
        ox, oy = arena.world_to_odom(wx, wy)
        rx, ry = arena.odom_to_world(ox, oy)
        if abs(rx - wx) > 1e-9 or abs(ry - wy) > 1e-9:
            check('round trip ({}, {})'.format(wx, wy), False,
                  'got ({:.6f}, {:.6f})'.format(rx, ry))
            return

    check('world -> odom -> world round trips exactly', True)

    # SPAWN_YAW is the literal 1.5708 from simulation.launch.py, which is
    # 3.7e-6 rad short of pi/2, so these comparisons use a real tolerance
    # rather than pretending the spawn is exactly a right angle.
    tol = 1e-4

    # The robot spawns facing world +y. The shelf is at world +x, i.e. 90 deg
    # to its right, so facing the shelf means turning to odom yaw -90 deg.
    odom_yaw = arena.world_yaw_to_odom(0.0)
    check('facing the shelf means turning to odom yaw -90 deg',
          abs(odom_yaw + math.pi / 2) < tol,
          '{:.6f} rad'.format(odom_yaw))

    # odom +x runs along world +y and odom +y along world -x, so the shelf
    # 3 m ahead in world +x sits at odom (0, -3): on the robot's right at
    # spawn, not in front of it. Getting this sign wrong would send the base
    # into the wall behind the start zone.
    ox, oy = arena.world_to_odom(arena.SHELF_X, 0.0)
    check('shelf at world +x maps to odom -y (it starts on the right)',
          abs(ox) < tol and abs(oy + arena.SHELF_X) < tol,
          'odom ({:.4f}, {:.4f})'.format(ox, oy))

    # The bin is behind the start zone in world -x, so it lands at odom +y.
    bx, by = arena.world_to_odom(arena.BIN_XYZ[0], arena.BIN_XYZ[1])
    check('bin at world -x maps to odom +y',
          abs(bx) < tol and abs(by - abs(arena.BIN_XYZ[0])) < tol,
          'odom ({:.4f}, {:.4f})'.format(bx, by))


def test_arena_constants():
    print('\n[arena constants vs the competition sources]')
    check('five columns 1 m apart, centred on the shelf',
          [arena.COLUMN_Y[s] for s in (1, 2, 3, 4, 5)] ==
          [2.0, 1.0, 0.0, -1.0, -2.0], str(arena.COLUMN_Y))
    check('four book rows, descending, 0.33 m apart',
          [round(arena.ROW_Z[r], 3) for r in (1, 2, 3, 4)] ==
          [1.595, 1.265, 0.935, 0.605], str(arena.ROW_Z))
    check('nearest_column_slot snaps correctly',
          arena.nearest_column_slot(1.8) == 1 and
          arena.nearest_column_slot(-0.4) == 3 and
          arena.nearest_column_slot(-2.3) == 5)
    check('nearest_row snaps correctly',
          arena.nearest_row(1.60) == 1 and arena.nearest_row(0.62) == 4)
    check('book scan standoff exceeds the 1.17 m geometric minimum',
          arena.BOOK_SCAN_STANDOFF > 1.17,
          '{:.2f} m'.format(arena.BOOK_SCAN_STANDOFF))


def main():
    test_nominal()
    test_navigation_goals()
    test_manipulation_commands()
    test_perception_modes()
    test_blocked_counts_as_arrival()
    test_column_not_found()
    test_bin_not_found_is_survivable()
    test_all_slots_and_rows()
    test_frame_conversion()
    test_arena_constants()

    print('\n{} passed, {} failed'.format(PASS, FAIL))
    return 1 if FAIL else 0


if __name__ == '__main__':
    raise SystemExit(main())
