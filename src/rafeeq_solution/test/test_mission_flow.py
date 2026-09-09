"""Offline tests for the mission state machine.

``rclpy`` and the message packages are stubbed so the node can be driven on a
laptop with no ROS, no Gazebo and no robot.  These catch the integration bugs
that are otherwise only findable by burning a five-minute simulation cycle: a
state waiting on a status nobody sends, a typo'd verb, a transition that fires
on a timer instead of on an observation.

The property being defended here is the one this package got wrong before:
**nothing may reach a successful terminal state because time passed.**
"""

import os
import sys
import types

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
PKG = os.path.join(REPO, 'src', 'rafeeq_solution')
sys.path.insert(0, PKG)


# ------------------------------------------------------------------ stubs --

class Msg(object):
    def __init__(self, **kw):
        self.data = kw.pop('data', None)
        for k, v in kw.items():
            setattr(self, k, v)


class _Vec(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.w = 1.0


class _Header(object):
    def __init__(self):
        self.frame_id = ''
        self.stamp = None


class _Pose(object):
    def __init__(self):
        self.position = _Vec()
        self.orientation = _Vec()


class PoseStamped(object):
    def __init__(self):
        self.header = _Header()
        self.pose = _Pose()


class PointStamped(object):
    def __init__(self):
        self.header = _Header()
        self.point = _Vec()


class _Clock(object):
    """A clock the test drives by hand."""
    def __init__(self):
        self.t = 0.0

    def now(self):
        clock = self

        class _T(object):
            nanoseconds = int(clock.t * 1e9)

            def __sub__(self, other):
                class _D(object):
                    nanoseconds = int(clock.t * 1e9) - other.nanoseconds
                return _D()

            def to_msg(self):
                return None
        return _T()


class _Logger(object):
    def __init__(self):
        self.errors = []

    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def error(self, msg, *a, **k):
        self.errors.append(str(msg))


class _Publisher(object):
    def __init__(self, sink, topic):
        self.sink = sink
        self.topic = topic

    def publish(self, msg):
        self.sink.append((self.topic, getattr(msg, 'data', msg)))


def _install_stubs():
    rclpy = types.ModuleType('rclpy')

    class Node(object):
        #: Values a test wants the node to see, consulted by
        #: ``declare_parameter``.  Set on the class before construction: the
        #: node calls ``super().__init__`` itself, so anything stashed on the
        #: instance beforehand is wiped.
        TEST_PARAMS = {}

        def __init__(self, name):
            self._params = {}
            self._clock = _Clock()
            self._log = _Logger()
            self.published = []
            self.timers = []

        def declare_parameter(self, name, default):
            self._params.setdefault(name, self.TEST_PARAMS.get(name, default))

        def get_parameter(self, name):
            return Msg(value=self._params[name])

        def set_parameters_for_test(self, **kw):
            self._params.update(kw)

        def create_publisher(self, _type, topic, _qos):
            return _Publisher(self.published, topic)

        def create_subscription(self, *a, **k):
            return None

        def create_timer(self, period, cb):
            self.timers.append(cb)
            return None

        def get_clock(self):
            return self._clock

        def get_logger(self):
            return self._log

    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = Node
    qos_mod = types.ModuleType('rclpy.qos')
    for name in ('DurabilityPolicy', 'HistoryPolicy', 'ReliabilityPolicy'):
        setattr(qos_mod, name, types.SimpleNamespace(
            TRANSIENT_LOCAL=1, KEEP_LAST=1, RELIABLE=1))
    qos_mod.QoSProfile = lambda **kw: None
    rclpy.node = node_mod
    rclpy.qos = qos_mod
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.ok = lambda: False
    rclpy.spin = lambda *a, **k: None

    geo = types.ModuleType('geometry_msgs.msg')
    geo.PoseStamped = PoseStamped
    geo.PointStamped = PointStamped
    std = types.ModuleType('std_msgs.msg')
    std.String = lambda **kw: Msg(**kw)
    std.Int32 = lambda **kw: Msg(**kw)
    std.Bool = lambda **kw: Msg(**kw)
    std.Float32 = lambda **kw: Msg(**kw)
    std.Empty = lambda **kw: Msg(**kw)
    gz = types.ModuleType('ros_gz_interfaces.msg')
    gz.Contacts = object

    for name, mod in (
            ('rclpy', rclpy), ('rclpy.node', node_mod), ('rclpy.qos', qos_mod),
            ('geometry_msgs', types.ModuleType('geometry_msgs')),
            ('geometry_msgs.msg', geo),
            ('std_msgs', types.ModuleType('std_msgs')), ('std_msgs.msg', std),
            ('ros_gz_interfaces', types.ModuleType('ros_gz_interfaces')),
            ('ros_gz_interfaces.msg', gz)):
        sys.modules[name] = mod


_install_stubs()
from rafeeq_solution.mission_node import MissionNode   # noqa: E402


# ------------------------------------------------------------------ helpers --

def make(column=2, colour='red', **overrides):
    from rclpy.node import Node as StubNode
    params = {
        'shelf_column_number': column, 'book_colour': colour,
        'bin_standoff': 0.55, 'detect_timeout': 40.0, 'nav_timeout': 90.0,
        'manip_timeout': 150.0, 'ready_timeout': 180.0, 'results_dir': '',
        'auto_start': True, 'run_id': 'test',
    }
    params.update(overrides)
    StubNode.TEST_PARAMS = params
    try:
        node = MissionNode.__new__(MissionNode)
        MissionNode.__init__(node)
    finally:
        StubNode.TEST_PARAMS = {}
    return node


def advance(node, seconds):
    node._clock.t += seconds


def reply(node, ok=True, detail='', verb=None):
    """Answer the manipulation command the node is currently waiting on."""
    node.manip_reply = ('done' if ok else 'failed',
                        verb or 'x', detail)


def sent_verbs(node):
    return [d.split('|')[1] for t, d in node.published
            if t == '/rafeeq/manip/command' and isinstance(d, str)]


# ------------------------------------------------------------------- tests --

def test_invalid_column_fails_before_anything_moves():
    node = make(column=9)
    assert node.state == 'FAILED'
    assert 'shelf_column_number' in node.invalid
    assert not sent_verbs(node), 'commanded motion despite bad arguments'


def test_invalid_colour_fails_before_anything_moves():
    node = make(colour='purple')
    assert node.state == 'FAILED'
    assert 'book_colour' in node.invalid
    assert not sent_verbs(node)


@pytest.mark.parametrize('column', [1, 2, 3, 4, 5])
@pytest.mark.parametrize('colour', ['red', 'green', 'blue', 'yellow'])
def test_every_valid_input_combination_is_accepted(column, colour):
    node = make(column=column, colour=colour)
    assert node.invalid == ''
    assert node.state == 'WAIT_READY'


def test_the_trial_clock_waits_for_a_live_simulation_clock():
    """With use_sim_time the clock reads 0 until /clock arrives, then jumps.

    Latching the start time before that made the first elapsed reading equal
    to the simulator's whole uptime, so every timeout fired at once and the
    run failed with "controllers never became ready" 0.06 s after starting.
    """
    node = make()
    node._clock.t = 0.0
    node.tick()
    assert node.started is None, 'started the trial clock before /clock'
    node._clock.t = 693.0            # simulator already running
    node.tick()
    assert node.started == pytest.approx(693.0)
    node.manip_ready = True
    node.tick()
    assert node.state == 'HOME', 'a live clock jump was treated as a timeout'


def test_readiness_is_observed_not_slept_through():
    node = make()
    node._clock.t = 1.0
    node.tick()
    assert node.state == 'WAIT_READY'
    advance(node, 60.0)
    node.tick()
    assert node.state == 'WAIT_READY', 'advanced without a readiness signal'
    node.manip_ready = True
    node.tick()
    assert node.state == 'HOME'


def test_readiness_timeout_fails_rather_than_continuing():
    node = make()
    node._clock.t = 1.0
    node.tick()                      # start the trial clock
    advance(node, 200.0)
    node.tick()
    assert node.state == 'FAILED'
    assert 'ready' in node.failure_reason


def test_a_manipulation_timeout_never_counts_as_success():
    node = make()
    node._clock.t = 1.0
    node.manip_ready = True
    node.tick()
    node.tick()
    assert node.state == 'HOME'
    advance(node, 500.0)
    node.tick()
    assert node.state == 'FAILED', (
        'HOME advanced on a timeout; that is the false-success bug')


def test_navigation_blocked_is_not_arrival():
    node = make()
    node._clock.t = 1.0
    node.manip_ready = True
    node.tick()
    node.tick()
    reply(node, ok=True)
    node.tick()
    assert node.state == 'FACE_SHELF'
    node.nav_status = 'blocked'
    advance(node, 20.0)
    node.tick()
    assert node.state == 'FAILED', 'a blocked path was treated as arrival'


def test_a_reply_to_an_older_command_is_ignored():
    node = make()
    node._clock.t = 1.0
    node.manip_ready = True
    node.tick()
    node.tick()                      # -> HOME, sends seq 1
    seq = node.manip_seq
    node.on_manip(Msg(data='done|{}|home|'.format(seq - 1)))
    assert node.manip_reply is None, 'accepted an answer to a previous command'
    node.on_manip(Msg(data='done|{}|home|'.format(seq)))
    assert node.manip_reply is not None


def test_column_topic_is_only_awarded_after_a_real_identification():
    node = make()
    assert 'column_topic' not in node.score.awarded
    node.state = 'DETECT_COLUMN'
    node.entered = node._clock.now()
    node.tick()
    assert 'column_topic' not in node.score.awarded, (
        'awarded the column before anything was identified')
    # The evidence path is captured when perception announces it, not read
    # back out of the status later - the slot also arrives on a latched topic
    # and can advance the state first, which used to lose the image credit.
    node.on_perception(Msg(data='column:done:2:/tmp/shelf_column_x.png'))
    node.tick()
    assert 'column_topic' in node.score.awarded
    assert 'column_image' in node.score.awarded
    assert node.evidence['column_image'] == '/tmp/shelf_column_x.png'


def test_column_image_points_are_not_claimed_without_an_image():
    node = make()
    node.state = 'DETECT_COLUMN'
    node.entered = node._clock.now()
    node.column_slot = 2                 # slot arrives, no image announced
    node.tick()
    assert 'column_topic' in node.score.awarded
    assert 'column_image' not in node.score.awarded


def test_grasp_failure_retries_then_fails_and_never_reaches_done():
    node = make()
    node.book_point = (2.9, 0.2, 1.25)
    node.column_slot = 3
    node.plan_poses = {'retreat': '1.9,0.2,0.0', 'grasp': '2.4,0.2,0.0'}
    node.state = 'CLOSE_GRASP'
    node.entered = node._clock.now()
    for _ in range(3):
        reply(node, ok=False, detail='the book was missed')
        node.tick()
        if node.state == 'FAILED':
            break
        node.state = 'CLOSE_GRASP'
        node.entered = node._clock.now()
    assert node.state == 'FAILED'
    assert node.grasp_attempts >= 3
    assert 'grasp' not in node.score.awarded


def test_delivery_requires_the_official_contact_not_just_letting_go():
    node = make()
    node.state = 'CONFIRM_DELIVERY'
    node.entered = node._clock.now()
    node.tick()
    assert node.state == 'CONFIRM_DELIVERY'
    assert 'return_with_book' not in node.score.awarded
    advance(node, 30.0)
    node.tick()
    assert node.state == 'FAILED', (
        'the gripper opened and the run was called delivered')


def test_delivery_is_awarded_once_the_bin_reports_a_book():
    node = make()
    node.state = 'CONFIRM_DELIVERY'
    node.entered = node._clock.now()
    node.plan_poses = {'release_z': '0.880'}
    node.delivery.first_contact_time = 1.0
    node.delivery.first_contact_model = 'book_col_2_row_3_red'
    node.tick()
    assert node.state == 'BACK_OFF'
    assert 'return_with_book' in node.score.awarded
    assert node.score.placement in ('gentle', 'drop')


def test_shelf_navigation_points_ride_on_the_grasp_not_the_waypoint():
    node = make()
    node.column_slot = 2
    node.state = 'CONFIRM_HOLD'
    node.entered = node._clock.now()
    node.tick()
    assert 'navigate_shelf' not in node.score.awarded
    reply(node, ok=True, detail='held=book_col_2_row_3_red')
    node.tick()
    assert node.state == 'NAV_BIN_AREA'
    assert 'navigate_shelf' in node.score.awarded
    assert 'grasp' in node.score.awarded


def test_losing_the_book_on_the_way_back_fails_the_run():
    node = make()
    node.state = 'NAV_BIN_AREA'
    node.entered = node._clock.now()
    node.hold_width = 0.0
    advance(node, 10.0)
    node.tick()
    assert node.state == 'FAILED'
    assert 'dropped' in node.failure_reason


def test_bin_detection_failure_does_not_fall_back_to_a_nominal_pose():
    node = make()
    node.state = 'DETECT_BIN'
    node.entered = node._clock.now()
    for _ in range(6):
        advance(node, 50.0)
        node.tick()
        if node.state == 'FAILED':
            break
    assert node.state == 'FAILED', (
        'a failed visual identification silently used a hard-coded bin pose')
    assert node.bin_point is None
