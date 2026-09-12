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
    assert node.state == 'FACE_BIN'
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
    advance(node, node.IMAGE_GRACE + 1.0)   # give the path its moment
    node.tick()
    assert 'column_topic' in node.score.awarded
    assert 'column_image' not in node.score.awarded


def test_the_column_image_is_credited_when_it_arrives_a_frame_late():
    """All five trials of the first batch forfeited these 2 points.

    The slot number arrives on a latched topic and the evidence path on the
    status topic.  The state advanced on the slot alone, so every run logged
    "no annotated column image was recorded" while the file sat on disk, and
    every score estimate came out 2 points light.
    """
    node = make()
    node.state = 'DETECT_COLUMN'
    node.entered = node._clock.now()
    node.column_slot = 3                 # latched topic wins the race
    node.tick()
    assert node.state == 'DETECT_COLUMN', (
        'advanced before the evidence path could arrive')

    node.on_perception(Msg(data='column:done:3:/tmp/shelf_column_late.png'))
    node.tick()
    assert 'column_image' in node.score.awarded
    assert node.evidence['column_image'] == '/tmp/shelf_column_late.png'


def test_a_column_image_that_never_arrives_does_not_stall_the_run():
    """The grace period is a courtesy to a slow topic, not a dependency."""
    node = make()
    node.state = 'DETECT_COLUMN'
    node.entered = node._clock.now()
    node.column_slot = 3
    node.tick()
    assert node.state == 'DETECT_COLUMN'
    advance(node, node.IMAGE_GRACE + 1.0)
    node.tick()
    assert node.state == 'NAV_COLUMN'


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


def _point(x, y, z):
    msg = PointStamped()
    msg.point.x, msg.point.y, msg.point.z = x, y, z
    return msg


def test_the_record_states_the_offset_against_the_plate_it_read():
    """The nominal comparison reports drift, not offset.

    A book really 0.25 m off its column centre was recorded at -0.703 m,
    because the nominal centre is not where the robot thinks it is.  The
    report headlines this number as "mean offset of book from column centre",
    where 0.70 m would read as having grabbed the wrong column's book.
    """
    node = make()
    node.state = 'DETECT_BOOK'
    node.column_slot = 2                     # nominal centre y = +1.00
    node.on_perception(Msg(data='column_ref:done:0.552'))
    node.on_target_point(_point(3.0, 0.298, 1.25))

    record = node._record('failed')
    assert record['detected']['book_offset_from_column_centre'] == pytest.approx(
        -0.254, abs=1e-3), 'offset measured against the nominal centre again'
    assert 'plate' in record['detected']['book_offset_measured_against']


def test_a_failed_re_look_does_not_un_measure_the_book():
    """T01 of the final batch recorded no book position at all.

    The book was measured, published and photographed; a later look after two
    failed grasps could not find it again - by then the arm had knocked it out
    of the shelf - and the record came out empty, understating what the run
    had actually achieved.
    """
    node = make()
    node.state = 'DETECT_BOOK'
    node.on_target_point(_point(2.9, -0.6, 1.58))
    node.book_point = None                   # the re-look found nothing
    got = node._record('failed')['detected']['book_point_world']
    assert got == (2.9, -0.6, 1.58)


def test_a_failed_re_look_does_not_erase_the_evidence_image():
    """Two records named no book image while the scored file sat on disk."""
    node = make()
    node.state = 'DETECT_BOOK'
    node.entered = node._clock.now()
    node.book_point = (2.9, -0.6, 1.58)
    node.on_perception(Msg(data='book:done:1:/tmp/target_book_1.png'))
    node.tick()
    assert 'row_image' in node.score.awarded

    node.enter_detect_book()                 # re-observe after a failed grasp
    assert node.evidence.get('book_image') == '/tmp/target_book_1.png'
    assert node._record('failed')['evidence']['book_image']


def test_the_staging_re_measure_is_not_thrown_away():
    """REFRESH_BOOK asked perception for a fresh point and then ignored it.

    on_target_point only accepted a book point in DETECT_BOOK, so the staging
    re-measurement arrived, was dropped, and the state timed out onto the stale
    point on every run - the re-measure had never once taken effect.
    """
    node = make()
    node.state = 'REFRESH_BOOK'
    node.on_target_point(_point(2.90, -0.15, 1.58))
    assert node.book_point == (2.90, -0.15, 1.58)
    assert node.book_point_best == (2.90, -0.15, 1.58)


def test_the_bin_is_found_before_the_shelf():
    """Measuring the bin first lets the return drive straight to it."""
    node = make()
    node.state = 'DETECT_BIN'
    node.bin_early = True
    node.entered = node._clock.now()
    node.bin_point = (-0.975, 0.02, 0.95)
    node.tick()
    assert node.state == 'FACE_SHELF'
    assert node.bin_point == (-0.975, 0.02, 0.95)


def test_an_early_bin_search_that_fails_is_not_fatal():
    """Before the grasp, a missed bin is searched for again later."""
    node = make()
    node.state = 'DETECT_BIN'
    node.bin_early = True
    node.entered = node._clock.now()
    for _ in range(6):
        advance(node, 50.0)
        node.tick()
        if node.state != 'DETECT_BIN':
            break
    assert node.state == 'FACE_SHELF'
    assert node.bin_point is None


HELD_HIGH = 'held=book_col_2_row_1_red tool=0.600,0.170,1.660 torso=0.350'


def test_a_book_held_high_is_carried_straight_to_the_bin():
    """The grip lasts 10-20 s; the direct drive-and-drop fits inside it."""
    node = make()
    node.column_slot = 2
    node.bin_point = (-0.975, 0.0, 0.95)
    node.state = 'CONFIRM_HOLD'
    node.entered = node._clock.now()
    node.tick()
    reply(node, ok=True, detail=HELD_HIGH)
    node.tick()
    assert node.state == 'CARRY_TO_BIN'
    x, y, torso = node.carry_goal
    # Aimed CARRY_NEAR_OFFSET short of the bin's middle, towards the robot,
    # then clipped to stay inside the bin's near wall.
    aim = min(-0.975 + node.CARRY_NEAR_OFFSET, -0.695 - node.CARRY_BIN_MARGIN)
    assert abs(x - (aim + 0.600)) < 1e-9, x
    assert abs(y - 0.170) < 1e-9
    drop = min(1.660 - node.CARRY_RELEASE_TOOL_Z, node.CARRY_MAX_TORSO_DROP)
    assert abs(torso - (0.350 - drop)) < 1e-9, torso
    assert sent_verbs(node)[-1] == 'torso'
    assert 'grasp' in node.score.awarded


def test_a_book_held_too_low_for_the_rim_takes_the_long_way():
    node = make()
    node.column_slot = 2
    node.bin_point = (-0.975, 0.0, 0.95)
    node.state = 'CONFIRM_HOLD'
    node.entered = node._clock.now()
    node.tick()
    reply(node, ok=True, detail='held=book tool=0.600,0.170,0.900 torso=0.350')
    node.tick()
    assert node.state == 'NAV_BIN_AREA'


def test_the_direct_carry_releases_only_once_the_torso_has_stopped():
    """The manipulation node refuses a second command while busy."""
    node = make()
    node.carry_goal = (-0.375, 0.17, 0.14)
    node.transition('CARRY_TO_BIN')
    node.hold_width = 0.024
    node.nav_status = 'reached'
    node.tick()
    assert node.state == 'CARRY_TO_BIN'
    reply(node, ok=True, detail='')
    node.tick()
    assert node.state == 'RELEASE'


def test_losing_the_book_on_the_direct_carry_fails_the_run():
    node = make()
    node.carry_goal = (-0.375, 0.17, None)
    node.transition('CARRY_TO_BIN')
    node.hold_width = 0.0
    advance(node, 5.0)
    node.tick()
    assert node.state == 'FAILED'
    assert 'dropped' in node.failure_reason


def test_backing_off_clears_the_table_before_stowing():
    """After the direct carry the gripper is over the bin; stowing there
    would swing the arm through it."""
    node = make()
    node.carry_goal = (-0.375, 0.17, 0.14)
    node.transition('BACK_OFF')
    assert 'stow' not in sent_verbs(node)
    node.nav_status = 'reached'
    node.tick()
    assert sent_verbs(node)[-1] == 'stow'
    reply(node, ok=True)
    node.tick()
    assert node.state == 'DONE'


def test_a_stow_failure_after_delivery_does_not_fail_the_run():
    node = make()
    node.carry_goal = None
    node.transition('BACK_OFF')
    assert sent_verbs(node)[-1] == 'stow'
    reply(node, ok=False, detail='motion finished off target')
    node.tick()
    assert node.state == 'DONE'


def test_the_direct_carry_never_drives_the_torso_into_the_table():
    """A bin measured deep in the table would put the base under its edge;
    the base stops CARRY_TABLE_CLEARANCE short and the gripper still ends up
    inside the bin."""
    node = make()
    node.bin_point = (-1.155, -0.08, 0.95)
    x, y, torso = node._direct_carry(HELD_HIGH)
    assert x >= -0.60 + node.CARRY_TABLE_CLEARANCE - 1e-9
    assert -1.255 + node.CARRY_BIN_MARGIN <= x - 0.600 <= -0.695 - node.CARRY_BIN_MARGIN


def test_a_bin_out_of_reach_from_the_table_edge_takes_the_long_way():
    node = make()
    node.bin_point = (-1.155, -0.08, 0.95)
    short_arm = 'held=book tool=0.200,0.170,1.660 torso=0.350'
    assert node._direct_carry(short_arm) is None


def _carrying(node, bin_point=(-0.975, 0.0, 0.95)):
    node.bin_point = bin_point
    node.carry_offset = (0.600, 0.170)
    aim = node._carry_base(bin_point, 0.600, 0.170)
    node.carry_goal = (aim[0], aim[1], 0.20)
    node.transition('CARRY_TO_BIN')
    node.hold_width = 0.024
    return node


def test_the_bin_is_re_measured_on_the_way_in():
    """The bin was measured before the shelf trip; about 5 m of driving of
    odometry drift stands between that and the release, against a bin only
    0.155 m wide either side."""
    node = _carrying(make())
    node.on_target_point(_point(-0.975, 0.12, 0.95))
    node.tick()
    assert abs(node.carry_goal[1] - (0.12 + 0.170)) < 1e-9, node.carry_goal
    assert node._carry_reaims == 1
    assert node.state == 'CARRY_TO_BIN'


def test_a_small_bin_correction_is_not_chased():
    """Re-publishing the goal restarts the ramp, so only a real shift is
    worth it."""
    node = _carrying(make())
    node.on_target_point(_point(-0.975, 0.01, 0.95))
    node.tick()
    assert node.carry_goal[1] == 0.170
    assert node._carry_reaims == 0
