"""Mission node - the high-level state machine.

Drives the whole run: read the shelf number plates, go to the right column,
find the target-colour book, measure where it actually is, pick it up, carry it
back and place it in the collection bin.

The single most important property of this file is what it does **not** do: no
state advances because a timer expired.  Earlier versions of this node treated
a timeout as completion in sixteen places, including the last one, so a run in
which nothing worked at all still ended in ``DONE`` and logged "MISSION
COMPLETE - book delivered to the collection bin".  Every transition here is
driven by an observation:

* command acceptance, motion completion and grasp verification are three
  separate things, reported separately by ``manipulation_node``;
* a grasp is confirmed by the pads stalling at a book-shaped width, not by the
  close command having been sent;
* delivery is confirmed on the organiser's ``/bin_contacts`` interface, and
  only for a contact between the bin and a *book* - the bin rests on the table,
  so that pair is published at 200 Hz from the moment the world loads and
  would otherwise report success before the robot has moved;
* every reply carries the sequence number of the command it answers, so an
  acknowledgement from an earlier state, or an earlier run, cannot advance
  anything.

A state that runs out of time either retries, bounded, or fails the run.
``FAILED`` is a real outcome and is reported as one.
"""

import json
import math
import os
import time
import uuid

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, Empty, Float32, Int32, String

from . import arena, scoring

#: Head pan/tilt pairs tried in order when the marker scan has to retry.
SWEEP = [(0.0, 0.32), (0.0, 0.20), (-0.35, 0.30), (0.35, 0.30), (0.0, 0.05)]

#: Head tilts tried in order when the book scan has to retry.  These sweep
#: **both ways** from the nominal.  An earlier version only ever tilted further
#: down, which is exactly the wrong direction when the target colour happens to
#: be in the top row - and with the colours shuffled per column, one run in
#: four puts it there.  Measured: the top row sits near the upper frame edge at
#: the nominal -0.30 tilt, so a small upward correction is what recovers it.
BOOK_TILTS = [-0.30, -0.12, 0.02, -0.45, -0.20]

LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class MissionNode(Node):

    def __init__(self):
        super().__init__('rafeeq_mission')

        self.declare_parameter('shelf_column_number', 0)
        self.declare_parameter('book_colour', '')
        self.declare_parameter('bin_standoff', 0.55)
        # Must comfortably exceed perception's tf_settle_seconds, or a
# detection is abandoned before the transforms have caught up.
        self.declare_parameter('detect_timeout', 18.0)
        self.declare_parameter('nav_timeout', 90.0)
        self.declare_parameter('manip_timeout', 150.0)
        self.declare_parameter('ready_timeout', 180.0)
        self.declare_parameter('results_dir', '')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('run_id', '')

        self.target_column = int(
            self.get_parameter('shelf_column_number').value)
        self.target_colour = str(
            self.get_parameter('book_colour').value).strip().lower()
        self.bin_standoff = float(self.get_parameter('bin_standoff').value)

        self.timeouts = {
            'detect': float(self.get_parameter('detect_timeout').value),
            'nav': float(self.get_parameter('nav_timeout').value),
            'manip': float(self.get_parameter('manip_timeout').value),
            'ready': float(self.get_parameter('ready_timeout').value),
        }

        self.run_id = (str(self.get_parameter('run_id').value)
                       or uuid.uuid4().hex[:8])

        # --- publishers ---------------------------------------------------
        self.pub_goal = self.create_publisher(
            PoseStamped, '/rafeeq/nav/goal', 10)
        self.pub_cancel = self.create_publisher(
            Empty, '/rafeeq/nav/cancel', 10)
        self.pub_mode = self.create_publisher(
            String, '/rafeeq/perception/mode', 10)
        self.pub_manip = self.create_publisher(
            String, '/rafeeq/manip/command', 10)
        self.pub_state = self.create_publisher(
            String, '/rafeeq/mission/state', LATCHED)
        self.pub_outcome = self.create_publisher(
            String, '/rafeeq/mission/outcome', LATCHED)

        # --- subscriptions ------------------------------------------------
        self.nav_status = 'idle'
        self.nav_seq = 0
        self.manip_seq = 0
        self.manip_reply = None
        self.manip_ready = False
        self.perception_status = ''
        self.hold_width = 0.0

        self.create_subscription(
            String, '/rafeeq/nav/status', self.on_nav, 10)
        self.create_subscription(
            String, '/rafeeq/manip/status', self.on_manip, 10)
        self.create_subscription(
            Bool, '/rafeeq/manip/ready', self.on_manip_ready, 10)
        self.create_subscription(
            Float32, '/rafeeq/manip/hold', self.on_hold, 10)
        self.create_subscription(
            String, '/rafeeq/perception/status', self.on_perception, 10)
        self.create_subscription(
            PointStamped, '/rafeeq/perception/target_point',
            self.on_target_point, 10)
        self.create_subscription(
            Int32, '/rafeeq/perception/column_slot', self.on_slot, LATCHED)
        self.create_subscription(
            String, '/rafeeq/mission/start', self.on_start, 10)

        # Official interfaces.
        self.create_subscription(
            Contacts, '/bin_contacts', self.on_bin_contacts, 10)
        self.create_subscription(
            Contacts, '/contacts', self.on_contacts, 10)

        # --- scoring and evidence ----------------------------------------
        self.collisions = scoring.CollisionMonitor()
        self.delivery = scoring.DeliveryMonitor(self.target_colour)
        self.score = scoring.ScoreCard(self.collisions)

        # --- run state ----------------------------------------------------
        self.column_slot = None
        self.column_point = None        # measured world XYZ of the plate
        self.book_row = None
        self.book_point = None          # measured world XYZ of the book
        self.bin_point = None           # measured world XYZ of the bin
        self.plan_poses = {}
        self.attempt = 0
        self.grasp_attempts = 0
        self._recover_driving = False
        self.state = None
        self.entered = self.get_clock().now()
        self.started = None
        self.stage_times = {}
        self.failure_reason = ''
        self.evidence = {}
        #: When the column slot first arrived, so the evidence path that
        #: follows it on a different topic gets a moment to catch up.
        self._slot_seen_at = None

        self.invalid = self._validate_inputs()

        self.create_timer(0.2, self.tick)

        if self.invalid:
            self.get_logger().error(self.invalid)
            self.transition('FAILED')
            return

        self.get_logger().info(
            '\n' + '=' * 66 + '\n'
            'RAFEEQ LIBRARY ROBOT - Team Kanban - ERC 2026 Phase 1\n'
            'run {}: shelf column {}, {} book\n'.format(
                self.run_id, self.target_column, self.target_colour)
            + '=' * 66)

        if self.get_parameter('auto_start').value:
            self.transition('WAIT_READY')
        else:
            self.transition('IDLE')

    # -- validation ------------------------------------------------------

    def _validate_inputs(self):
        """Reject bad arguments before anything is commanded to move."""
        problems = []
        if self.target_column not in range(1, arena.NUM_COLUMNS + 1):
            problems.append(
                'shelf_column_number must be an integer 1-{}, got {!r}'.format(
                    arena.NUM_COLUMNS,
                    self.get_parameter('shelf_column_number').value))
        if self.target_colour not in arena.BOOK_COLOURS:
            problems.append(
                'book_colour must be one of {}, got {!r}'.format(
                    ', '.join(arena.BOOK_COLOURS),
                    self.get_parameter('book_colour').value))
        if not problems:
            return ''
        return ('invalid launch arguments:\n  - ' + '\n  - '.join(problems)
                + '\n\nExample:\n  ros2 launch rafeeq_solution '
                  'solution.launch.py shelf_column_number:=2 book_colour:=red')

    # -- subscriptions ---------------------------------------------------

    def on_nav(self, msg):
        self.nav_status = msg.data

    def on_manip(self, msg):
        parts = msg.data.split('|')
        if len(parts) < 3:
            return
        kind, seq = parts[0], parts[1]
        try:
            seq = int(seq)
        except ValueError:
            return
        if seq != self.manip_seq:
            # An answer to a command we are no longer waiting on.
            return
        self.manip_reply = (kind, parts[2], '|'.join(parts[3:]))

    def on_manip_ready(self, msg):
        self.manip_ready = bool(msg.data)

    def on_hold(self, msg):
        self.hold_width = float(msg.data)

    def on_perception(self, msg):
        if (msg.data != self.perception_status
                and ':searching:' in msg.data):
            self.get_logger().info(
                'perception: {}'.format(msg.data),
                throttle_duration_sec=4.0)
        self.perception_status = msg.data
        # Capture the evidence path the moment it is announced.  The slot also
        # arrives on a latched topic, so a state can advance on that before the
        # status message is read - and the image then never gets credited.
        parts = msg.data.split(':')
        if msg.data.startswith('column:done:'):
            self.column_slot = int(parts[2])
            if len(parts) > 3 and parts[3]:
                self.evidence['column_image'] = ':'.join(parts[3:])
        elif msg.data.startswith('book:done:'):
            self.book_row = int(parts[2])
            if len(parts) > 3 and parts[3]:
                self.evidence['book_image'] = ':'.join(parts[3:])

    def on_slot(self, msg):
        self.column_slot = int(msg.data)

    def on_target_point(self, msg):
        point = (msg.point.x, msg.point.y, msg.point.z)
        if self.state in ('DETECT_COLUMN',):
            self.column_point = point
        elif self.state in ('DETECT_BOOK',):
            self.book_point = point
        elif self.state in ('DETECT_BIN',):
            self.bin_point = point

    def on_bin_contacts(self, msg):
        if self.delivery.ingest(msg, self.now()):
            self.get_logger().info(
                'OFFICIAL COMPLETION EVENT: {} contacted the collection bin '
                'at t+{:.1f} s'.format(self.delivery.first_contact_model,
                                       self.elapsed_total))

    def on_contacts(self, msg):
        manipulating = self.state in (
            'CLOSE_GRASP', 'LIFT', 'REFINE', 'RETREAT', 'CARRY',
            'PLACE', 'RELEASE', 'BACK_OFF')
        self.collisions.manipulating = manipulating
        for event in self.collisions.ingest(msg, self.now(), self.state or ''):
            self.get_logger().warn(
                'collision #{}: {} hit {} during {} (-{:.1f})'.format(
                    self.collisions.count, event['link'], event['object'],
                    event['state'], scoring.COLLISION_PENALTY))

    def on_start(self, _msg):
        if self.state in (None, 'IDLE', 'DONE', 'FAILED'):
            self.transition('WAIT_READY')

    # -- plumbing --------------------------------------------------------

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @property
    def elapsed(self):
        return (self.get_clock().now() - self.entered).nanoseconds / 1e9

    @property
    def elapsed_total(self):
        if self.started is None:
            return 0.0
        return self.now() - self.started

    def transition(self, state):
        if self.state is not None and self.state not in ('IDLE',):
            self.stage_times[self.state] = round(
                self.stage_times.get(self.state, 0.0) + self.elapsed, 2)
        self.state = state
        self.entered = self.get_clock().now()
        self.attempt = 0
        self.pub_state.publish(String(data=state))
        self.get_logger().info('[STATE] {}  (t+{:.1f} s)'.format(
            state, self.elapsed_total))

        handler = getattr(self, 'enter_{}'.format(state.lower()), None)
        if handler is not None:
            try:
                handler()
            except Exception as exc:
                self.fail('{} failed to start: {}'.format(state, exc))

    def tick(self):
        if self.state in (None, 'IDLE', 'DONE', 'FAILED'):
            return
        handler = getattr(self, 'check_{}'.format(self.state.lower()), None)
        if handler is None:
            return
        try:
            nxt = handler()
        except Exception as exc:
            self.fail('{} raised {}: {}'.format(
                self.state, type(exc).__name__, exc))
            return
        if nxt is not None and nxt != self.state:
            self.transition(nxt)

    def fail(self, reason):
        self.failure_reason = reason
        self.get_logger().error('MISSION FAILED in {}: {}'.format(
            self.state, reason))
        self.pub_cancel.publish(Empty())
        self.set_mode('idle')
        self.transition('FAILED')
        return None

    # -- command helpers -------------------------------------------------

    def set_mode(self, mode):
        self.perception_status = ''
        self.pub_mode.publish(String(data=mode))

    def manip(self, verb, *args):
        """Send a manipulation command and start waiting for *its* reply."""
        self.manip_seq += 1
        self.manip_reply = None
        payload = '|'.join([str(self.manip_seq), verb] + [str(a) for a in args])
        self.pub_manip.publish(String(data=payload))
        return self.manip_seq

    def manip_result(self):
        """``(ok, detail)`` once the current command has answered, else None."""
        if self.manip_reply is None:
            return None
        kind, verb, detail = self.manip_reply
        if kind == 'busy':
            return None
        return (kind == 'done', detail)

    def manip_gate(self, on_success, retry=None):
        """Standard wait: succeed, fail, or time out - never 'assume done'."""
        result = self.manip_result()
        if result is not None:
            ok, detail = result
            if ok:
                if detail:
                    self.get_logger().info('  {}'.format(detail))
                return on_success
            if retry is not None:
                return retry(detail)
            return self.fail(detail)
        if self.elapsed > self.timeouts['manip']:
            return self.fail(
                'manipulation did not answer within {:.0f} s'.format(
                    self.timeouts['manip']))
        return None

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

    def nav_gate(self, on_success, what):
        """Wait for the base to actually arrive.

        ``blocked`` is *not* arrival.  An earlier version treated it as one,
        which meant a run could 'reach' a grasp pose it had never got to.
        """
        if self.nav_status == 'reached':
            return on_success
        if self.nav_status == 'failed':
            return self.fail('navigation failed while {}'.format(what))
        if self.nav_status == 'blocked' and self.elapsed > 15.0:
            return self.fail(
                'the path is blocked while {} and did not clear'.format(what))
        if self.elapsed > self.timeouts['nav']:
            return self.fail('timed out while {}'.format(what))
        return None

    # -- states ----------------------------------------------------------

    def enter_wait_ready(self):
        self.started = None
        self.get_logger().info(
            'waiting for controllers, joint feedback and odometry '
            '(no fixed sleep - readiness is observed)')

    def check_wait_ready(self):
        # Do not start the trial clock until the simulation clock is real.
        # With use_sim_time the node's clock reads 0 until the first /clock
        # message arrives, and it then jumps to whatever the simulator is
        # already at.  Latching the start time before that makes the very
        # first elapsed reading enormous, and every timeout fires at once -
        # observed as "controllers never became ready" 0.06 s after start.
        now = self.now()
        if now <= 0.0:
            return None
        if self.started is None:
            self.started = now
            self.entered = self.get_clock().now()
            self.get_logger().info(
                'simulation clock is live at t={:.1f} s; trial clock '
                'started'.format(now))
            return None

        if self.manip_ready:
            self.get_logger().info(
                'everything reported ready after {:.1f} s'.format(
                    self.elapsed))
            return 'HOME'
        if self.elapsed > self.timeouts['ready']:
            return self.fail(
                'controllers never became ready; /rafeeq/manip/ready stayed '
                'false for {:.0f} s'.format(self.timeouts['ready']))
        return None

    def enter_home(self):
        self.manip('home')

    def check_home(self):
        return self.manip_gate('FACE_SHELF')

    def enter_face_shelf(self):
        # The robot spawns yawed +90 deg, side-on to the shelf.  Turn to face
        # world +x without leaving the start zone.
        self.goto(arena.START_XY[0], arena.START_XY[1], 0.0)

    def check_face_shelf(self):
        return self.nav_gate('SCAN_POSE', 'turning to face the shelf')

    def enter_scan_pose(self):
        self.manip('scan_markers')

    def check_scan_pose(self):
        return self.manip_gate('DETECT_COLUMN')

    def enter_detect_column(self):
        self.column_point = None
        self.evidence.pop('column_image', None)
        self.set_mode('column')

    #: How long to let the evidence path catch up with the slot number.
    IMAGE_GRACE = 3.0

    def check_detect_column(self):
        if self.column_slot is not None:
            # The slot number arrives on a latched topic and the evidence path
            # on the status topic, so the slot can win the race by a frame or
            # two.  Advancing on the slot alone forfeits the 2 points for the
            # column image - which is what happened on all five trials of the
            # first batch, every one of them logging "no annotated column image
            # was recorded" while the file sat on disk.
            if not self.evidence.get('column_image'):
                if self._slot_seen_at is None:
                    self._slot_seen_at = self.now()
                if self.now() - self._slot_seen_at < self.IMAGE_GRACE:
                    return None

            self.get_logger().info(
                'number plate {} identified above physical column {} '
                '(world y = {:+.2f} m)'.format(
                    self.target_column, self.column_slot,
                    arena.COLUMN_Y[self.column_slot]))
            self.score.award(
                'column_topic',
                'published {} on /erc/shelf_column_identification after the '
                'marker was read'.format(self.target_column))
            image = self.evidence.get('column_image')
            if image:
                self.score.award('column_image', image)
            else:
                self.get_logger().warn(
                    'no annotated column image was recorded within {:.0f} s of '
                    'the slot number; the 2 points for it are not claimed'
                    .format(self.IMAGE_GRACE))
            return 'NAV_COLUMN'

        if self.elapsed <= self.timeouts['detect']:
            return None

        self.attempt += 1
        if self.attempt >= len(SWEEP):
            return self.fail(
                'never identified number plate {} (last perception status: '
                '{!r})'.format(self.target_column, self.perception_status))
        pan, tilt = SWEEP[self.attempt]
        self.get_logger().warn(
            'plate not found, retry {}/{} with head pan={:+.2f} tilt={:+.2f}'
            .format(self.attempt, len(SWEEP) - 1, pan, tilt))
        self.manip('head', '{},{}'.format(pan, tilt))
        self.entered = self.get_clock().now()
        self.set_mode('column')
        return None

    @property
    def column_y(self):
        return arena.COLUMN_Y.get(self.column_slot, 0.0)

    def enter_nav_column(self):
        """Drive in front of the plate we actually measured.

        Not the nominal centre of the slot it snapped to.  The mecanum base's
        odometry drifts by tenths of a metre, so a nominal world waypoint puts
        the robot somewhere else entirely - but the measurement and the drive
        share that same drifting frame, so aiming at the measurement lands the
        robot physically in front of the right column.

        The standoff is the book-scan distance, not the grasp distance: all
        four rows span 1.24 m and the camera only covers that from about
        1.17 m back, so driving straight up would leave rows out of frame.
        """
        if self.column_point is not None:
            target_y = self.column_point[1]
        else:
            target_y = self.column_y
            self.get_logger().warn(
                'no measured plate position; falling back to the nominal '
                'centre of column slot {}'.format(self.column_slot))
        self.goto(arena.BOOK_X - arena.BOOK_SCAN_STANDOFF, target_y, 0.0)

    def check_nav_column(self):
        return self.nav_gate(
            'COLUMN_REF_POSE',
            'driving to column {}'.format(self.column_slot))

    def enter_column_ref_pose(self):
        """Look back up at the marker plates from where the books will be seen.

        Costs one torso move and a head tilt.  It buys the book filter a
        reference measured from this pose, which is the only way to tell our
        column from its neighbour once odometry has drifted - see
        ``perception_node._do_book``.
        """
        self.manip('scan_markers')

    def check_column_ref_pose(self):
        return self.manip_gate('COLUMN_REF')

    def enter_column_ref(self):
        self.set_mode('column_ref')

    def check_column_ref(self):
        if (self.perception_status or '').startswith('column_ref:done:'):
            return 'BOOK_POSE'

        if self.elapsed <= self.timeouts['detect']:
            return None

        # The column itself was already identified and published from the
        # start zone, so this is a lost reference rather than a lost column.
        # Going on means the book filter falls back to the image centre, which
        # drift can get wrong - so say that plainly rather than bury it.
        self.get_logger().warn(
            'could not re-read plate {} from the book-scan pose (perception: '
            '{!r}); continuing without a drift-matched column reference - the '
            'book filter will fall back to the image centre'.format(
                self.target_column, self.perception_status))
        return 'BOOK_POSE'

    def enter_book_pose(self):
        self.manip('scan_books')

    def check_book_pose(self):
        return self.manip_gate('DETECT_BOOK')

    def enter_detect_book(self):
        self.book_point = None
        self.evidence.pop('book_image', None)
        self.set_mode('book')

    def check_detect_book(self):
        if self.book_row is not None and self.book_point is not None:
            self.get_logger().info(
                '{} book identified in row {}, measured at world '
                '({:+.3f}, {:+.3f}, {:.3f}) - {:+.3f} m from the column '
                'centre'.format(
                    self.target_colour, self.book_row, *self.book_point,
                    self.book_point[1] - self.column_y))
            self.score.award(
                'row_topic',
                'published row {} on /erc/shelf_row_identification'.format(
                    self.book_row))
            image = self.evidence.get('book_image')
            if image:
                self.score.award('row_image', image)
            else:
                self.get_logger().warn(
                    'no annotated book image was recorded; the 2 points for '
                    'it are not claimed')
            return 'NAV_STAGE'

        if self.elapsed <= self.timeouts['detect']:
            return None

        self.attempt += 1
        if self.attempt >= len(BOOK_TILTS):
            return self.fail(
                'never located a {} book in column {} (last perception '
                'status: {!r})'.format(
                    self.target_colour, self.column_slot,
                    self.perception_status))
        tilt = BOOK_TILTS[min(self.attempt, len(BOOK_TILTS) - 1)]
        self.get_logger().warn(
            'book not found, retry {}/{} with head tilt {:+.2f}'.format(
                self.attempt, len(BOOK_TILTS) - 1, tilt))
        self.manip('head', '0.0,{}'.format(tilt))
        self.entered = self.get_clock().now()
        self.set_mode('book')
        return None

    def enter_nav_stage(self):
        """Park square to the measured book, far enough back to raise the arm.

        Strafing to the book's own y matters: it can be up to 0.25 m off the
        column centre and the pads only open to a 0.052 m gap.  Standing this
        far back matters too - it is where the arm takes its grasp posture, and
        the arm must not be able to reach the shelf while it swings there.
        """
        self.goto(self.book_point[0] - arena.ARM_TRANSIT_STANDOFF,
                  self.book_point[1], 0.0)

    def check_nav_stage(self):
        return self.nav_gate('PLAN_GRASP', 'moving to the staging pose')

    def enter_plan_grasp(self):
        advance = arena.ARM_TRANSIT_STANDOFF - arena.GRASP_STANDOFF
        self.manip('plan_grasp',
                   '{:.4f},{:.4f},{:.4f}'.format(*self.book_point),
                   self.column_slot if self.column_slot else 'none',
                   '{:.4f}'.format(advance))

    def check_plan_grasp(self):
        def on_fail(detail):
            self.grasp_attempts += 1
            if self.grasp_attempts >= 2:
                return self.fail('grasp planning failed: {}'.format(detail))
            self.get_logger().warn(
                'grasp planning failed ({}); re-observing the book'.format(
                    detail))
            return 'DETECT_BOOK'

        result = self.manip_result()
        if result is not None and result[0]:
            for item in result[1].split():
                if '=' in item:
                    key, value = item.split('=', 1)
                    self.plan_poses[key] = value
            self.get_logger().info(
                'grasp planned: roll {}, {} m up the spine, IK error {} m'
                .format(self.plan_poses.get('roll', '?'),
                        self.plan_poses.get('spine_offset', '?'),
                        self.plan_poses.get('ik_error', '?')))
            return 'ARM_PREGRASP'
        return self.manip_gate('ARM_PREGRASP', retry=on_fail)

    def enter_arm_pregrasp(self):
        self.manip('arm_pregrasp')

    def check_arm_pregrasp(self):
        return self.manip_gate('NAV_GRASP')

    def _pose_from_plan(self, key):
        raw = self.plan_poses.get(key)
        if not raw:
            raise RuntimeError('grasp plan did not provide a {} pose'.format(
                key))
        return [float(v) for v in raw.split(',')]

    def enter_nav_grasp(self):
        x, y, yaw = self._pose_from_plan('grasp')
        self.goto(x, y, yaw)

    def check_nav_grasp(self):
        return self.nav_gate('REFINE', 'advancing onto the book')

    def enter_refine(self):
        self.manip('refine_grasp')

    def check_refine(self):
        return self.manip_gate('CLOSE_GRASP')

    def enter_close_grasp(self):
        self.manip('close_grasp')

    def check_close_grasp(self):
        def on_fail(detail):
            self.grasp_attempts += 1
            if self.grasp_attempts >= 3:
                return self.fail('grasp failed: {}'.format(detail))
            self.get_logger().warn(
                'grasp attempt {} failed ({}); backing off and re-observing'
                .format(self.grasp_attempts, detail))
            return 'RECOVER'
        return self.manip_gate('LIFT', retry=on_fail)

    def enter_recover(self):
        """Back the base out and reopen the gripper before trying again."""
        self.manip('gripper', 'open')
        self._recover_driving = False

    def check_recover(self):
        """Open first, then drive clear - one goal, sent once."""
        if not self._recover_driving:
            result = self.manip_result()
            if result is None:
                if self.elapsed > self.timeouts['manip']:
                    return self.fail(
                        'the gripper did not reopen for a retry')
                return None
            # Whether or not the reopen succeeded, get clear of the shelf.
            self._recover_driving = True
            try:
                goal = self._pose_from_plan('retreat')
            except RuntimeError:
                goal = (self.book_point[0] - arena.ARM_TRANSIT_STANDOFF,
                        self.book_point[1], 0.0)
            self.entered = self.get_clock().now()
            self.goto(*goal)
            return None

        return self.nav_gate(
            'COLUMN_REF_POSE', 'backing away to retry the grasp')

    def enter_lift(self):
        self.manip('lift')

    def check_lift(self):
        return self.manip_gate('RETREAT')

    def enter_retreat(self):
        x, y, yaw = self._pose_from_plan('retreat')
        self.goto(x, y, yaw)

    def check_retreat(self):
        return self.nav_gate('CONFIRM_HOLD', 'pulling the book off the shelf')

    def enter_confirm_hold(self):
        self.manip('check_hold')

    def check_confirm_hold(self):
        result = self.manip_result()
        if result is not None:
            ok, detail = result
            if not ok:
                return self.fail(
                    'the book was lost during extraction: {}'.format(detail))
            # The book is out of the shelf and still held.  Both the grasp
            # points and the shelf-navigation points are earned here: the
            # rubric awards navigation on a successful grasp, and this is the
            # first moment we can prove one.
            self.score.award(
                'grasp', 'pads stalled at {} and the book stayed held through '
                         'extraction'.format(detail or 'a book-shaped width'))
            self.score.award(
                'navigate_shelf',
                'grasp succeeded at column {}'.format(self.column_slot))
            self.get_logger().info(
                'BOOK ACQUIRED: {} book from column {} row {} is held and '
                'clear of the shelf'.format(
                    self.target_colour, self.column_slot, self.book_row))
            return 'NAV_BIN_AREA'
        if self.elapsed > self.timeouts['manip']:
            return self.fail('retention check did not answer')
        return None

    def enter_nav_bin_area(self):
        # Face the bin from the start zone so the camera is already pointing
        # at it when the search begins.
        self.goto(arena.START_XY[0], arena.START_XY[1], math.pi)

    def check_nav_bin_area(self):
        if self.hold_width <= 0.0 and self.elapsed > 5.0:
            return self.fail(
                'the book was dropped on the way back to the bin')
        return self.nav_gate('BIN_POSE', 'returning to the collection bin')

    def enter_bin_pose(self):
        self.manip('head', 'scan_bin')

    def check_bin_pose(self):
        return self.manip_gate('DETECT_BIN')

    def enter_detect_bin(self):
        self.bin_point = None
        self.set_mode('bin')

    def check_detect_bin(self):
        if self.bin_point is not None:
            self.get_logger().info(
                'collection bin identified at world ({:+.3f}, {:+.3f}, '
                '{:.3f})'.format(*self.bin_point))
            return 'NAV_BIN'
        if self.elapsed <= self.timeouts['detect']:
            return None

        self.attempt += 1
        if self.attempt > 3:
            # Deliberately a failure, not a silent fallback to a nominal pose.
            # Replacing a failed visual identification with a hard-coded
            # coordinate and calling the vision task complete is exactly the
            # kind of false success this node exists to avoid.
            return self.fail(
                'never identified the collection bin visually (last status: '
                '{!r})'.format(self.perception_status))
        tilt = -0.10 - 0.10 * self.attempt
        self.get_logger().warn(
            'bin not found, retry {}/3 with head tilt {:+.2f}'.format(
                self.attempt, tilt))
        self.manip('head', '0.0,{}'.format(tilt))
        self.entered = self.get_clock().now()
        self.set_mode('bin')
        return None

    def enter_nav_bin(self):
        self.goto(self.bin_point[0] + self.bin_standoff,
                  self.bin_point[1], math.pi)

    def check_nav_bin(self):
        return self.nav_gate('PLACE', 'lining up on the bin')

    def enter_place(self):
        here = self.bin_point
        self.manip('place', '{:.4f},{:.4f},{:.4f}'.format(*here),
                   '0,0,{:.4f}'.format(math.pi))

    def check_place(self):
        return self.manip_gate('RELEASE')

    def enter_release(self):
        self._release_at = self.now()
        self.manip('release')

    def check_release(self):
        result = self.manip_result()
        if result is None:
            if self.elapsed > self.timeouts['manip']:
                return self.fail('release did not answer')
            return None
        if not result[0]:
            return self.fail('could not open the gripper: {}'.format(
                result[1]))
        return 'CONFIRM_DELIVERY'

    def check_confirm_delivery(self):
        """Delivery is the official contact, not the fact that we let go."""
        if self.delivery.delivered:
            self.score.award(
                'return_with_book',
                'book carried back and delivered; {} contacted the bin'.format(
                    self.delivery.first_contact_model))
            gentle, why = self._classify_placement()
            self.score.set_placement(gentle, why)
            return 'BACK_OFF'
        if self.elapsed > 20.0:
            return self.fail(
                'the gripper opened over the bin but nothing ever contacted '
                'it on /bin_contacts - the book did not reach the bin')
        return None

    def _classify_placement(self):
        """Drop or gentle placement, with the reasoning recorded.

        This is a judge's call in the end.  The rule used here is stated so it
        can be checked: a placement counts as gentle when the book was still
        held at the moment it was lowered to the release height, i.e. it was
        set down rather than let go from above.
        """
        release_height = float(self.plan_poses.get('release_z', 0.0) or 0.0)
        drop = release_height - arena.BIN_FLOOR_Z
        if release_height and drop <= arena.BOOK_SPINE_H / 2.0 + 0.05:
            return ('gentle',
                    'released at z={:.3f} m with the book lowered to within '
                    '{:.3f} m of the bin floor'.format(release_height, drop))
        return ('drop',
                'released at z={:.3f} m, {:.3f} m above the bin floor'.format(
                    release_height, drop))

    def enter_back_off(self):
        self.manip('stow')

    def check_back_off(self):
        return self.manip_gate('DONE')

    # -- terminal states -------------------------------------------------

    def enter_done(self):
        self._finish('completed')

    def enter_failed(self):
        self._finish('failed')

    def _finish(self, outcome):
        self.pub_cancel.publish(Empty())
        self.set_mode('idle')
        summary = self._record(outcome)
        self.get_logger().info(self.score.render())
        if outcome == 'completed':
            self.get_logger().info(
                '\n' + '=' * 66 + '\n'
                'RUN {} COMPLETE - {} book from column {} (physical slot {}), '
                'row {}\ndelivered to the collection bin in {:.1f} s of '
                'simulated time\n'.format(
                    self.run_id, self.target_colour, self.target_column,
                    self.column_slot, self.book_row, self.elapsed_total)
                + '=' * 66)
        else:
            self.get_logger().error(
                '\n' + '=' * 66 + '\n'
                'RUN {} FAILED after {:.1f} s: {}\n'.format(
                    self.run_id, self.elapsed_total,
                    self.failure_reason or self.invalid)
                + '=' * 66)
        self.pub_outcome.publish(String(data=json.dumps(summary)))

    def _record(self, outcome):
        """Write a machine-readable trial record, if a directory is set."""
        summary = {
            'run_id': self.run_id,
            'outcome': outcome,
            'failure_reason': self.failure_reason or self.invalid,
            'requested': {'shelf_column_number': self.target_column,
                          'book_colour': self.target_colour},
            'detected': {'physical_column_slot': self.column_slot,
                         'column_point_world': self.column_point,
                         'book_row': self.book_row,
                         'book_point_world': self.book_point,
                         'book_offset_from_column_centre':
                             (None if self.book_point is None
                              else round(self.book_point[1] - self.column_y, 4)),
                         'bin_point_world': self.bin_point},
            'evidence': dict(self.evidence),
            'grasp': {'attempts': self.grasp_attempts,
                      'plan': dict(self.plan_poses),
                      'held_width': self.hold_width},
            'delivery': {'delivered': self.delivery.delivered,
                         'model': self.delivery.first_contact_model,
                         'colour_matches': self.delivery.colour_matches,
                         'contact_time_s': (
                             None if self.delivery.first_contact_time is None
                             else round(self.delivery.first_contact_time
                                        - (self.started or 0.0), 2))},
            'timing': {'simulated_seconds_total': round(self.elapsed_total, 2),
                       'stage_seconds': dict(self.stage_times)},
            'score': self.score.summary(),
        }

        directory = str(self.get_parameter('results_dir').value)
        if directory:
            try:
                os.makedirs(directory, exist_ok=True)
                path = os.path.join(
                    directory, 'trial_{}.json'.format(self.run_id))
                with open(path, 'w') as handle:
                    json.dump(summary, handle, indent=2, sort_keys=True)
                self.get_logger().info('trial record -> {}'.format(path))
            except OSError as exc:
                self.get_logger().error(
                    'could not write the trial record: {}'.format(exc))
        return summary

    def shutdown(self):
        self.pub_cancel.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
