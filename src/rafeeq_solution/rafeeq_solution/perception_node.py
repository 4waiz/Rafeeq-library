"""Perception node - shelf numbers, book colour/row/position, and the red bin.

Runs one of four modes, selected by the mission node over
``/rafeeq/perception/mode``:

``idle``    do nothing (default, so we never burn CPU or emit false positives)
``column``  read the shelf number plates and find the target column
``book``    find the target-colour book in that column, its row **and where it
            physically is**
``bin``     find the red collection bin and a reachable placement point inside it

Published competition outputs::

    /erc/shelf_column_identification   std_msgs/Int32
    /erc/shelf_row_identification      std_msgs/Int32

Both are latched (TRANSIENT_LOCAL) and re-published on a timer, so a scorer that
subscribes late still receives the value.  Neither is published because it was
passed in as an argument - each goes out only once the corresponding marker has
actually been identified in an image.

Three things this node is careful about, because each was wrong before:

* **The measured position is the output that matters.**  The organiser jitters
  every book by up to +/- 0.25 m in world y.  A row number alone cannot be
  grasped; ``/rafeeq/perception/target_point`` carries the measured 3D point and
  the grasp planner consumes it.  If depth or TF is unavailable the mode
  *fails* rather than publishing a nominal column-centre position.
* **Transforms are taken at the image's own timestamp.**  Deprojecting a pixel
  from a 30 Hz image against "the latest available" transform silently mixes an
  old image with a new pose, which on a moving base is metres of error.
* **The bin is not just red pixels.**  Red books, and the red book in our own
  gripper, are also red.  A bin candidate has to be the right size, in the right
  place, and at a plausible depth before it is believed.
"""

import math
import os
from collections import Counter, deque
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32, String

import tf2_ros

# Importing tf2_geometry_msgs is what registers the PointStamped transform
# handler with tf2.  It is never referenced directly, but without it every
# Buffer.transform() call below raises a TypeException at runtime.
import tf2_geometry_msgs  # noqa: F401

from ament_index_python.packages import get_package_share_directory

from . import arena, vision
from .image_utils import (PinholeModel, depth_to_metres, patch_depth,
                          patch_half_for_bbox, to_bgr)

try:
    import cv2
except ImportError as exc:                                  # pragma: no cover
    raise ImportError(
        'OpenCV (cv2) is required by rafeeq_solution but was not found. '
        'It ships with ros-humble-desktop; if this fires, install '
        'python3-opencv.') from exc


LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('rafeeq_perception')

        self.declare_parameter('shelf_column_number', 0)
        self.declare_parameter('book_colour', '')
        self.declare_parameter('image_dir', '')
        self.declare_parameter('row_one_is_top', True)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('marker_min_score', 0.55)
        self.declare_parameter('column_tolerance_px', 90)
        self.declare_parameter('spawn_yaw', arena.SPAWN_YAW)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('tf_timeout', 0.25)
        self.declare_parameter('max_image_age', 1.0)
        self.declare_parameter(
            'rgb_topic',
            '/head_front_camera/head_front_camera/color/image_raw')
        self.declare_parameter(
            'depth_topic',
            '/head_front_camera/head_front_camera/depth/image_rect_raw')
        self.declare_parameter(
            'info_topic',
            '/head_front_camera/head_front_camera/color/camera_info')
        self.declare_parameter('rgb_depth_baseline', 0.015)
        # Above this speed the robot is moving fast enough that using a
        # transform from a different instant would matter.
        self.declare_parameter('stationary_speed', 0.02)
        # How long the base must have been still before the newest transform
        # is guaranteed to describe where it is now, not where it was.
        self.declare_parameter('tf_settle_seconds', 2.5)
        # Extra allowance on top of the organiser's +/-0.25 m book jitter when
        # deciding whether a detected book belongs to the column the robot is
        # parked at. Covers the robot not being exactly on the column centre.
        self.declare_parameter('column_band_margin', 0.22)

        self.target_column = int(
            self.get_parameter('shelf_column_number').value)
        self.target_colour = str(
            self.get_parameter('book_colour').value).strip().lower()
        self.row_one_is_top = bool(self.get_parameter('row_one_is_top').value)
        self.confirm_frames = int(self.get_parameter('confirm_frames').value)
        self.marker_min_score = float(
            self.get_parameter('marker_min_score').value)
        self.column_tol = float(
            self.get_parameter('column_tolerance_px').value)
        self.spawn_yaw = float(self.get_parameter('spawn_yaw').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.max_image_age = float(self.get_parameter('max_image_age').value)
        self.baseline = float(self.get_parameter('rgb_depth_baseline').value)
        self.stationary_speed = float(
            self.get_parameter('stationary_speed').value)
        self.tf_settle_seconds = float(
            self.get_parameter('tf_settle_seconds').value)
        self.column_band_margin = float(
            self.get_parameter('column_band_margin').value)
        self.base_speed = None
        self.base_world = None
        self._stopped_at = None
        self._late_tf_warned = False

        self.image_dir = self._resolve_image_dir()
        self.templates = self._load_templates()

        if self.target_colour and self.target_colour not in arena.BOOK_COLOURS:
            self.get_logger().error(
                'book_colour={!r} is not one of {}; no book will be '
                'found'.format(self.target_colour, list(arena.BOOK_COLOURS)))

        self.depth_m = None
        self.depth_stamp = None
        self.camera = None
        self.mode = 'idle'

        self._votes = deque(maxlen=max(self.confirm_frames * 3, 9))
        self.column_slot = None
        self.book_row = None

        self.pub_column = self.create_publisher(
            Int32, '/erc/shelf_column_identification', LATCHED)
        self.pub_row = self.create_publisher(
            Int32, '/erc/shelf_row_identification', LATCHED)
        self.pub_status = self.create_publisher(
            String, '/rafeeq/perception/status', 10)
        self.pub_slot = self.create_publisher(
            Int32, '/rafeeq/perception/column_slot', LATCHED)
        self.pub_target = self.create_publisher(
            PointStamped, '/rafeeq/perception/target_point', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            String, '/rafeeq/perception/mode', self.on_mode, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value,
            self.on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.on_depth, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('rgb_topic').value,
            self.on_rgb, qos_profile_sensor_data)

        self.create_timer(2.0, self._republish)

        self.get_logger().info(
            'perception up | target column={} colour={} | digit templates={} '
            '| images -> {}'.format(
                self.target_column, self.target_colour or '(unset)',
                sorted(self.templates), self.image_dir))

    # -- setup helpers ---------------------------------------------------

    def _resolve_image_dir(self):
        """Find the repository's ``erc_images/`` directory and use it.

        The rules ask for annotated images in ``erc_images/``.  Resolution order:

        1. an explicit ``image_dir`` parameter, if given;
        2. ``ERC_IMAGES_DIR`` in the environment;
        3. ``<workspace>/erc_images`` found by walking up from this package's
           source directory, which is what a normal clone and a mounted
           container workspace both look like;
        4. ``./erc_images`` relative to the current directory.

        Deliberately no attempt to create a symlink at the filesystem root:
        that needs write permission on ``/``, is not available in every
        container, and is not something a judge should have to grant.
        """
        given = str(self.get_parameter('image_dir').value).strip()
        if given:
            return self._ensure(given)

        env = os.environ.get('ERC_IMAGES_DIR', '').strip()
        if env:
            return self._ensure(env)

        here = os.path.dirname(os.path.abspath(__file__))
        for _ in range(6):
            here = os.path.dirname(here)
            if not here or here == os.sep:
                break
            candidate = os.path.join(here, 'erc_images')
            if os.path.isdir(candidate):
                return self._ensure(candidate)

        return self._ensure(os.path.join(os.getcwd(), 'erc_images'))

    def _ensure(self, path):
        try:
            os.makedirs(path, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(
                'cannot create image directory {}: {}'.format(path, exc))
        return path

    def _load_templates(self):
        """Load the ground-truth digit textures shipped by erc_description.

        Template matching against the exact textures the simulator renders
        beats OCR here: there are five classes, the ground truth ships with the
        arena, and there is nothing to train or tune.
        """
        try:
            share = get_package_share_directory('erc_description')
        except Exception as exc:
            self.get_logger().error(
                'erc_description not found ({}); shelf numbers cannot be '
                'read'.format(exc))
            return {}

        textures = os.path.join(share, 'models', 'number_marker', 'textures')
        templates = vision.load_digit_templates(textures)
        if len(templates) < arena.NUM_COLUMNS:
            self.get_logger().warn(
                'only loaded {} of {} digit templates from {}'.format(
                    len(templates), arena.NUM_COLUMNS, textures))
        return templates

    # -- subscriptions ---------------------------------------------------

    def on_mode(self, msg):
        mode = msg.data.strip().lower()
        if mode == self.mode:
            return
        if mode not in ('idle', 'column', 'book', 'bin'):
            self.get_logger().warn('ignoring unknown mode {!r}'.format(mode))
            return
        self.mode = mode
        # Clear every cached result on a mode change so a stale detection from
        # an earlier state, or an earlier run, can never satisfy a new one.
        self._votes.clear()
        self.get_logger().info('mode -> {}'.format(mode))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.base_world = arena.odom_to_world(p.x, p.y, self.spawn_yaw)
        t = msg.twist.twist
        self.base_speed = math.sqrt(
            t.linear.x ** 2 + t.linear.y ** 2) + abs(t.angular.z) * 0.3
        now = self.get_clock().now().nanoseconds / 1e9
        if self.base_speed > self.stationary_speed:
            self._stopped_at = None
        elif self._stopped_at is None:
            self._stopped_at = now

    def settled(self):
        """True once the transform stream has caught up with a stopped base.

        The camera frames on this bridge are stamped ahead of ``/tf``, so an
        image-time lookup asks for the future and the only thing available is
        the newest transform.  That is safe when the robot is standing still -
        but *only once the newest transform post-dates the stop*.  Used the
        moment the wheels halt, the newest transform is still from mid-motion,
        and the whole scene is deprojected against a pose the robot has already
        left.  Measured, that put every book about 0.45 m sideways - half a
        column pitch, which is enough to place them all in the wrong column
        while ``nearest_column_slot`` quietly snapped the markers back to the
        right one and hid the bias.
        """
        if self._stopped_at is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return (now - self._stopped_at) >= self.tf_settle_seconds

    def on_info(self, msg):
        model = PinholeModel(msg)
        if model.valid:
            self.camera = model

    def on_depth(self, msg):
        try:
            self.depth_m = depth_to_metres(msg)
            self.depth_stamp = Time.from_msg(msg.header.stamp)
        except Exception as exc:
            self.get_logger().warn(
                'depth conversion failed: {}'.format(exc), once=True)

    def on_rgb(self, msg):
        if self.mode == 'idle':
            return
        try:
            bgr = to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(
                'rgb conversion failed: {}'.format(exc), once=True)
            return

        if self.mode == 'column':
            self._do_column(bgr, msg.header)
        elif self.mode == 'book':
            self._do_book(bgr, msg.header)
        elif self.mode == 'bin':
            self._do_bin(bgr, msg.header)

    # -- geometry --------------------------------------------------------

    def depth_is_current(self, header):
        """True if the depth frame is close enough in time to this colour one."""
        if self.depth_stamp is None:
            return False
        age = abs((Time.from_msg(header.stamp) - self.depth_stamp).nanoseconds)
        return age / 1e9 <= self.max_image_age

    def world_point(self, u, v, header, bbox=None):
        """Deproject a colour pixel into world XYZ, or None.

        Pixel -> camera optical frame -> odom (via TF **at the image's own
        timestamp**) -> world (via the known spawn pose).

        The colour and depth cameras are 15 mm apart on the same rig and the
        depth image is rendered from the depth frame, so a colour pixel does
        not index the depth image exactly.  The disparity is
        ``fx * baseline / range`` - about 3.4 px on a 7 px book spine at
        1.5 m - so it is corrected rather than ignored.
        """
        if self.camera is None or self.depth_m is None:
            return None
        if not self.depth_is_current(header):
            self.get_logger().warn(
                'depth frame is stale relative to the colour frame; not '
                'deprojecting', throttle_duration_sec=5.0)
            return None

        half = patch_half_for_bbox(bbox) if bbox else 4
        depth = patch_depth(self.depth_m, u, v, half=half)
        if depth is None:
            return None

        shift = self.camera.fx * self.baseline / max(depth, 1e-3)
        corrected = patch_depth(self.depth_m, u - shift, v, half=half)
        if corrected is not None:
            depth = corrected

        x, y, z = self.camera.deproject(u, v, depth)

        point = PointStamped()
        point.header.frame_id = header.frame_id
        point.header.stamp = header.stamp
        point.point.x, point.point.y, point.point.z = x, y, z

        try:
            odom_pt = self.tf_buffer.transform(
                point, self.odom_frame,
                timeout=Duration(seconds=self.tf_timeout))
        except Exception as exc:
            # On this bridge the camera images are stamped 1.5-2 s *ahead* of
            # the newest transform, so an image-time lookup always asks for the
            # future.  Falling back to the latest transform is only sound when
            # the base is not moving - which it is not during a scan, because
            # the mission stops before it looks.  While moving, the frame is
            # dropped instead: an old pose with a new image is metres of error.
            if not self.settled():
                self.get_logger().info(
                    'waiting for transforms to catch up with the stopped base '
                    'before deprojecting', throttle_duration_sec=5.0)
                return None
            point.header.stamp = Time().to_msg()   # latest available
            try:
                odom_pt = self.tf_buffer.transform(
                    point, self.odom_frame,
                    timeout=Duration(seconds=self.tf_timeout))
            except Exception as exc2:
                self.get_logger().warn(
                    'TF {} -> {} failed even against the latest transform: '
                    '{}'.format(header.frame_id, self.odom_frame, exc2),
                    throttle_duration_sec=5.0)
                return None
            if not self._late_tf_warned:
                self._late_tf_warned = True
                self.get_logger().info(
                    'camera frames are stamped ahead of /tf on this bridge; '
                    'using the latest transform while the base is stationary '
                    '(<= {:.2f} m/s)'.format(self.stationary_speed))

        wx, wy = arena.odom_to_world(
            odom_pt.point.x, odom_pt.point.y, self.spawn_yaw)
        return (wx, wy, odom_pt.point.z)

    # -- mode: column ----------------------------------------------------

    def _do_column(self, bgr, header):
        plates = vision.detect_number_plates(
            bgr, self.templates, min_score=self.marker_min_score)
        if not plates:
            self._status('column:searching:none')
            return

        slots = self._assign_slots(plates, header)
        target = next(
            (p for p in plates if p['digit'] == self.target_column), None)

        if target is None or slots.get(self.target_column) is None:
            self._status('column:searching:saw={}'.format(
                sorted(p['digit'] for p in plates)))
            return

        slot = slots[self.target_column]
        self._votes.append(slot)
        if not self._settled(slot):
            return

        self.column_slot = slot
        self.pub_column.publish(Int32(data=int(self.target_column)))
        self.pub_slot.publish(Int32(data=int(slot)))

        # Publish where the plate actually *is*, not the nominal centre of the
        # slot it snapped to.  The mission drives to this measurement, so the
        # robot ends up in front of the plate it read even when odometry drift
        # has moved the whole world estimate - the drive and the measurement
        # share the same drifting frame, so the error cancels.  The snapped
        # slot number is still reported, but only as a label.
        measured = self.world_point(
            target['centroid'][0], target['centroid'][1], header,
            bbox=target.get('bbox'))
        if measured is not None:
            self._publish_target(measured[0], measured[1], measured[2])

        # The rules ask for a box around the target **shelf column**, not a
        # tiny box around the digit that identifies it.  Grow the plate box
        # down over the column it labels, clipped to the image.
        column_box = vision.column_box_from_plate(target, bgr.shape)
        path = self._save(
            bgr,
            [{'bbox': column_box, 'colour': 'marker'},
             {'bbox': target['bbox'], 'colour': 'marker'}],
            [lambda d: 'TARGET SHELF COLUMN {}'.format(self.target_column),
             lambda d: 'marker {} ({:.2f})'.format(
                 target['digit'], target['score'])],
            'shelf_column',
            ['requested column {} -> physical shelf slot {} '
             '(world y = {:+.2f} m)'.format(
                 self.target_column, slot, arena.COLUMN_Y[slot]),
             'plates read: {}'.format(', '.join(
                 '{}@slot{}'.format(p['digit'], slots.get(p['digit'], '?'))
                 for p in plates))])

        self.get_logger().info(
            'COLUMN CONFIRMED: number {} is above physical column {} '
            '(world y = {:+.2f} m) -> {}'.format(
                self.target_column, slot, arena.COLUMN_Y[slot], path))
        self._status('column:done:{}:{}'.format(slot, path))
        self.mode = 'idle'
        self._votes.clear()

    def _assign_slots(self, plates, header):
        """Map each detected digit to a physical shelf slot 1..5.

        Primary method is deprojection: measure the plate's world ``y`` and
        snap it to the nearest column centre.  Plates are *not* jittered - they
        sit exactly on the centres - and the columns are 1 m apart, so the
        measurement only has to be good to +/- 0.5 m.

        Fallback, only when all five plates are in view: their left-to-right
        image order is the slot order, because the robot faces the shelf along
        world +x and image x increases as world y decreases.
        """
        slots = {}
        for plate in plates:
            u, v = plate['centroid']
            world = self.world_point(u, v, header,
                                     bbox=plate.get('bbox'))
            if world is not None:
                slots[plate['digit']] = arena.nearest_column_slot(world[1])

        if len(slots) == len(plates):
            return slots

        if len(plates) == arena.NUM_COLUMNS:
            ordered = sorted(plates, key=lambda p: p['centroid'][0])
            for index, plate in enumerate(ordered, start=1):
                slots.setdefault(plate['digit'], index)
            self.get_logger().warn(
                'slot assignment fell back to left-to-right image ordering '
                '(depth or TF unavailable for some plates)',
                throttle_duration_sec=10.0)
        return slots

    # -- mode: book ------------------------------------------------------

    def _do_book(self, bgr, header):
        """Find the target book, its row, and where it physically is.

        Books are selected by their **measured world position**, not by how
        close they are to the image centre.  The pixel test that used to do
        this assumed the robot was parked exactly on the column centre and that
        the column therefore straddled the middle of the frame; in practice the
        books of two columns can both be in view and neither cluster sits at
        the centre, so the filter either dropped the target or admitted a book
        from the wrong column.  Deprojecting each candidate and asking whether
        it lies inside *this* column's jitter band answers the actual question.
        """
        books = vision.detect_books(bgr)
        if not books:
            self._status('book:searching:none')
            return

        # Deproject every candidate once.
        placed = []
        for book in books:
            world = self.world_point(*book['centroid'], header=header,
                                     bbox=book['bbox'])
            if world is None:
                continue
            book = dict(book)
            book['world'] = world
            placed.append(book)

        if not placed:
            self._status('book:searching:no_depth')
            return

        # Column membership is judged **relative to the robot**, not against
        # the nominal column centre in world coordinates.
        #
        # This matters more than it looks. The mecanum base's wheel odometry
        # drifts: measured against Gazebo ground truth after driving to a
        # column, the robot believed it was at world (1.381, 0.995) facing
        # +x when it was actually at (1.147, 1.160) yawed 0.181 rad - 0.29 m
        # and 10 degrees out. Every world-frame position derived from that
        # pose inherits the error, which came to about 0.44 m laterally at
        # book range and pushed every book outside its column's jitter band.
        #
        # The robot parks at the column it identified, so a book belonging to
        # that column is near the robot's *own* y. Both that and the book's
        # measured y come from the same odometry, so the drift cancels.
        slot = self.column_slot
        reference = None
        if self.base_world is not None:
            reference = self.base_world[1]
        elif slot in arena.COLUMN_Y:
            reference = arena.COLUMN_Y[slot]

        if reference is None:
            in_column = placed
        else:
            band = arena.COLUMN_JITTER + self.column_band_margin
            in_column = [b for b in placed
                         if abs(b['world'][1] - reference) <= band]

        if not in_column:
            self._status('book:searching:none_in_column:{}'.format(
                ','.join('{:+.2f}'.format(b['world'][1]) for b in placed[:6])))
            return

        # Row ranking uses only the books that belong to this column, and only
        # those at a plausible shelf height.
        in_column = [b for b in in_column
                     if arena.row_height_plausible(b['world'][2])]
        if not in_column:
            self._status('book:searching:bad_heights')
            return

        target = next(
            (b for b in in_column if b['colour'] == self.target_colour), None)
        if target is None:
            self._status('book:searching:saw={}'.format(
                sorted({b['colour'] for b in in_column})))
            return

        world = target['world']
        ranked = vision.rank_rows(in_column, self.row_one_is_top)
        row, method = self._resolve_row(target, ranked, world)
        if row is None:
            self._status('book:searching:ambiguous:{}'.format(len(ranked)))
            return

        self._votes.append(row)
        if not self._settled(row):
            return

        self.book_row = row
        self.pub_row.publish(Int32(data=int(row)))
        self._publish_target(*world)

        offset = (world[1] - arena.COLUMN_Y[slot]
                  if slot in arena.COLUMN_Y else 0.0)
        path = self._save(
            bgr, [target],
            [lambda d: '{} BOOK - ROW {}'.format(d['colour'].upper(), row)],
            'target_book',
            ['target: {} book in row {} (row from {})'.format(
                self.target_colour, row, method),
             'measured world ({:+.3f}, {:+.3f}, {:.3f}) m'.format(*world),
             'offset from column centre {:+.3f} m ({} book(s) in column)'
             .format(offset, len(in_column))])

        self.get_logger().info(
            'BOOK CONFIRMED: {} book in row {} ({}), measured at world '
            '({:+.3f}, {:+.3f}, {:.3f}), {:+.3f} m off the column centre '
            '-> {}'.format(self.target_colour, row, method,
                           world[0], world[1], world[2], offset, path))
        self._status('book:done:{}:{}'.format(row, path))
        self.mode = 'idle'
        self._votes.clear()

    def _resolve_row(self, target, ranked, world):
        """Decide the competition row number, and say how it was decided.

        ``ordering``  the books of a column occupy the four active rows one
                      each, so their vertical order in the image is the row
                      order.  Needs all four in frame.
        ``height``    snap the measured world z to the nearest nominal row.
                      Works with a single book in frame.

        When both are available they cross-check, and a disagreement is logged
        loudly - that is the signature of a wrong ``row_one_is_top``.
        """
        ordered_row = target.get('row')
        height_row = arena.nearest_row(world[2])
        if not self.row_one_is_top:
            height_row = (len(arena.ROW_Z) + 1) - height_row

        if len(ranked) >= len(arena.ROW_Z) and ordered_row is not None:
            if height_row != ordered_row:
                self.get_logger().warn(
                    'row disagreement: vertical order says {} but measured '
                    'height says {} - check row_one_is_top'.format(
                        ordered_row, height_row))
                return height_row, 'height (ordering disagreed)'
            return ordered_row, 'ordering confirmed by height'

        return height_row, 'height'

    # -- mode: bin -------------------------------------------------------

    def _do_bin(self, bgr, header):
        found = vision.detect_red_bin(bgr)
        if found is None:
            self._status('bin:searching:none')
            return

        world = self.world_point(*found['centroid'], header=header,
                                 bbox=found['bbox'])
        if world is None:
            self._status('bin:searching:no_depth')
            return

        # A red blob at a plausible depth is still not necessarily the bin: a
        # red book on the shelf, or the red book in our own gripper, are both
        # red.  Require it to be where the bin actually is.
        if not arena.point_in_bin(world[0], world[1], world[2], margin=0.35):
            self.get_logger().warn(
                'rejecting a red blob at ({:+.2f}, {:+.2f}, {:.2f}) - too far '
                'from where the collection bin can be'.format(*world),
                throttle_duration_sec=5.0)
            self._status('bin:searching:not_the_bin')
            return

        bucket = int(found['centroid'][0] // 40)
        self._votes.append(bucket)
        if not self._settled(bucket):
            return

        # The observed point is on the bin's near *wall*, not inside it.  The
        # placement target is the middle of the bin's mouth, derived from the
        # measured position plus the bin's known extent - a red pixel's depth
        # is not by itself a place to let go of a book.
        placement = (
            float(np.clip(world[0] + (arena.BIN_X[1] - arena.BIN_X[0]) / 2.0,
                          arena.BIN_X[0] + 0.10, arena.BIN_X[1] - 0.10)),
            float(np.clip(world[1], arena.BIN_Y[0] + 0.06,
                          arena.BIN_Y[1] - 0.06)),
            arena.BIN_RIM_Z,
        )
        self._publish_target(*placement)

        path = self._save(
            bgr, [found], [lambda d: 'COLLECTION BIN'], 'red_bin',
            ['bin observed at world ({:+.3f}, {:+.3f}, {:.3f}) m'.format(*world),
             'placement target ({:+.3f}, {:+.3f}, {:.3f}) m, rim {:.3f} m'
             .format(placement[0], placement[1], placement[2],
                     arena.BIN_RIM_Z)])

        self.get_logger().info(
            'BIN CONFIRMED: observed ({:+.3f}, {:+.3f}, {:.3f}), placing at '
            '({:+.3f}, {:+.3f}, {:.3f}) -> {}'.format(
                world[0], world[1], world[2],
                placement[0], placement[1], placement[2], path))
        self._status('bin:done:{:.3f}:{}'.format(placement[0], path))
        self.mode = 'idle'
        self._votes.clear()

    # -- shared ----------------------------------------------------------

    def _settled(self, value):
        """True once the same answer has held for ``confirm_frames``."""
        if len(self._votes) < self.confirm_frames:
            return False
        recent = list(self._votes)[-self.confirm_frames:]
        return (all(v == value for v in recent)
                and Counter(recent).most_common(1)[0][1] == self.confirm_frames)

    def _publish_target(self, wx, wy, wz):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.point.x, msg.point.y, msg.point.z = float(wx), float(wy), float(wz)
        self.pub_target.publish(msg)

    def _status(self, text):
        self.pub_status.publish(String(data=text))

    def _save(self, bgr, detections, label_fns, kind, lines):
        """Annotate and write a timestamped image, returning its path."""
        stamp_text = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        name = '{}_{}.png'.format(kind, stamp_text)
        path = os.path.join(self.image_dir, name)

        annotated = vision.annotate_many(bgr, detections, label_fns)
        annotated = vision.stamp(
            annotated, ['Rafeeq / Team Kanban  ' + stamp_text] + list(lines))

        try:
            if not cv2.imwrite(path, annotated):
                raise IOError('imwrite returned False')
        except Exception as exc:
            self.get_logger().error(
                'failed to save evidence image {}: {}'.format(path, exc))
            return ''
        return path

    def _republish(self):
        """Keep the competition topics alive for late subscribers."""
        if self.column_slot is not None:
            self.pub_column.publish(Int32(data=int(self.target_column)))
            self.pub_slot.publish(Int32(data=int(self.column_slot)))
        if self.book_row is not None:
            self.pub_row.publish(Int32(data=int(self.book_row)))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
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
