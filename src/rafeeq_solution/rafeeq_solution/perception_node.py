"""Perception node - shelf numbers, book colour/row, and the red bin.

Runs one of four modes, selected by the mission node over
``/rafeeq/perception/mode``:

``idle``    do nothing (default, so we never burn CPU or emit false positives)
``column``  read the shelf number plates, find the target column
``book``    find the target-colour book in that column and its row
``bin``     find the red collection bin

Each mode requires the same detection on several consecutive frames before it
commits. A single frame at 30 Hz is cheap to get wrong; three in a row with a
consistent answer is not.

Published competition outputs:
    /erc/shelf_column_identification   std_msgs/Int32
    /erc/shelf_row_identification      std_msgs/Int32

Both are latched (TRANSIENT_LOCAL) and re-published on a timer, so a scorer
that subscribes late still receives the value.
"""

import os
from collections import Counter, deque
from datetime import datetime

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32, String

import tf2_ros

# Importing tf2_geometry_msgs is what registers the PointStamped transform
# handler with tf2.  It is never referenced directly, but without it every
# Buffer.transform() call below raises a TypeException at runtime.
import tf2_geometry_msgs  # noqa: F401

from rclpy.duration import Duration
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

from . import arena, vision
from .image_utils import PinholeModel, depth_to_metres, patch_depth, to_bgr

try:
    import cv2
except ImportError as exc:                                  # pragma: no cover
    raise ImportError(
        'OpenCV (cv2) is required by rafeeq_solution but was not found. '
        'Inside the competition container run: '
        'apt-get update && apt-get install -y python3-opencv') from exc


LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('rafeeq_perception')

        self.declare_parameter('shelf_column_number', 2)
        self.declare_parameter('book_colour', 'red')
        self.declare_parameter('image_dir', '/opt/erc_ws/src/erc_images')
        self.declare_parameter('link_erc_images', True)
        self.declare_parameter('row_one_is_top', True)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('marker_min_score', 0.55)
        self.declare_parameter('column_tolerance_px', 90)
        self.declare_parameter('spawn_yaw', arena.SPAWN_YAW)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter(
            'rgb_topic',
            '/head_front_camera/head_front_camera/color/image_raw')
        self.declare_parameter(
            'depth_topic',
            '/head_front_camera/head_front_camera/depth/image_rect_raw')
        self.declare_parameter(
            'info_topic',
            '/head_front_camera/head_front_camera/color/camera_info')

        self.target_column = int(self.get_parameter(
            'shelf_column_number').value)
        self.target_colour = str(self.get_parameter('book_colour').value).lower()
        self.image_dir = str(self.get_parameter('image_dir').value)
        self.row_one_is_top = bool(self.get_parameter('row_one_is_top').value)
        self.confirm_frames = int(self.get_parameter('confirm_frames').value)
        self.marker_min_score = float(
            self.get_parameter('marker_min_score').value)
        self.column_tol = float(self.get_parameter('column_tolerance_px').value)
        self.spawn_yaw = float(self.get_parameter('spawn_yaw').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)

        if self.target_colour not in arena.BOOK_COLOURS:
            self.get_logger().error(
                'book_colour={!r} is not one of {}; perception will not find '
                'a book'.format(self.target_colour,
                                list(arena.BOOK_COLOURS)))

        self._prepare_image_dir()
        self.templates = self._load_templates()

        # Latest sensor data, kept loosely synchronised. At 30 Hz the frames
        # are at most one period apart, which is far tighter than any of the
        # tolerances below.
        self.depth_m = None
        self.camera = None
        self.mode = 'idle'

        self._votes = deque(maxlen=max(self.confirm_frames * 3, 9))
        self.column_slot = None
        self.column_u = None
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
            'perception up | target column={} colour={} | templates={} | '
            'images -> {}'.format(self.target_column, self.target_colour,
                                  sorted(self.templates), self.image_dir))

    # -- setup helpers ---------------------------------------------------

    def _prepare_image_dir(self):
        """Create the image directory and, if possible, the /erc_images link.

        The rules ask for images in ``/erc_images/``. That path is inside the
        container and is not volume-mounted, so anything written straight to
        it is lost when the container stops. We write into the mounted
        workspace instead and symlink ``/erc_images`` at it, which satisfies
        both the rulebook path and the need to keep the files.
        """
        try:
            os.makedirs(self.image_dir, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(
                'cannot create image dir {}: {}'.format(self.image_dir, exc))
            return

        if not self.get_parameter('link_erc_images').value:
            return

        link = '/erc_images'
        try:
            if os.path.islink(link):
                if os.path.realpath(link) != os.path.realpath(self.image_dir):
                    os.remove(link)
                    os.symlink(self.image_dir, link)
            elif not os.path.exists(link):
                os.symlink(self.image_dir, link)
        except OSError as exc:
            # Not fatal - the images still land in the mounted directory.
            self.get_logger().warn(
                'could not link /erc_images -> {}: {}'.format(
                    self.image_dir, exc))

    def _load_templates(self):
        """Load the ground-truth digit textures shipped by erc_description."""
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
        self._votes.clear()
        self.get_logger().info('mode -> {}'.format(mode))

    def on_info(self, msg):
        model = PinholeModel(msg)
        if model.valid:
            self.camera = model

    def on_depth(self, msg):
        try:
            self.depth_m = depth_to_metres(msg)
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

    def world_point(self, u, v, header):
        """Deproject a pixel into world XYZ, or None if it cannot be done.

        Goes pixel -> camera optical frame -> odom (via TF) -> world (via the
        known spawn pose). Returns None on missing depth, missing intrinsics
        or a TF gap; every caller has a geometric fallback for that case.
        """
        if self.camera is None or self.depth_m is None:
            return None

        depth = patch_depth(self.depth_m, u, v)
        if depth is None:
            return None

        x, y, z = self.camera.deproject(u, v, depth)

        point = PointStamped()
        point.header.frame_id = header.frame_id
        point.header.stamp = Time().to_msg()   # zero stamp = latest available
        point.point.x, point.point.y, point.point.z = x, y, z

        try:
            odom_pt = self.tf_buffer.transform(
                point, self.odom_frame,
                timeout=Duration(seconds=0.2))
        except Exception as exc:
            self.get_logger().warn(
                'TF {} -> {} failed: {}'.format(
                    header.frame_id, self.odom_frame, exc),
                throttle_duration_sec=5.0)
            return None

        wx, wy = arena.odom_to_world(
            odom_pt.point.x, odom_pt.point.y, self.spawn_yaw)
        return (wx, wy, odom_pt.point.z)

    # -- mode: column ----------------------------------------------------

    def _do_column(self, bgr, header):
        plates = vision.detect_number_plates(
            bgr, self.templates, min_score=self.marker_min_score)

        if not plates:
            return

        slots = self._assign_slots(plates, header)
        target = next(
            (p for p in plates if p['digit'] == self.target_column), None)

        if target is None or slots.get(target['digit']) is None:
            self._status('column:searching:saw={}'.format(
                sorted(p['digit'] for p in plates)))
            return

        slot = slots[target['digit']]
        self._votes.append(slot)

        if not self._settled(slot):
            return

        self.column_slot = slot
        self.column_u = target['centroid'][0]

        self.pub_column.publish(Int32(data=int(self.target_column)))
        self.pub_slot.publish(Int32(data=int(slot)))
        self._publish_target(arena.SHELF_X, arena.COLUMN_Y[slot],
                             arena.MARKER_Z)

        path = self._save(
            bgr, [target],
            lambda d: 'COLUMN {} (slot {}, {:.2f})'.format(
                d['digit'], slot, d['score']),
            'shelf_column',
            ['target column {} -> shelf slot {} (world y={:+.2f} m)'.format(
                self.target_column, slot, arena.COLUMN_Y[slot]),
             'plates seen: {}'.format(
                 ', '.join('{}@slot{}'.format(p['digit'], slots.get(p['digit']))
                           for p in plates))])

        self.get_logger().info(
            'COLUMN CONFIRMED: number {} is at shelf slot {} '
            '(world y={:+.2f} m) -> {}'.format(
                self.target_column, slot, arena.COLUMN_Y[slot], path))
        self._status('column:done:{}'.format(slot))
        self.mode = 'idle'
        self._votes.clear()

    def _assign_slots(self, plates, header):
        """Map each detected digit to a physical shelf slot 1..5.

        Primary method is deprojection: measure the plate's world ``y`` and
        snap it to the nearest column centre. Columns are 1 m apart, so the
        measurement only has to be good to +/- 0.5 m.

        Fallback, used when depth or TF is unavailable: if all five plates
        are in view, their left-to-right image order *is* the slot order,
        because the robot faces the shelf along world +x and camera x
        increases as world y decreases.
        """
        slots = {}

        for plate in plates:
            u, v = plate['centroid']
            world = self.world_point(u, v, header)
            if world is not None:
                slots[plate['digit']] = arena.nearest_column_slot(world[1])

        if len(slots) == len(plates):
            return slots

        if len(plates) == arena.NUM_COLUMNS:
            ordered = sorted(plates, key=lambda p: p['centroid'][0])
            for index, plate in enumerate(ordered, start=1):
                slots.setdefault(plate['digit'], index)
            self.get_logger().warn(
                'slot assignment fell back to image ordering '
                '(depth or TF unavailable)', throttle_duration_sec=10.0)

        return slots

    # -- mode: book ------------------------------------------------------

    def _do_book(self, bgr, header):
        books = vision.detect_books(bgr)
        if not books:
            self._status('book:searching:none')
            return

        # The robot is parked directly in front of the target column by now,
        # so the column we want is the one straddling the image centre.
        # ``column_u`` came from the 3 m shelf scan and is stale here - using
        # it would filter against the wrong part of the image.
        centre_u = bgr.shape[1] / 2.0
        column_books = vision.books_in_column(books, centre_u, self.column_tol)
        if len(column_books) < 2:
            column_books = books

        ranked = vision.rank_rows(column_books, self.row_one_is_top)
        target = next(
            (b for b in ranked if b['colour'] == self.target_colour), None)

        if target is None:
            self._status('book:searching:saw={}'.format(
                sorted({b['colour'] for b in ranked})))
            return

        world = self.world_point(*target['centroid'], header=header)
        row, method = self._resolve_row(target, ranked, world)

        if row is None:
            self._status('book:searching:ambiguous:{}'.format(len(ranked)))
            return

        self._votes.append(row)
        if not self._settled(row):
            return

        self.book_row = row
        self.pub_row.publish(Int32(data=int(row)))

        if world is not None:
            self._publish_target(*world)
            measured = 'row from {}, measured world z={:.2f} m'.format(
                method, world[2])
        else:
            self._publish_target(
                arena.BOOK_X,
                arena.COLUMN_Y.get(self.column_slot or 3, 0.0),
                arena.ROW_Z[row])
            measured = 'row from {}, depth unavailable'.format(method)

        path = self._save(
            bgr, [target],
            lambda d: '{} BOOK row {}'.format(d['colour'].upper(), row),
            'target_book',
            ['target book: {} in row {} ({} books in column)'.format(
                self.target_colour, row, len(ranked)),
             '{} (nominal {:.2f} m)'.format(measured, arena.ROW_Z[row])])

        self.get_logger().info(
            'BOOK CONFIRMED: {} book in row {} ({}) -> {}'.format(
                self.target_colour, row, method, path))
        self._status('book:done:{}'.format(row))
        self.mode = 'idle'
        self._votes.clear()

    def _resolve_row(self, target, ranked, world):
        """Decide the competition row number, and say how we decided.

        Two independent methods, because neither is reliable alone:

        ``ordering``
            The four books of a column occupy the four active rows, one
            each, so their vertical order in the image *is* the row order.
            This needs all four in frame, which needs a standoff of at least
            ~1.17 m (the rows span 1.24 m and the camera covers 1.06 m at
            1.0 m range). Closer than that and the ranking silently shifts.

        ``height``
            Snap the measured world z to the nearest nominal row height.
            Works with a single book in frame, but needs depth and TF.

        When both are available they cross-check each other and a
        disagreement is logged loudly - that is the signature of a wrong
        ``row_one_is_top`` setting.
        """
        ordered_row = target.get('row')
        height_row = None

        if world is not None:
            height_row = arena.nearest_row(world[2])
            if not self.row_one_is_top:
                # arena.ROW_Z is indexed top-first; flip to match the
                # bottom-first convention.
                height_row = (len(arena.ROW_Z) + 1) - height_row

        if len(ranked) >= len(arena.ROW_Z) and ordered_row is not None:
            if height_row is not None and height_row != ordered_row:
                self.get_logger().warn(
                    'row disagreement: vertical order says {} but measured '
                    'height says {} - check the row_one_is_top parameter'
                    .format(ordered_row, height_row))
            return ordered_row, 'ordering'

        if height_row is not None:
            self.get_logger().info(
                'only {} of {} books visible; using measured height for the '
                'row instead of vertical ordering'.format(
                    len(ranked), len(arena.ROW_Z)))
            return height_row, 'height'

        self.get_logger().warn(
            'only {} books in frame and no depth available - cannot resolve '
            'the row; back off to {:.1f} m so all four rows fit'.format(
                len(ranked), arena.BOOK_SCAN_STANDOFF),
            throttle_duration_sec=5.0)
        return None, None

    # -- mode: bin -------------------------------------------------------

    def _do_bin(self, bgr, header):
        found = vision.detect_red_bin(bgr)
        if found is None:
            self._status('bin:searching')
            return

        # Vote on a coarse horizontal bucket so jitter does not reset us.
        bucket = int(found['centroid'][0] // 40)
        self._votes.append(bucket)
        if not self._settled(bucket):
            return

        world = self.world_point(*found['centroid'], header=header)
        if world is not None:
            self._publish_target(*world)
            detail = 'world ({:+.2f}, {:+.2f}, {:.2f})'.format(*world)
        else:
            self._publish_target(*arena.BIN_XYZ)
            detail = 'depth unavailable, using nominal bin pose'

        # Hand the mission node the pixel offset too - aligning on bearing
        # is more reliable than aligning on a deprojected point.
        offset = found['centroid'][0] - (bgr.shape[1] / 2.0)

        path = self._save(
            bgr, [found], lambda d: 'COLLECTION BIN', 'red_bin',
            ['red bin detected, {}'.format(detail),
             'horizontal offset {:+.0f} px from image centre'.format(offset)])

        self.get_logger().info(
            'BIN CONFIRMED: {} (offset {:+.0f} px) -> {}'.format(
                detail, offset, path))
        self._status('bin:done:{:.1f}'.format(offset))
        self.mode = 'idle'
        self._votes.clear()

    # -- shared ----------------------------------------------------------

    def _settled(self, value):
        """True once the same answer has held for ``confirm_frames``."""
        if len(self._votes) < self.confirm_frames:
            return False
        recent = list(self._votes)[-self.confirm_frames:]
        return all(v == value for v in recent) and \
            Counter(recent).most_common(1)[0][1] == self.confirm_frames

    def _publish_target(self, wx, wy, wz):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.point.x, msg.point.y, msg.point.z = float(wx), float(wy), float(wz)
        self.pub_target.publish(msg)

    def _status(self, text):
        self.pub_status.publish(String(data=text))

    def _save(self, bgr, detections, label_fn, kind, lines):
        """Annotate and write a timestamped image, returning its path."""
        stamp_text = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        name = '{}_{}.png'.format(kind, stamp_text)
        path = os.path.join(self.image_dir, name)

        annotated = vision.annotate(bgr, detections, label_fn)
        annotated = vision.stamp(
            annotated, [stamp_text] + list(lines))

        try:
            if not cv2.imwrite(path, annotated):
                raise IOError('imwrite returned False')
        except Exception as exc:
            self.get_logger().error(
                'failed to save {}: {}'.format(path, exc))
            return '<unsaved>'

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
