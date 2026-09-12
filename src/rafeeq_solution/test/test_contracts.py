"""Offline regression tests for the defects this package actually had.

No ROS, no Gazebo, no camera.  Run them before every attempt::

    python3 -m pytest src/rafeeq_solution/test -q

Each test here exists because something broke in exactly that way.  The name of
the game is that a test fails for the *reason* the run would have failed, so the
failure message is a diagnosis rather than a puzzle.
"""

import ast
import math
import os
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
PKG = os.path.join(REPO, 'src', 'rafeeq_solution')
sys.path.insert(0, PKG)

from rafeeq_solution import arena, scoring          # noqa: E402


# --------------------------------------------------------------------------
# Cross-module attribute contract
# --------------------------------------------------------------------------

MODULES = ('arena', 'vision', 'image_utils', 'kinematics', 'grasp', 'scoring')


def _module_source(name):
    with open(os.path.join(PKG, 'rafeeq_solution', name + '.py'),
              encoding='utf-8') as handle:
        return handle.read()


def _referenced_attributes(source, module_alias):
    """Every ``alias.attr`` referenced in ``source``."""
    tree = ast.parse(source)
    found = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
            if node.value.id == module_alias:
                found.add(node.attr)
    return found


@pytest.mark.parametrize('consumer', [
    'perception_node', 'mission_node', 'manipulation_node',
    'navigation_node', 'grasp',
])
def test_every_cross_module_attribute_exists(consumer):
    """A node must not reference a helper that no longer exists.

    This is the test that would have caught the live crash where
    ``perception_node`` called ``vision.column_box_from_plate`` after a bad
    patch had removed it: the node came up fine, identified the shelf column
    correctly, and then died with ``AttributeError`` while writing the evidence
    image - taking the rest of the run with it.
    """
    source = _module_source(consumer)
    problems = []
    for name in MODULES:
        if name == consumer:
            continue
        if 'import' not in source or name not in source:
            continue
        module = __import__(
            'rafeeq_solution.' + name, fromlist=[name])
        for attr in _referenced_attributes(source, name):
            if not hasattr(module, attr):
                problems.append('{}.{} used by {}'.format(
                    name, attr, consumer))
    assert not problems, 'missing attributes: ' + ', '.join(sorted(problems))


def test_all_modules_import_cleanly():
    for name in MODULES:
        __import__('rafeeq_solution.' + name, fromlist=[name])


# --------------------------------------------------------------------------
# Gripper geometry
# --------------------------------------------------------------------------

def test_pad_gap_matches_the_measured_curve():
    """Measured: pad gap = 0.0051 + 0.777 * joint."""
    assert arena.pad_gap(0.0) == pytest.approx(0.0051, abs=1e-4)
    assert arena.pad_gap(0.069) == pytest.approx(0.0587, abs=1e-3)


def test_closed_gripper_actually_squeezes_the_spine():
    """The original bug: 'closed' left a gap wider than the book.

    The first version used a closed value of 0.021, which the *fingertip link*
    curve made look reasonable.  Against the real pad surfaces it leaves the
    pads 0.021 m apart - and the book is 0.030 m - only because the two curves
    differ.  Whatever the value, the invariant is that closing must bring the
    pads inside the spine, not around it.
    """
    gap = arena.pad_gap(arena.GRIPPER_CLOSE)
    assert gap < arena.BOOK_SPINE_W, (
        'closing to {} leaves a {:.4f} m pad gap around a {:.3f} m spine - '
        'the fingers would never touch the book'.format(
            arena.GRIPPER_CLOSE, gap, arena.BOOK_SPINE_W))


def test_open_gripper_admits_the_spine_with_clearance():
    gap = arena.pad_gap(arena.GRIPPER_OPEN)
    assert gap > arena.BOOK_SPINE_W + 0.015, (
        'opening to {} gives only {:.4f} m, too tight to enter around a '
        '{:.3f} m spine'.format(arena.GRIPPER_OPEN, gap, arena.BOOK_SPINE_W))


def test_gripper_commands_stay_inside_the_organiser_clamp():
    """The clamp REJECTS out-of-range points; it does not clip them."""
    for value in (arena.GRIPPER_OPEN, arena.GRIPPER_CLOSE):
        assert arena.GRIPPER_MIN <= value <= arena.GRIPPER_MAX


# --------------------------------------------------------------------------
# Arena facts
# --------------------------------------------------------------------------

def test_bin_geometry_is_the_settled_pose_not_the_spawn_pose():
    """The world file spawns the bin at z = 1.3; it falls onto the table."""
    assert arena.BIN_RIM_Z == pytest.approx(0.950, abs=1e-3)
    assert arena.BIN_FLOOR_Z == pytest.approx(0.740, abs=1e-3)
    assert arena.BIN_XYZ[2] < 1.0, (
        'BIN_XYZ still holds the spawn height; a placement solved against it '
        'is ~0.45 m too high')


def test_row_clearance_accounts_for_board_thickness():
    """0.050 m, not ROW_SPACING - BOOK_SPINE_H = 0.080 m."""
    assert arena.ROW_CLEARANCE == pytest.approx(0.050, abs=1e-3)
    for row, board in arena.ROW_BOARD_Z.items():
        ceiling = arena.ROW_CEILING_Z[row]
        assert ceiling - board == pytest.approx(0.300, abs=1e-3)
        book_top = arena.ROW_Z[row] + arena.BOOK_SPINE_H / 2.0
        assert ceiling - book_top == pytest.approx(0.050, abs=0.002)


def test_column_jitter_is_documented_and_large():
    """The organiser jitters books far more than the gripper can span."""
    assert arena.COLUMN_JITTER == pytest.approx(0.25)
    assert arena.COLUMN_JITTER > arena.pad_gap(arena.GRIPPER_MAX), (
        'if this ever became false, aiming at the column centre would be '
        'defensible; it is not')


def test_nearest_row_and_column_snapping():
    for row, z in arena.ROW_Z.items():
        assert arena.nearest_row(z) == row
        assert arena.nearest_row(z + 0.05) == row
    for slot, y in arena.COLUMN_Y.items():
        assert arena.nearest_column_slot(y) == slot
        assert arena.nearest_column_slot(y + 0.24) == slot


def test_book_y_plausible_rejects_a_neighbouring_column():
    assert arena.book_y_plausible(arena.COLUMN_Y[3] + 0.20, 3)
    assert not arena.book_y_plausible(arena.COLUMN_Y[3] + 0.90, 3)


def test_frame_round_trip():
    for wx, wy in ((2.9, 0.23), (-1.0, 0.0), (0.0, 0.0), (1.4, -2.2)):
        ox, oy = arena.world_to_odom(wx, wy)
        rx, ry = arena.odom_to_world(ox, oy)
        assert rx == pytest.approx(wx, abs=1e-9)
        assert ry == pytest.approx(wy, abs=1e-9)


def test_yaw_round_trip():
    for wyaw in (0.0, math.pi, -1.2, 2.5):
        assert arena.odom_yaw_to_world(
            arena.world_yaw_to_odom(wyaw)) == pytest.approx(
                arena.normalize_angle(wyaw), abs=1e-9)


# --------------------------------------------------------------------------
# Depth patch sizing
# --------------------------------------------------------------------------

def test_depth_patch_fits_inside_a_narrow_book_spine():
    """A 9x9 median on a 7 px spine medians in the shelf behind it.

    That inflated the measured range, and because lateral deprojection scales
    with range it put books ~0.5 m sideways - half a column pitch, enough to
    place every book in the wrong column and fail every run at DETECT_BOOK.
    """
    from rafeeq_solution.image_utils import patch_half_for_bbox
    spine = (100, 40, 7, 56)          # a book spine at the 1.5 m scan range
    half = patch_half_for_bbox(spine)
    assert 2 * half + 1 <= spine[2], (
        'patch is {} px across a {} px wide detection'.format(
            2 * half + 1, spine[2]))
    assert half >= 1


def test_depth_patch_stays_useful_for_a_large_detection():
    from rafeeq_solution.image_utils import patch_half_for_bbox
    assert patch_half_for_bbox((0, 0, 200, 200)) == 4      # capped
    assert patch_half_for_bbox((0, 0, 3, 3)) == 1          # floored


# --------------------------------------------------------------------------
# Column association in image space
# --------------------------------------------------------------------------

def _book(u, colour='red'):
    return {'colour': colour, 'centroid': (float(u), 100.0),
            'bbox': (int(u) - 5, 80, 10, 60), 'area': 500.0}


def test_columns_separate_by_image_x():
    """Books of one column share an image x; adjacent columns are ~240 px apart."""
    from rafeeq_solution import vision
    books = [_book(u) for u in (290, 356, 293, 366)] +             [_book(u) for u in (107, 124, 145)]
    clusters = vision.cluster_columns(books)
    assert len(clusters) == 2
    assert sorted(len(c) for c in clusters) == [3, 4]


def test_jitter_within_a_column_does_not_split_it():
    """+/-0.25 m of jitter is about +/-60 px at scan range; that is one column."""
    from rafeeq_solution import vision
    books = [_book(u) for u in (260, 290, 320, 350, 380)]
    assert len(vision.cluster_columns(books)) == 1


def test_the_column_in_front_of_the_robot_is_chosen():
    from rafeeq_solution import vision
    left = [_book(u) for u in (100, 120, 140)]
    middle = [_book(u) for u in (300, 320, 340)]
    right = [_book(u) for u in (520, 540, 560)]
    clusters = vision.cluster_columns(left + middle + right)
    chosen = vision.cluster_nearest(clusters, 320.0)
    assert [b['centroid'][0] for b in chosen] == [300.0, 320.0, 340.0]


def test_clustering_needs_no_robot_pose():
    """The point of doing this in image space.

    A 0.18 rad heading error - measured on this robot - rotates a deprojected
    measurement by 0.25 m at book range without moving the base's own estimate,
    so a world-frame column test does not cancel it. Image x needs no pose.
    """
    from rafeeq_solution import vision
    import inspect
    src = inspect.getsource(vision.cluster_columns)
    for forbidden in ('odom', 'world', 'base_'):
        assert forbidden not in src.split('"""')[2], (
            'cluster_columns must not depend on any pose estimate')


# --------------------------------------------------------------------------
# Choosing which column the books in frame belong to
# --------------------------------------------------------------------------
#
# Live failure this pins down, seed 101, column 2, colour red:
#
#   BOOK CONFIRMED: red book in row 2 ... measured at world (+3.004, +0.467,
#   1.254), +0.467 m off the column centre
#
# Ground truth said the red book of the target column sat at y = -0.154 in the
# top row.  The one actually reported was `book_col_2_row_3_red` at y = +0.969
# - the red book of the *neighbouring* column, which the frame happened to
# centre on because heading drift had swung the shelf across the image.  It was
# picked three times running, so agreement between frames was no defence.  The
# robot strafed to the wrong column and closed the gripper on empty air.


def _placed(cluster, ys, colour='red'):
    return [{'cluster': cluster, 'colour': colour,
             'centroid': (u, 100.0), 'world': (2.9, y, 1.25)}
            for u, y in zip(range(100, 100 + 40 * len(ys), 40), ys)]


def _pick(ref_y, placed, width=640):
    from rafeeq_solution import vision
    return vision.choose_column(placed, ref_y, arena.COLUMN_JITTER + 0.15,
                                width)


def test_the_neighbouring_columns_book_is_not_chosen():
    """The exact live failure: two columns in frame, the wrong one centred."""
    target = _placed(0, [-0.15, 0.14, -0.13])       # the column we were sent to
    neighbour = _placed(1, [0.97, 0.88, 0.83])      # one column over, at +1 m
    chosen, how = _pick(-0.02, target + neighbour)   # plate, measured here
    assert chosen is not None
    assert {b['cluster'] for b in chosen} == {0}, (
        'picked the neighbouring column - the failure this test exists for')
    assert 'plate reference' in how


def test_the_choice_survives_the_drift_that_broke_the_old_one():
    """Drift is common-mode, so the *difference* still points at the column.

    The plate and the books are measured from one stationary pose seconds
    apart, so a 0.5 m pose error moves both together and cancels.  The old
    image-centre test had no such cancellation, which is what put the robot in
    front of the wrong column.
    """
    bias = 0.5
    target = _placed(0, [y + bias for y in (-0.15, 0.14, -0.13)])
    neighbour = _placed(1, [y + bias for y in (0.97, 0.88, 0.83)])
    chosen, _ = _pick(-0.02 + bias, target + neighbour)
    assert {b['cluster'] for b in chosen} == {0}


def test_a_column_further_than_half_the_spacing_is_refused():
    """Better to retry than to grasp from a column we cannot vouch for."""
    chosen, why = _pick(0.0, _placed(0, [1.2, 1.3, 1.25]))
    assert chosen is None
    assert 'nearest_book' in why


def test_a_cluster_straddling_two_columns_is_split_by_the_plate():
    """Seed 404, column 5, green: the cluster mean lied and the book was wrong.

    Perspective closed the image gap between two columns enough that one
    cluster held books from both.  Its mean sat 0.42 m from the plate - inside
    any sane cluster tolerance - while the book it yielded sat 0.755 m away,
    three times the jitter a book can have.  Membership is per book for exactly
    this reason.
    """
    straddling = _placed(0, [0.55, 0.45, 1.30, 1.25])
    chosen, _ = _pick(0.547, straddling)
    assert chosen is not None
    kept = sorted(round(b['world'][1], 2) for b in chosen)
    assert kept == [0.45, 0.55], (
        'kept a book {} m from its plate; books jitter by at most {} m'
        .format(max(abs(b['world'][1] - 0.547) for b in chosen),
                arena.COLUMN_JITTER))


def test_without_a_plate_reference_the_fallback_is_labelled_unreliable():
    """The fallback is the old, broken rule; a run must be traceable to it."""
    chosen, how = _pick(None, _placed(0, [-0.15, 0.14]) + _placed(1, [0.97]))
    assert chosen is not None
    assert 'unreliable' in how


def test_the_book_scan_is_only_entered_from_a_fresh_plate_reading():
    """A reference measured before the drive has already drifted.

    Both routes into the book scan - the first approach and the re-observation
    after a failed grasp - must pass through the column-reference state, or the
    filter silently falls back to the image centre and can pick a neighbour.
    """
    source = open(os.path.join(PKG, 'rafeeq_solution', 'mission_node.py'),
                  encoding='utf-8').read()
    tree = ast.parse(source)
    movers = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef):
            continue
        for child in ast.walk(node):
            if isinstance(child, ast.Constant) and child.value == 'BOOK_POSE':
                movers.add(node.name)
    assert movers == {'check_column_ref'}, (
        'the book scan must be entered only from check_column_ref; these also '
        'move to it: {}'.format(sorted(movers - {'check_column_ref'})))


# --------------------------------------------------------------------------
# Reading the plate directly overhead
# --------------------------------------------------------------------------
#
# The plate above the target column is the one the robot most needs to read -
# it is the only drift-free way to tell that column's books from its
# neighbour's - and it is also the hardest, because standing in front of it
# makes it the steepest plate in the arena.  Seen from 1.5 m away and 0.76 m
# below, the digit is foreshortened vertically: measured live, the digit 2 came
# out 56 x 49 px, an aspect of 1.14.
#
# It matched its own template at 0.827 and was then discarded by an aspect band
# that stopped at 1.05, on the reasoning that "digits 1-5 are all taller than
# wide".  True square on, false from below.  The robot read its neighbours'
# plates, never its own, and fell back to picking a column by image centre.


def _glyph(width, height):
    """The digit 2 at an arbitrary aspect: drawn once, then scaled.

    Scaling one rendering is the point - foreshortening squashes the same
    glyph rather than drawing a different one, which is why the live match
    still scored 0.827 while the aspect gate threw it away.
    """
    cv2 = pytest.importorskip('cv2')
    import numpy as np
    canvas = np.full((120, 80), 255, np.uint8)
    cv2.putText(canvas, '2', (10, 104), cv2.FONT_HERSHEY_SIMPLEX,
                3.4, 0, 8, cv2.LINE_AA)
    return cv2.resize(canvas, (width, height),
                      interpolation=cv2.INTER_AREA)


def _scene(width, height):
    """A white frame with that digit as the only dark blob in it."""
    cv2 = pytest.importorskip('cv2')
    import numpy as np
    frame = np.full((240, 320, 3), 255, np.uint8)
    glyph = cv2.cvtColor(_glyph(width, height), cv2.COLOR_GRAY2BGR)
    frame[60:60 + height, 120:120 + width] = glyph
    return frame


def _templates():
    from rafeeq_solution import vision
    return {2: vision._normalise(vision._binarise_digit(_glyph(40, 64)))}


def test_a_foreshortened_plate_digit_is_still_read():
    """56 x 49 px, aspect 1.14 - the live measurement, from the book pose."""
    from rafeeq_solution import vision
    found = vision.detect_number_plates(_scene(56, 49), _templates())
    assert [d['digit'] for d in found] == [2], (
        'the plate overhead reads wider than tall and must still be read; '
        'without it the book filter has no drift-free column reference')


def test_a_plate_read_square_on_is_unaffected():
    """The long-range case the column scan depends on must not regress."""
    from rafeeq_solution import vision
    found = vision.detect_number_plates(_scene(30, 60), _templates())
    assert [d['digit'] for d in found] == [2]


def test_a_hairline_smear_is_still_not_a_digit():
    """Widening the band must not turn every dark streak into a plate."""
    np = pytest.importorskip('numpy')
    from rafeeq_solution import vision
    frame = np.full((240, 320, 3), 255, np.uint8)
    frame[100:106, 80:240] = 0            # aspect 26.7, far outside the band
    assert vision.detect_number_plates(frame, _templates()) == []


# --------------------------------------------------------------------------
# A measurement that is not at a row height is not a book
# --------------------------------------------------------------------------


def test_a_measurement_between_two_rows_is_not_a_book():
    """Seed 202, column 4, blue, on the look after a failed grasp.

    A blue book confirmed in row 4 at z = 0.597 came back as row 3 at
    z = 1.080 - 0.163 m from the nearest row, and 0.32 m behind the plane the
    books sit in, so the depth patch had landed on shelf structure.  The old
    gate accepted it because it asked only whether z was within *half a row
    spacing* of some row, which every z between two rows is.  The robot drove
    at it and the run failed navigating onto a book that was not there.
    """
    assert not arena.row_height_plausible(1.080), (
        'accepted a measurement 0.163 m from the nearest row')


@pytest.mark.parametrize('measured,row', [
    (1.586, 1), (1.588, 1), (1.580, 1),      # every measured centre from the
    (0.931, 3), (0.936, 3),                  # recorded trials, against the
    (0.569, 4), (0.597, 4),                  # row it really belonged to
])
def test_every_genuine_measurement_is_still_accepted(measured, row):
    assert arena.row_height_plausible(measured)
    assert arena.nearest_row(measured) == row
    assert abs(arena.ROW_Z[row] - measured) < arena.ROW_HEIGHT_TOLERANCE


def test_the_tolerance_leaves_real_headroom_over_the_worst_measurement():
    """Worst live miss was 0.019 m; the gate is at 0.08 m."""
    assert arena.ROW_HEIGHT_TOLERANCE >= 0.019 * 2
    assert arena.ROW_HEIGHT_TOLERANCE < arena.ROW_SPACING / 2.0


# --------------------------------------------------------------------------
# The evidence images a judge actually reads
# --------------------------------------------------------------------------


def test_a_long_caption_line_wraps_instead_of_running_off_the_frame():
    """One evidence image was cut off mid-way through the book's offset."""
    from rafeeq_solution import vision
    line = ('offset from column centre -0.254 m, against the plate re-read '
            'from the book-scan pose, four books in this column')
    rows = vision.wrap_to_width(line, 640 - 8 - 6)
    assert len(rows) > 1
    import cv2
    for row in rows:
        (width, _), _ = cv2.getTextSize(
            row, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        assert width <= 640 - 8 - 6, 'a wrapped row still overflows the frame'
    assert ' '.join(rows) == line, 'wrapping dropped or duplicated words'


def test_a_label_on_a_top_row_book_does_not_land_on_the_caption():
    """A book in the top row has its box against the top of the image.

    The label used to be clamped upwards into the caption block, leaving both
    unreadable on the one image the rules ask a judge to look at.  The box
    itself may sit inside the caption band - that is where the book is - but
    the label, which is far wider than the box, must not.
    """
    np = pytest.importorskip('numpy')
    from rafeeq_solution import vision
    frame = np.zeros((360, 640, 3), np.uint8)
    x, y, w, h = 300, 12, 20, 60
    caption_height = 90

    out = vision.annotate(frame, [{'bbox': (x, y, w, h), 'colour': 'green'}],
                          lambda d: 'GREEN BOOK - ROW 1',
                          avoid_top=caption_height)
    painted = np.argwhere(out.any(axis=2))
    in_caption = painted[painted[:, 0] < caption_height]
    assert len(in_caption), 'the box itself should still be drawn'
    assert in_caption[:, 1].min() >= x - 2 and in_caption[:, 1].max() <= x + w + 2, (
        'something wider than the box was drawn into the caption block - '
        'the label was not moved clear of it')


def test_a_label_near_the_right_edge_stays_inside_the_frame():
    """A book at the right of frame must not push its label off the image."""
    np = pytest.importorskip('numpy')
    from rafeeq_solution import vision
    frame = np.zeros((360, 640, 3), np.uint8)
    x = 600
    out = vision.annotate(frame, [{'bbox': (x, 200, 20, 60), 'colour': 'red'}],
                          lambda d: 'RED BOOK - ROW 3')
    columns = np.argwhere(out.any(axis=2))[:, 1]
    assert columns.max() <= frame.shape[1] - 1, 'drawing ran off the frame'
    assert columns.min() < x, (
        'the label was left at the box, where it would run off the frame')


def test_the_caption_moves_out_of_the_way_of_a_top_row_book():
    """The box around the book is what the rules ask the image to show.

    A book in the top row sits directly behind a four-line caption.  With the
    caption at the top, the box is legible only through the text; moved to the
    foot of the frame, the top of the image is clear for it.
    """
    np = pytest.importorskip('numpy')
    from rafeeq_solution import vision
    frame = np.zeros((360, 640, 3), np.uint8)
    caption = ['Rafeeq / Team Kanban  20260909_233044_902',
               'target: red book in row 1 (row from ordering confirmed by height)',
               'measured world (+2.794, -0.586, 1.580) m',
               'column chosen by plate reference (y -0.425, 4 book(s) within 0.40 m)',
               'offset from column centre -0.161 m, against the measured plate']
    band = vision.stamp_height(frame, caption)
    assert band > 40, 'a five-line caption should occupy a real band'

    top = vision.stamp(frame, caption, at_bottom=False)
    bottom = vision.stamp(frame, caption, at_bottom=True)

    def ink_rows(img):
        return np.argwhere(img.any(axis=2))[:, 0]

    assert ink_rows(top).min() < band, 'caption was not at the top'
    assert ink_rows(bottom).min() > frame.shape[0] // 2, (
        'caption did not move to the foot of the frame')
    assert ink_rows(bottom).max() <= frame.shape[0] - 1, (
        'caption ran off the bottom of the frame')


# --------------------------------------------------------------------------
# The drive tucks
# --------------------------------------------------------------------------


def _arm_poses():
    yaml = pytest.importorskip('yaml')
    path = os.path.join(PKG, 'config', 'arm_poses.yaml')
    with open(path, encoding='utf-8') as handle:
        return yaml.safe_load(handle)['arm_poses']


def test_the_left_tuck_mirrors_the_right_one():
    """The left arm had a tuck of its own, and it hit the table every run.

    It put the wrist out at a 0.516 m radius, hanging at world z = 0.668 -
    below the 0.740 m table top - so the turn to face the shelf swept it to
    within 0.084 m of the table, less than the link's own radius.  That was
    -0.5 on all five recorded trials and the only collision in the set.

    Joints 2-7 are geometrically identical on both arms in the URDF and only
    joint 1 is mounted mirrored, with exactly negated limits, so the right
    arm's tuck - which has never collided - transfers directly.  Measured on
    the simulator afterwards, the worst approach went from 0.084 m to 0.425 m.
    """
    poses = _arm_poses()
    right, left = poses['tucked'], poses['tucked_left']
    assert len(left) == len(right) == 7
    assert left[1:] == right[1:], (
        'the left tuck no longer mirrors the right one; the shape that clears '
        'the table is q2..q7 of the right arm')
    assert left[0] == pytest.approx(-right[0], abs=1e-6), (
        'joint 1 is mounted mirrored, so its angle must be negated')


def test_both_tucks_keep_the_grippers_behind_the_base():
    """The reason both arms are tucked at all.

    An arm left forward sits inside the shelf when the base parks to grasp -
    12 852 fingertip/shelf contacts in a single move, before this existed.
    """
    poses = _arm_poses()
    for name in ('tucked', 'tucked_left'):
        assert len(poses[name]) == 7
    # Shoulder-lift well back, elbow folded: the shape that keeps the wrist in.
    for name in ('tucked', 'tucked_left'):
        assert poses[name][1] < -1.5, (
            '{} no longer folds the shoulder back'.format(name))


# --------------------------------------------------------------------------
# Passing the book on the way to grasping it
# --------------------------------------------------------------------------
#
# T02 and T05 of the recorded batch each reported arm_right_6_link striking the
# *target book* during NAV_GRASP - the wrist knocking the book over before the
# fingers arrived.  The arm is solved once and held rigid while the base drives
# in, so the whole approach is that one posture swept along the shelf normal,
# and the wrist sits 0.228 m deeper into the shelf than the grasping point: it
# passes the book on the way past.
#
# The planner never looked.  It ranked candidates on shelf penetration alone,
# took the first tilt that scored a clean zero, and that plan cleared the book
# by 0.065 m - less than the link's own 0.05 m radius plus the error in a
# measured book position.  On the same book, tilt +70 with the opposite wrist
# roll clears it by 0.179 m for the same clean zero against the shelf.


def _intrusion(point, centre, margin):
    grasp_mod = pytest.importorskip('rafeeq_solution.grasp')
    return grasp_mod.GraspPlanner.book_intrusion(point, centre, margin)


def test_the_margin_covers_the_link_radius_and_then_some():
    """A gap smaller than the link's radius is not a gap."""
    grasp_mod = pytest.importorskip('rafeeq_solution.grasp')
    assert grasp_mod.GraspPlanner.BOOK_MARGIN >= (
        2.0 * grasp_mod.GraspPlanner.LINK_RADIUS - 1e-9), (
        'the clearance model walks link origins, so the margin has to carry '
        'the link radius before it carries any measurement error')


def test_the_plan_that_collided_is_now_rejected():
    """0.065 m from the box was the plan the simulator reported hitting."""
    grasp_mod = pytest.importorskip('rafeeq_solution.grasp')
    margin = grasp_mod.GraspPlanner.BOOK_MARGIN
    centre = (2.758, -0.997, 0.936)
    # A wrist passing 0.065 m to the side of the spine, level with the book.
    point = (centre[0], centre[1] + arena.BOOK_SPINE_W / 2.0 + 0.065, centre[2])
    assert _intrusion(point, centre, margin) > 0.005, (
        'the 0.065 m plan still passes; it is the one that struck the book')


def test_the_clearance_that_replaced_it_is_accepted():
    grasp_mod = pytest.importorskip('rafeeq_solution.grasp')
    margin = grasp_mod.GraspPlanner.BOOK_MARGIN
    centre = (2.758, -0.997, 0.936)
    point = (centre[0], centre[1] + arena.BOOK_SPINE_W / 2.0 + 0.179, centre[2])
    assert _intrusion(point, centre, margin) <= 0.0


def test_a_point_well_clear_of_the_book_is_clear_on_every_axis():
    grasp_mod = pytest.importorskip('rafeeq_solution.grasp')
    margin = grasp_mod.GraspPlanner.BOOK_MARGIN
    centre = (2.758, -0.997, 0.936)
    for axis, half in ((0, arena.BOOK_DEPTH / 2.0),
                       (1, arena.BOOK_SPINE_W / 2.0),
                       (2, arena.BOOK_SPINE_H / 2.0)):
        point = list(centre)
        point[axis] += half + margin + 0.01
        assert _intrusion(point, centre, margin) < 0.0, (
            'clear on axis {} was reported as an intrusion'.format(axis))
        point[axis] = centre[axis] + half + margin - 0.01
        assert _intrusion(point, centre, margin) > 0.0


def test_the_book_is_ranked_alongside_the_shelf_not_after_it():
    """Ranking on the shelf alone always chose the first tilt tried."""
    import inspect
    grasp_mod = pytest.importorskip('rafeeq_solution.grasp')
    src = inspect.getsource(grasp_mod.GraspPlanner.plan_grasp)
    assert 'book_offender' in src, 'plan_grasp no longer checks the book'
    assert "max(info.get('shelf_penetration', 0.0), book_depth)" in src, (
        'the book has to gate acceptance, not just be recorded')


# --------------------------------------------------------------------------
# Measuring how far away the book is
# --------------------------------------------------------------------------
#
# Measured book x scattered by +/-0.25 m against a true 2.900.  Two separate
# faults, needing opposite fixes:
#
#   * a median over the depth patch mixed the spine with the shelf behind it
#     and read *between* them - the face, really at 2.820, came back anywhere
#     from 2.822 to 2.905 on seven books;
#   * nothing then converted the face the camera can see into the centre every
#     consumer assumes, so the grasp aimed 0.08 m short.


def _depth_patch(values):
    np = pytest.importorskip('numpy')
    return np.array(values, dtype=float)


def test_background_bleed_no_longer_drags_the_reading_backwards():
    """A patch straddling the spine edge must read the spine, not the blend.

    The spine is about 7 px wide at scan range, so even a fitted patch catches
    the shelf at its edges, and the edge pixels themselves come back as a blend
    of the two ranges.  A median then lands on the blend - which is how a face
    really at 2.820 m came back anywhere up to 2.905 m.
    """
    np = pytest.importorskip('numpy')
    from rafeeq_solution.image_utils import patch_depth
    book, edge, shelf = 1.50, 1.62, 1.78
    values = [book] * 11 + [edge] * 3 + [shelf] * 11
    patch = np.array(values, dtype=float).reshape(5, 5)

    assert float(np.median(patch)) == pytest.approx(edge), (
        'this patch does not reproduce the failure, so it proves nothing'
    )
    got = patch_depth(patch, 2, 2, half=2)
    assert got == pytest.approx(book, abs=0.01), (
        'read {:.3f} for a surface at {:.3f}'.format(got, book)
    )


def test_a_single_near_outlier_does_not_win():
    """Why a low order statistic and not the minimum."""
    np = pytest.importorskip('numpy')
    from rafeeq_solution.image_utils import patch_depth
    patch = np.full((5, 5), 1.50)
    patch[0, 0] = 0.42                        # one bad pixel
    assert patch_depth(patch, 2, 2, half=2) == pytest.approx(1.50, abs=0.02)


def test_a_uniform_patch_is_unchanged():
    np = pytest.importorskip('numpy')
    from rafeeq_solution.image_utils import patch_depth
    patch = np.full((5, 5), 1.234)
    assert patch_depth(patch, 2, 2, half=2) == pytest.approx(1.234, abs=1e-6)


def test_nans_are_still_ignored():
    np = pytest.importorskip('numpy')
    from rafeeq_solution.image_utils import patch_depth
    patch = np.full((5, 5), np.nan)
    assert patch_depth(patch, 2, 2, half=2) is None
    patch[3, 3] = 1.6
    assert patch_depth(patch, 2, 2, half=2) == pytest.approx(1.6, abs=1e-6)


def test_the_published_book_point_is_the_centre_not_the_face():
    """The camera sees the face; every consumer assumes the centre.

    `grasp.GRASP_DEPTH` grips 0.06 m in front of what it is handed, so the
    face put the fingers 0.06 m in front of the book instead of 0.02 m inside
    its front face.  The same number anchors the shelf plane the arm's whole
    clearance model hangs off.
    """
    import inspect
    perception = os.path.join(PKG, 'rafeeq_solution', 'perception_node.py')
    src = open(perception, encoding='utf-8').read()
    body = src.split('def _do_book')[1].split('def _pick_column')[0]
    assert 'arena.BOOK_DEPTH / 2.0' in body, (
        'the measured spine face is being published as the book position'
    )
    # And the grip lands inside the book once the centre is used.
    grasp_mod = pytest.importorskip('rafeeq_solution.grasp')
    depth = grasp_mod.GraspPlanner.GRASP_DEPTH
    assert -arena.BOOK_DEPTH / 2.0 < depth < 0.0, (
        'the grip must sit inside the book, measured from its centre'
    )


def test_the_base_keeps_its_clearance_from_the_shelf():
    """Correcting the book depth moved the base towards the shelf.

    The standoff is measured from the book, and the book moved 0.08 m deeper
    when perception started reporting the centre instead of the spine face, so
    the base followed it in.  Its leading edge sits 0.281 m ahead of base_link;
    at the old 0.52 m standoff that left 0.094 m of clearance, and one trial's
    drift ate all of it and put base_footprint into erc_shelf.
    """
    base_front = 0.281
    base_x = arena.BOOK_X - arena.GRASP_STANDOFF
    clearance = arena.SHELF_FRONT_X - (base_x + base_front)
    assert clearance > 0.15, (
        'base clears the shelf by only {:.3f} m; drift alone has been '
        'measured at 0.094 m'.format(clearance)
    )


def test_the_arm_is_still_moved_into_posture_out_of_reach_of_the_shelf():
    """The transit standoff has to stay well beyond the grasp standoff."""
    assert arena.ARM_TRANSIT_STANDOFF > arena.GRASP_STANDOFF + 0.5, (
        'the arm would take its grasp posture close enough to touch the shelf'
    )


# --------------------------------------------------------------------------
# Getting the pads around the spine without knocking the book away
# --------------------------------------------------------------------------


def test_the_lateral_tolerance_is_inside_the_fingers_clearance():
    """Measured at the grasp pose, with the gripper at its widest.

        BOOK             y[0.2157, 0.2457]
        fingertip_left   y[0.1682, 0.1995]   -> 0.0162 m clear
        fingertip_right  y[0.2585, 0.2898]   -> 0.0128 m clear

    The navigation goal tolerance was 0.02 m and radial, so the base could
    legitimately arrive with 0.02 m of lateral error - more than the 0.0128 m
    the tighter finger has - which put a fingertip against the spine and then
    dragged the book 0.047 m deeper as the base finished advancing.  The pads
    then closed on empty space 0.015 m in front of where the book had gone.
    """
    import re
    path = os.path.join(PKG, 'rafeeq_solution', 'navigation_node.py')
    source = open(path, encoding='utf-8').read()
    m = re.search(
        r"declare_parameter\('precise_lateral_tolerance',\s*([0-9.]+)\)",
        source)
    assert m, 'the lateral tolerance is gone; the fingers will clip the spine'
    lateral = float(m.group(1))
    tightest_finger_clearance = 0.0128
    assert lateral < tightest_finger_clearance, (
        'lateral tolerance {:.4f} m exceeds the {:.4f} m the fingers clear the '
        'spine by'.format(lateral, tightest_finger_clearance))
    assert lateral <= tightest_finger_clearance * 0.75, (
        'no margin left for anything but a perfect measurement'
    )


def test_arrival_is_judged_along_and_across_the_heading_separately():
    """They are not equally forgiving, so one radial number cannot serve.

    Along the approach, an error only changes how deep the pads grip - they
    span 0.057 m.  Across it, the same error puts a finger into the book.
    """
    path = os.path.join(PKG, 'rafeeq_solution', 'navigation_node.py')
    source = open(path, encoding='utf-8').read()
    assert 'self.lat_tol' in source and 'self.pos_tol' in source, (
        'arrival is back to a single radial tolerance')
    assert 'across' in source and 'along' in source


def test_only_the_grasp_approach_asks_for_millimetre_precision():
    """Holding every leg to millimetres is not free.

    Tightening the tolerance globally made a long traverse fight the obstacle
    checker instead of arriving: a trial failed with "the path is blocked while
    driving to column 2 and did not clear" that had never failed that way.  The
    precision is requested per goal, and only the grasp approach needs it.
    """
    import ast
    path = os.path.join(PKG, 'rafeeq_solution', 'mission_node.py')
    tree = ast.parse(open(path, encoding='utf-8').read())
    asked = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef):
            continue
        for call in ast.walk(node):
            if not isinstance(call, ast.Call):
                continue
            if getattr(call.func, 'attr', None) != 'goto':
                continue
            for kw in call.keywords:
                if kw.arg == 'precise' and getattr(kw.value, 'value', False):
                    asked.add(node.name)
    assert asked == {'enter_nav_grasp'}, (
        'precision is requested by {} - it costs convergence time on every '
        'leg that asks for it'.format(sorted(asked) or 'nothing'))


def test_the_refine_keeps_its_correction_and_the_lift_follows_it():
    """Two ways the refine could silently undo itself.

    Its arm step used the 0.10 rad default tolerance - about 0.015 m at the
    gripper, the whole lateral error it exists to remove - and LIFT replayed a
    posture solved before the refine, carrying the arm back to the
    uncorrected position with the book already in the jaw.
    """
    path = os.path.join(PKG, 'rafeeq_solution', 'manipulation_node.py')
    src = open(path, encoding='utf-8').read()
    body = src.split('def build_refine_grasp')[1].split('def build_close_grasp')[0]
    assert 'tolerance=GRASP_POSTURE_TOLERANCE' in body, (
        'refine is back on the default tolerance')
    assert "name='lift refine'" in body, (
        'the lift is no longer re-solved from the refined posture')


# --------------------------------------------------------------------------
# Which frame a colour pixel belongs to
# --------------------------------------------------------------------------


def test_colour_pixels_are_deprojected_from_where_they_were_rendered():
    """The 0.015 m that kept the pads beside the book instead of around it.

    Both head-camera sensors are mounted on head_front_camera_link with no
    pose, so Gazebo renders colour and depth from the same point, pixel-aligned
    with identical intrinsics.  The colour image is only *labelled*
    head_front_camera_color_optical_frame, which TF puts 0.015 m to the side of
    the link.  Deprojecting in that label shifted every measurement sideways:
    against ground truth, the target book came back 0.019 m off relative to the
    robot and the gripper closed 0.0141 m off a spine it clears by 0.0144 m.
    It hid from the plate check because the plate carried the same offset.
    """
    path = os.path.join(PKG, 'rafeeq_solution', 'perception_node.py')
    src = open(path, encoding='utf-8').read()
    assert "declare_parameter('deproject_frame'," in src
    assert "'head_front_camera_depth_optical_frame')" in src, (
        'the deprojection frame is no longer the render origin')
    body = src.split('    def world_point')[1].split('    def _do_column')[0]
    assert 'self.deproject_frame' in body, (
        'world_point is trusting the colour header frame again')


def test_no_disparity_is_applied_between_pixel_aligned_images():
    """The simulator does not model the RealSense baseline, so shifting the
    depth lookup by fx * 0.015 / range sampled the wrong pixel - 3.4 px at
    1.5 m, on a spine that is only about 7 px wide."""
    path = os.path.join(PKG, 'rafeeq_solution', 'perception_node.py')
    src = open(path, encoding='utf-8').read()
    assert "declare_parameter('rgb_depth_baseline', 0.0)" in src


def _stub_planner(reachable, lift_ok):
    """A GraspPlanner whose IK is replaced by a table of what is reachable."""
    import math
    import numpy as np
    from rafeeq_solution import grasp
    planner = grasp.GraspPlanner(chain=None)
    planner.check_target = lambda *a, **k: None
    planner.warm_up = lambda: None
    planner.grasp_candidates = lambda tz: [(+1.0, -0.054), (+1.0, 0.0),
                                           (-1.0, 0.072)]
    planner.lock_torso = lambda value: None
    planner.unlock_torso = lambda saved: None
    planner.torso_for = lambda z: 0.35

    def solve_clear(target, base_xy, base_yaw, direction, sign, **kw):
        tilt = int(round(math.degrees(math.atan2(direction[2], direction[0]))))
        if (tilt, sign) not in reachable:
            raise grasp.PlanFailure('unreachable')
        return np.zeros(8), {'position_error': 0.003, 'finger_sign': sign,
                             'shelf_penetration': 0.0}

    planner.solve_clear = solve_clear
    planner.book_offender = lambda *a: None
    planner.shelf_offender = lambda *a: None
    planner._solve_lift = lambda q, grasp_world, base_xy, base_yaw, approach, \
        sign: ((np.zeros(8), 0.02) if lift_ok(sign) else (None, 0.0))
    return planner


def test_the_book_is_hung_from_above_its_centre_of_mass():
    """Pinched below its centre, the book is an inverted pendulum.

    The pads only reach the front of the book, so once its rear leaves the
    board it swings about the pinch.  Measured in truth with the pinch 0.054 m
    below centre: out 0.17 m still held, then flipped over and dropped.  An
    above-centre grasp at a later tilt must win over a below-centre one at the
    first tilt tried.
    """
    planner = _stub_planner({(90, +1.0), (-70, -1.0)}, lambda sign: True)
    plan = planner.plan_grasp((2.9, -0.155, 1.578), (2.26, -0.33), 0.0, slot=3)
    assert plan.height_offset > 0.0, plan.height_offset
    assert plan.info['approach_tilt_deg'] == -70
    assert plan.lift_q is not None


def test_an_above_centre_grasp_without_a_lift_is_not_taken():
    """With no lift the rear drags on the board and drops off its edge -
    exactly the swing the above-centre grasp exists to tame."""
    planner = _stub_planner({(90, +1.0), (-90, -1.0)}, lambda sign: sign > 0)
    plan = planner.plan_grasp((2.9, -0.155, 1.578), (2.26, -0.33), 0.0, slot=3)
    assert plan.height_offset <= 0.0
    assert plan.info['approach_tilt_deg'] == 90


def test_the_book_in_our_gripper_cannot_hide_the_bin():
    """Seen side-on from half a metre, the red book we are carrying is broad,
    solid and bigger than the bin.  Picking only the largest blob would lock
    the bin search onto it for good; every bin-shaped blob must be offered to
    the geometric check, largest first."""
    import numpy as np
    import cv2
    from rafeeq_solution import vision
    frame = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(frame, (380, 20), (540, 270), (0, 0, 255), -1)   # held book
    cv2.rectangle(frame, (100, 300), (300, 400), (0, 0, 255), -1)  # the bin
    found = vision.red_bin_candidates(frame)
    assert len(found) == 2, found
    assert found[0]['area'] > found[1]['area']
    assert vision.detect_red_bin(frame)['bbox'] == found[0]['bbox']

    path = os.path.join(PKG, 'rafeeq_solution', 'perception_node.py')
    src = open(path, encoding='utf-8').read()
    body = src.split('    def _do_bin')[1].split('    def _settled')[0]
    assert 'red_bin_candidates' in body and 'for blob in candidates' in body, (
        'the bin search is back to trusting the single largest red blob')


def test_a_velocity_ramp_never_steps():
    from rafeeq_solution.arena import ramp
    assert ramp(0.0, 0.6, 0.0125) == 0.0125
    assert ramp(0.3, -0.3, 0.05) == 0.25
    assert ramp(0.10, 0.11, 0.05) == 0.11
    speed, steps = 0.0, 0
    while speed < 0.25:
        speed = ramp(speed, 0.25, 0.25 * 0.05)
        steps += 1
    assert steps == 20          # 1 s to reach 0.25 rad/s at 0.25 rad/s^2


def test_the_book_is_carried_gently():
    """An unramped start to the turn back to the bin threw out a book that
    had survived the whole extraction.  Both legs driven with the book in the
    jaw must ask for gentle motion, and the navigation node must honour it."""
    path = os.path.join(PKG, 'rafeeq_solution', 'mission_node.py')
    src = open(path, encoding='utf-8').read()
    for state in ('    def enter_retreat(', '    def enter_nav_bin_area(',
                  '    def enter_nav_bin('):
        body = src.split(state)[1].split('\n    def ')[0]
        assert 'gentle=True' in body, state
    nav = open(os.path.join(PKG, 'rafeeq_solution', 'navigation_node.py'),
               encoding='utf-8').read()
    assert "Bool, '/rafeeq/nav/gentle'" in nav
    control = nav.split('    def control(')[1].split('\n    def ')[0]
    assert 'self.goal_gentle' in control and '_ramp_up(' in control


def test_the_highest_above_centre_grasp_is_preferred():
    """The higher the pinch above the centre of mass, the smaller the swing as
    the book leaves the board.  At +0.054 m it swung twice as far as at
    +0.072 m and slipped out of the pads during the retreat itself."""
    planner = _stub_planner({(-90, -1.0)}, lambda sign: True)
    planner.grasp_candidates = lambda tz: [(-1.0, 0.054), (-1.0, 0.063),
                                           (-1.0, 0.072)]
    plan = planner.plan_grasp((2.9, -0.155, 1.578), (2.26, -0.33), 0.0, slot=3)
    assert plan.height_offset == 0.072, plan.height_offset


def test_the_bin_placement_steps_back_from_the_far_side_it_sees():
    """Measured: from the start zone the bin blob's depth lands on the far
    side of the interior (x = -1.201 against a middle of -0.975), so the
    middle is half an extent back along +x.  Subtracting it drove the target
    into the far clip and the robot's torso towards the table."""
    path = os.path.join(PKG, 'rafeeq_solution', 'perception_node.py')
    src = open(path, encoding='utf-8').read()
    body = src.split('    def _do_bin')[1].split('    def _settled')[0]
    assert 'world[0] + (arena.BIN_X[1] - arena.BIN_X[0]) / 2.0' in body


def test_the_carry_to_the_bin_is_quick_but_ramped():
    nav = open(os.path.join(PKG, 'rafeeq_solution', 'navigation_node.py'),
               encoding='utf-8').read()
    assert "Bool, '/rafeeq/nav/smooth'" in nav
    control = nav.split('    def control(')[1].split('\n    def ')[0]
    assert 'self.goal_smooth' in control
    mission = open(os.path.join(PKG, 'rafeeq_solution', 'mission_node.py'),
                   encoding='utf-8').read()
    body = mission.split('    def enter_carry_to_bin(')[1].split('\n    def ')[0]
    assert 'smooth=True' in body


def test_the_gripper_height_is_reported_in_the_world():
    """The IK chain is rooted at base_link, which stands 0.0762 m above the
    ground, but the mission's rim thresholds are world heights.  Reporting the
    raw chain z put every one of them 0.076 m low."""
    path = os.path.join(PKG, 'rafeeq_solution', 'manipulation_node.py')
    src = open(path, encoding='utf-8').read()
    body = src.split('    def _verify_retention_step')[1].split(
        '    def build_check_hold')[0]
    assert 'arena.BASE_LINK_Z' in body


def test_the_carry_ramp_works_in_the_odom_frame():
    """Ramping vx and vy in the body frame while the base turns 180 degrees
    makes the command chase its own rotation - simulated, 0.29-0.71 m off the
    straight line and up to 2.7 s slower."""
    nav = open(os.path.join(PKG, 'rafeeq_solution', 'navigation_node.py'),
               encoding='utf-8').read()
    control = nav.split('    def control(')[1].split('\n    def ')[0]
    ramp = control.split('if self.goal_gentle or self.goal_smooth:')[1]
    assert 'math.cos(yaw)' in ramp, 'the ramp is back in the body frame'
    assert '_ramp_up(' in ramp
    assert 'math.hypot(ox, oy)' in ramp, 'speed is capped per axis again'
    # Growth is rate-limited; decay is not, or the base cannot stop in time
    # from the carry's top speed and coasts into the table.
    assert 'def _ramp_up(' in nav
