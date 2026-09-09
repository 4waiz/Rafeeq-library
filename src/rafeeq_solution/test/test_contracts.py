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
