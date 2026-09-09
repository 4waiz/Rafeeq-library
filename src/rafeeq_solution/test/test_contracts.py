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
