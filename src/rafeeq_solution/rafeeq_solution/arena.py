"""Arena facts and frame helpers for the ERC 2026 library arena.

Everything here is **documented prior information**, not a substitute for
perception.  The constants describe the parts of the arena that genuinely never
move (shelf plane, column pitch, bin and table geometry, robot frame offsets)
and they are used for sanity checks, snapping and fallbacks.  The parts the
organiser randomises are *measured at run time* - see ``perception_node`` and
``grasp.py``.

Provenance: every value is either read out of the competition sources or
measured from the running simulator.  See ``docs/measured_environment.md`` for
the measurements and the baseline they were taken against.

What ``simulation.launch.py`` actually randomises (all three, not one):

1. ``random.shuffle(shelf_number_markers)`` - which digit hangs over which
   physical column.
2. ``random.shuffle(colours_this_column)`` **per column** - so every column
   holds all four colours, one per row, and the target colour sits in a
   different row in every column and in every run.
3. ``random.uniform(-COLUMN_JITTER, +COLUMN_JITTER)`` on each book's world
   ``y`` - **+/- 0.25 m**, which is far more than the gripper's 0.082 m
   maximum opening.  A grasp aimed at a column centre misses.
"""

import math

# --- Shelf -------------------------------------------------------------
SHELF_X = 3.0
SHELF_Y = 0.0
SHELF_Z = 1.1
NUM_COLUMNS = 5
COLUMN_WIDTH = 1.0

#: Measured world bounds of the shelf mesh.  The **front face is at 2.755**,
#: so books (front face 2.820) sit 0.065 m recessed inside it.
SHELF_FRONT_X = 2.755
SHELF_BACK_X = 3.105

#: Measured z of the top surface of the board each row of books stands on,
#: keyed by competition row 1..4.  Boards are 0.030 m thick.
ROW_BOARD_Z = {1: 1.452, 2: 1.122, 3: 0.792, 4: 0.462}
#: Measured z of the underside of the board *above* each row.
ROW_CEILING_Z = {1: 1.752, 2: 1.422, 3: 1.092, 4: 0.762}
#: Vertical gap between the top of a book and the board above it: 0.050 m.
#: This is smaller than ``ROW_SPACING - BOOK_SPINE_H`` because the boards
#: themselves are 0.030 m thick.  Any lift must stay well inside it.
ROW_CLEARANCE = 0.050

#: World ``y`` of each column *centre*, indexed by physical shelf slot 1..5.
#: From ``COLUMN_Y_OFFSETS = [((5 - 1) / 2 - col) * 1.0 for col in range(5)]``.
COLUMN_Y = {1: 2.0, 2: 1.0, 3: 0.0, 4: -1.0, 5: -2.0}

#: Books are offset from the column centre by up to this much in world y.
#: ``COLUMN_JITTER_RANGE = COLUMN_WIDTH * 0.25`` in simulation.launch.py.
COLUMN_JITTER = 0.25

# --- Number marker plates ---------------------------------------------
#: Plates hang above the shelf, one per column, flush with the shelf face.
#: Unlike the books these are **not** jittered - measured at exactly the
#: column centres.
MARKER_X = SHELF_X - 0.245          # 2.755
MARKER_Z = 2.26
MARKER_SIZE = 0.30

# --- Books -------------------------------------------------------------
#: Book box is 0.25 x 0.03 x 0.16 spawned with pitch = 90 deg, so the face
#: pointing at the robot is the *spine*: 0.03 m wide by 0.25 m tall.
BOOK_SPINE_W = 0.03
BOOK_SPINE_H = 0.25
BOOK_DEPTH = 0.16
BOOK_X = SHELF_X - 0.1              # 2.9, book centre plane

#: Measured settled book-centre height for competition rows 1..4, top first.
#: The organiser model names use physical shelf rows 2..5; competition row N
#: is model row N+1.
ROW_Z = {1: 1.5769, 2: 1.2469, 3: 0.9169, 4: 0.5869}
ROW_SPACING = 0.33

BOOK_COLOURS = ('red', 'green', 'blue', 'yellow')

# --- Collection bin and table -----------------------------------------
#: MEASURED settled pose.  The world SDF *spawns* the bin at z = 1.3 and it
#: drops onto the table; the old code used the spawn pose and was 0.455 m out.
BIN_XYZ = (-0.975, 0.0, 0.845)
#: World axis-aligned bounds of the bin mesh, computed from the STL.
BIN_X = (-1.255, -0.695)
BIN_Y = (-0.155, 0.155)
BIN_Z = (0.740, 0.950)
#: Top of the bin wall - a book must clear this before it can be released.
BIN_RIM_Z = 0.950
#: Inside floor of the bin (it is the table top).
BIN_FLOOR_Z = 0.740

TABLE_X = (-1.400, -0.600)
TABLE_Y = (-0.700, 0.700)
TABLE_TOP_Z = 0.740

START_XY = (0.0, 0.0)

#: Robot spawn yaw in the world frame (``simulation.launch.py`` spawns with
#: ``-Y 1.5708``).  Odometry starts at identity at the spawn pose, so this is
#: the rotation between ``odom`` and ``world``.  ``navigation_node`` confirms
#: it from observations at start-up and only falls back to this value.
SPAWN_YAW = 1.5708

# --- Robot frame offsets (measured from TF) ---------------------------
#: ``base_footprint`` -> ``base_link`` translation in z.  The IK chain is
#: rooted at ``base_link``, so a world z must be reduced by this to become an
#: IK target height when the robot is on level ground.
BASE_LINK_Z = 0.0762

#: Approximate base footprint radius, used for clearance reasoning.
BASE_RADIUS = 0.31

# --- Gripper (measured, see docs/measured_environment.md) -------------
#: Commandable range enforced by the organiser's ``gripper_command_clamp``,
#: which **rejects** rather than clips anything outside it.
GRIPPER_MIN = 0.0
GRIPPER_MAX = 0.069
#: Fingertip gap when the joint is at zero.
GRIPPER_GAP_AT_ZERO = 0.0280
#: Fingertip gap gained per unit of joint travel.
GRIPPER_GAP_PER_UNIT = 0.783


def gripper_gap(joint):
    """Fingertip gap in metres for a finger-joint position."""
    return GRIPPER_GAP_AT_ZERO + GRIPPER_GAP_PER_UNIT * float(joint)


def gripper_joint_for_gap(gap):
    """Finger-joint position that yields ``gap`` metres between the fingers.

    Clamped into the organiser's accepted command range.  A 0.03 m book would
    need a negative joint value to be squeezed, which is not commandable, so
    the answer for a book is simply 0.0 - a 2 mm interference on a 30 mm
    spine, which is the intended friction grip.
    """
    raw = (float(gap) - GRIPPER_GAP_AT_ZERO) / GRIPPER_GAP_PER_UNIT
    return max(GRIPPER_MIN, min(GRIPPER_MAX, raw))


#: MEASURED separation of the actual gripping **pads** (not the fingertip link
#: origins, which sit 22 mm outboard of them and gave a misleading curve).
#: Transforming the fingertip collision mesh into the grasp frame gives
#: pad gap = 0.0051 + 0.777 * joint.
PAD_GAP_AT_ZERO = 0.0051
PAD_GAP_PER_UNIT = 0.777


def pad_gap(joint):
    """Separation of the gripping pads for a finger-joint position."""
    return PAD_GAP_AT_ZERO + PAD_GAP_PER_UNIT * float(joint)


def joint_for_pad_gap(gap):
    """Finger-joint position that puts the pads ``gap`` metres apart."""
    raw = (float(gap) - PAD_GAP_AT_ZERO) / PAD_GAP_PER_UNIT
    return max(GRIPPER_MIN, min(GRIPPER_MAX, raw))


#: Joint value used to clamp onto a book.  The pads meet a 0.030 m spine at
#: joint ~0.032, so this squeezes ~0.016 m into it and loads the friction
#: surfaces (mu = 5.0 on the book collision).
#:
#: The squeeze has to be generous because the gripper does not end up perfectly
#: centred on the spine - measured residual about 0.003 m.  At only 0.006 m of
#: interference that offset left one pad pressing and the other barely
#: reaching, so the simulator reported contact on a single fingertip and the
#: book was never actually clamped.
#:
#: Note this is a *position* command, and gz_ros2_control implements position
#: commands as a velocity setpoint, so the joint reaches whatever it is told to
#: reach: the finger will not stall on the book.  That is why grasp
#: verification reads the contact sensors instead of the joint position.
GRIPPER_CLOSE = 0.014
#: Joint value used to present the open gripper at a book.  Deliberately the
#: maximum the organiser's clamp accepts: it gives a 0.0587 m pad gap, i.e.
#: 14.4 mm of clearance either side of a 0.030 m spine instead of 11 mm, and
#: every millimetre of that matters when the base is carrying the arm in.
GRIPPER_OPEN = 0.069
#: A measured finger position this far above ``GRIPPER_CLOSE`` means the pads
#: stalled on something.  Below it, they closed to the commanded position and
#: there is nothing between them.
GRIPPER_EMPTY_EPS = 0.004

# --- Gripper envelope in the grasp frame (measured from the collision
# --- meshes transformed into ``gripper_right_grasping_link``).
#: The gripper body reaches 0.157 m along the grasp frame's local -x and only
#: 0.017 m along +x.  With the fingers horizontal, local x is **vertical** in
#: the world, so the body hangs 0.157 m either above or below the grasp point
#: depending on which way the wrist is rolled.  Getting this wrong drives the
#: gripper straight into the shelf board - measured as 1950 tiago_pro/erc_shelf
#: contacts per second with the fingers jammed and unable to open or close.
GRIPPER_BODY_BACK = 0.157
GRIPPER_BODY_FRONT = 0.017
#: Half-extent along the approach axis.
GRIPPER_APPROACH_HALF = 0.038
#: The pads themselves, along the same local x, relative to the grasp point.
GRIPPER_PAD_X = (-0.040, 0.017)

# --- Arena bounds (walls), used for waypoint sanity checks ------------
ARENA_X = (-4.0, 6.0)
ARENA_Y = (-5.0, 5.0)

# --- Standoff distances ----------------------------------------------
#: Where to park to read the number plates: far enough back that all five
#: plates fit in the 87 deg horizontal FOV.
SCAN_STANDOFF_X = 0.0
#: Where to park to see all four book rows of one column at once.  The rows
#: span 1.24 m; the camera covers that from about 1.17 m back.
BOOK_SCAN_STANDOFF = 1.5
#: Where to park to reach into the shelf.  The measured reachable envelope is
#: 0.45-0.65 m from the book plane for every row (see docs).
GRASP_STANDOFF = 0.52
#: Where the arm is moved into its grasp posture.  Far enough back that the
#: arm cannot touch the shelf *while swinging into that posture*: the gripper
#: reaches at most ~0.98 m in front of ``base_link``, so at 1.35 m of standoff
#: the furthest it can get is x = 2.53, comfortably short of the 2.755 shelf
#: face.  Getting this wrong is not theoretical - taking the posture at 0.60 m
#: logged 2168 arm_right_5_link/erc_shelf contacts on the way in.
ARM_TRANSIT_STANDOFF = 1.35
PREGRASP_STANDOFF = ARM_TRANSIT_STANDOFF


def world_to_odom(wx, wy, spawn_yaw=SPAWN_YAW):
    """Convert a world-frame XY to the robot's ``odom`` frame."""
    c, s = math.cos(-spawn_yaw), math.sin(-spawn_yaw)
    return (c * wx - s * wy, s * wx + c * wy)


def odom_to_world(ox, oy, spawn_yaw=SPAWN_YAW):
    """Inverse of :func:`world_to_odom`."""
    c, s = math.cos(spawn_yaw), math.sin(spawn_yaw)
    return (c * ox - s * oy, s * ox + c * oy)


def world_yaw_to_odom(wyaw, spawn_yaw=SPAWN_YAW):
    """Convert a world-frame heading to an ``odom``-frame heading."""
    return normalize_angle(wyaw - spawn_yaw)


def odom_yaw_to_world(oyaw, spawn_yaw=SPAWN_YAW):
    """Convert an ``odom``-frame heading to a world-frame heading."""
    return normalize_angle(oyaw + spawn_yaw)


def normalize_angle(a):
    """Wrap an angle into [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def nearest_column_slot(world_y):
    """Snap a measured world ``y`` to the nearest of the five column centres.

    Strictly valid only for *markers*, which sit exactly on the column
    centres.  Books are jittered by up to +/- 0.25 m, which is still inside
    the 0.5 m half-pitch, so this also classifies a book into its column - but
    the jittered value, not the snapped one, is what a grasp must use.
    """
    return min(COLUMN_Y, key=lambda s: abs(COLUMN_Y[s] - world_y))


def nearest_row(world_z):
    """Snap a measured world ``z`` to the nearest of the four book rows."""
    return min(ROW_Z, key=lambda r: abs(ROW_Z[r] - world_z))


#: How far a measured book centre may sit from its nominal row height.
#: Half the row spacing - the old default - is not a test at all: every z
#: between two rows is within half a spacing of one of them.  A measurement
#: 0.163 m adrift passed it, was snapped to a row it was not in, and the robot
#: drove at it.  Real detections do far better than this: across the recorded
#: trials the measured centres missed their nominal rows by 0.003 to 0.019 m,
#: so 0.08 m accepts every genuine book with room to spare while rejecting
#: anything that landed on shelf structure instead.
ROW_HEIGHT_TOLERANCE = 0.08


def row_height_plausible(world_z, tolerance=None):
    """True if ``world_z`` is close enough to a real row to be a book."""
    if tolerance is None:
        tolerance = ROW_HEIGHT_TOLERANCE
    row = nearest_row(world_z)
    return abs(ROW_Z[row] - world_z) <= tolerance


def book_y_plausible(world_y, slot):
    """True if a measured book ``y`` is within the jitter band of ``slot``."""
    if slot not in COLUMN_Y:
        return False
    return abs(world_y - COLUMN_Y[slot]) <= COLUMN_JITTER + 0.05


def point_in_bin(x, y, z, margin=0.0):
    """True if a world point lies inside the bin's bounding box."""
    return (BIN_X[0] - margin <= x <= BIN_X[1] + margin
            and BIN_Y[0] - margin <= y <= BIN_Y[1] + margin
            and BIN_Z[0] - margin <= z <= BIN_Z[1] + margin)
