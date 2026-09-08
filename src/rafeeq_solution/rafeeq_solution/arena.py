"""Arena constants and frame helpers for the ERC 2026 library arena.

Every value here was read out of the competition sources, not guessed:

  * ``src/erc_description/worlds/erc_world.sdf``      — world layout
  * ``src/erc_bringup/launch/simulation.launch.py``   — spawn maths
  * ``src/erc_description/models/*/sdf/*.sdf``        — object geometry

The arena is fully static.  The *only* thing randomised per run is which
number plate sits above which physical column (``random.shuffle`` on
``shelf_number_markers`` in simulation.launch.py).  Column geometry, book
heights, the bin and the start zone never move.  That is what lets us drive
on odometry waypoints instead of standing up a full Nav2 stack.
"""

import math

# ── Shelf ────────────────────────────────────────────────────────────────
SHELF_X = 3.0
SHELF_Y = 0.0
SHELF_Z = 1.1
NUM_COLUMNS = 5
COLUMN_WIDTH = 1.0

#: World ``y`` of each column centre, indexed by column *slot* 1..5.
#: From ``COLUMN_Y_OFFSETS = [((5 - 1) / 2 - col) * 1.0 for col in range(5)]``
#: so slot 1 is at y = +2.0 and slot 5 at y = -2.0.
COLUMN_Y = {1: 2.0, 2: 1.0, 3: 0.0, 4: -1.0, 5: -2.0}

# ── Number marker plates ─────────────────────────────────────────────────
#: Plates hang above the shelf, one per column, flush with the shelf face.
MARKER_X = SHELF_X - 0.245          # 2.755
MARKER_Z = 2.26
MARKER_SIZE = 0.30                  # square plate, metres
MARKER_TEXTURE_REL = 'models/number_marker/textures/{n}.png'

# ── Books ────────────────────────────────────────────────────────────────
#: Book box is 0.25 x 0.03 x 0.16 spawned with pitch = 90 deg, so the face
#: pointing at the robot is the *spine*: 0.03 m wide by 0.25 m tall.
#: That 1:8 aspect ratio is the single most distinctive cue in the image.
BOOK_SPINE_W = 0.03
BOOK_SPINE_H = 0.25
BOOK_DEPTH = 0.16
BOOK_X = SHELF_X - 0.1              # 2.9, front face of the book

#: Book centre height for competition rows 1..4, top row first.
#: ``SHELF_Z + (TOP_BOOK_Z - i * ROW_SPACING)`` for i in ACTIVE_ROWS = 1..4
#: with TOP_BOOK_Z = 0.825 and ROW_SPACING = 0.33.
ROW_Z = {1: 1.595, 2: 1.265, 3: 0.935, 4: 0.605}

BOOK_COLOURS = ('red', 'green', 'blue', 'yellow')

# ── Collection bin and start zone ────────────────────────────────────────
#: The bin sits *on a table*, so it is 1.3 m off the floor — this is a
#: place-at-height task, not a drop-on-the-floor task.
BIN_XYZ = (-1.0, 0.0, 1.3)
TABLE_XYZ = (-1.0, 0.0, 0.7)
START_XY = (0.0, 0.0)

#: Robot spawn yaw in the world frame (simulation.launch.py spawns the robot
#: with ``-Y 1.5708``).  Odometry is published relative to the spawn pose, so
#: this is the rotation between the ``odom`` frame and the world frame.
SPAWN_YAW = 1.5708

# ── Arena bounds (walls), used for waypoint sanity checks ────────────────
ARENA_X = (-4.0, 6.0)
ARENA_Y = (-5.0, 5.0)

# ── Standoff distances ───────────────────────────────────────────────────
#: Where to park to read the number plates: far enough back that all five
#: plates fit in the 87 deg horizontal FOV (5 columns = 5 m of shelf; at
#: 3.0 m the visible width is 2 * 3 * tan(43.5 deg) = 5.7 m).
SCAN_STANDOFF_X = 0.0
#: Where to park to see all four book rows of one column at once.
#: The four rows span 1.24 m vertically (row 1 centre 1.595 m down to row 4
#: centre 0.605 m, plus one 0.25 m book height).  With fy = 339 px and a
#: 360 px tall image the camera covers 1.06 m at 1.0 m range and 1.59 m at
#: 1.5 m, so anything closer than ~1.17 m cannot see all four rows at once
#: and the row-by-vertical-order logic would silently mis-rank.
BOOK_SCAN_STANDOFF = 1.5

#: Where to park to reach into the shelf.
PREGRASP_STANDOFF = 0.75            # metres in front of the shelf face
GRASP_STANDOFF = 0.48


def world_to_odom(wx, wy, spawn_yaw=SPAWN_YAW):
    """Convert a world-frame XY to the robot's ``odom`` frame.

    ``odom`` starts at identity at the spawn pose, so a world point is the
    spawn rotation applied to the odom point.  Inverting that rotation gives
    us the odom coordinates of a known world landmark.

    If the robot ever spawns somewhere else, ``spawn_yaw`` is the single
    parameter to change — every waypoint in the mission flows through here.
    """
    dx = wx - 0.0
    dy = wy - 0.0
    c, s = math.cos(-spawn_yaw), math.sin(-spawn_yaw)
    return (c * dx - s * dy, s * dx + c * dy)


def odom_to_world(ox, oy, spawn_yaw=SPAWN_YAW):
    """Inverse of :func:`world_to_odom`."""
    c, s = math.cos(spawn_yaw), math.sin(spawn_yaw)
    return (c * ox - s * oy, s * ox + c * oy)


def world_yaw_to_odom(wyaw, spawn_yaw=SPAWN_YAW):
    """Convert a world-frame heading to an ``odom``-frame heading."""
    return normalize_angle(wyaw - spawn_yaw)


def normalize_angle(a):
    """Wrap an angle into [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def column_world_pose(slot, standoff=PREGRASP_STANDOFF):
    """World-frame ``(x, y, yaw)`` for a robot parked facing column ``slot``."""
    return (SHELF_X - standoff, COLUMN_Y[slot], 0.0)


def nearest_column_slot(world_y):
    """Snap a measured world ``y`` to the nearest of the five fixed columns.

    Vision only has to be accurate to +/- 0.5 m for this to land on the right
    column, which is why we snap instead of trusting the raw deprojection.
    """
    return min(COLUMN_Y, key=lambda s: abs(COLUMN_Y[s] - world_y))


def nearest_row(world_z):
    """Snap a measured world ``z`` to the nearest of the four book rows."""
    return min(ROW_Z, key=lambda r: abs(ROW_Z[r] - world_z))
