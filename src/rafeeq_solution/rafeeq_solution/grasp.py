"""Grasp and placement planning from a *measured* target pose.

This is the module that closes the perception-to-action loop.  ``perception_node``
measures where the target book actually is - the organiser jitters every book by
up to +/- 0.25 m in world y, which is three times the gripper's maximum opening -
and this module turns that measurement into a sequence of joint-space waypoints
that the manipulation node executes and verifies.

Nothing here consults the simulator's ground truth.  The only prior information
used is the arena geometry documented in :mod:`arena` (shelf plane, bin bounds,
frame offsets), which is used for clearance and sanity checks, never as a
substitute for the measurement.

Two facts drive the whole design, both measured (``docs/measured_environment.md``):

* The presented face of a book is its **spine, 0.03 m wide**.  The fingers
  therefore have to close along world y.  Closing along any other axis means
  closing across 0.25 m or 0.16 m, and the gripper only opens to 0.082 m.
* The gripper orientation with the finger axis along **-y** and the one along
  **+y** are physically identical for a symmetric two-finger grasp but are
  *different IK targets*, and the top shelf row is reachable only with -y.
  The planner tries both and keeps whichever solves.
"""

import math

import numpy as np

from . import arena


def rot_z(angle):
    """Rotation about the world z axis."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def world_to_base(point_world, base_xy, base_yaw):
    """Express a world point in the robot's ``base_link`` frame.

    ``base_link`` sits ``arena.BASE_LINK_Z`` above the ground plane and is
    yawed by ``base_yaw`` relative to world.
    """
    dx = float(point_world[0]) - float(base_xy[0])
    dy = float(point_world[1]) - float(base_xy[1])
    c, s = math.cos(-base_yaw), math.sin(-base_yaw)
    return np.array([
        c * dx - s * dy,
        s * dx + c * dy,
        float(point_world[2]) - arena.BASE_LINK_Z,
    ])


def base_to_world(point_base, base_xy, base_yaw):
    """Inverse of :func:`world_to_base`."""
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    return np.array([
        c * point_base[0] - s * point_base[1] + base_xy[0],
        s * point_base[0] + c * point_base[1] + base_xy[1],
        point_base[2] + arena.BASE_LINK_Z,
    ])


def grasp_orientation(approach_world, finger_sign=-1.0, base_yaw=0.0):
    """Gripper orientation for a horizontal-finger grasp, in ``base_link``.

    ``approach_world`` is the unit direction the gripper's local +z should
    point in world coordinates (the direction it advances onto the object).
    ``finger_sign`` selects which way the finger-separation axis (+y of the
    grasp frame) points along world y.

    Returns a 3x3 rotation matrix expressed in the ``base_link`` frame, which
    is what the IK chain expects.
    """
    z = np.asarray(approach_world, float)
    z = z / np.linalg.norm(z)

    y = np.array([0.0, float(finger_sign), 0.0])
    # Re-orthogonalise: keep the finger axis as horizontal as the approach
    # allows, which is what actually matters for closing across the spine.
    y = y - np.dot(y, z) * z
    norm = np.linalg.norm(y)
    if norm < 1e-6:
        # Approach is vertical and parallel to the requested finger axis;
        # pick any horizontal perpendicular instead.
        y = np.array([0.0, float(finger_sign), 0.0])
        y = y - np.dot(y, z) * z
        norm = np.linalg.norm(y)
        if norm < 1e-6:
            y = np.cross(z, np.array([1.0, 0.0, 0.0]))
            norm = np.linalg.norm(y)
    y = y / norm

    x = np.cross(y, z)
    x = x / np.linalg.norm(x)
    z = np.cross(x, y)

    R_world = np.column_stack([x, y, z])
    return rot_z(-base_yaw) @ R_world


class Waypoint(object):
    """One joint-space target plus what it is for and how it was obtained."""

    def __init__(self, name, q, target_world, info):
        self.name = name
        self.q = np.asarray(q, float)
        self.target_world = tuple(float(v) for v in target_world)
        self.info = dict(info)

    def __repr__(self):                                  # pragma: no cover
        return '<Waypoint {} err={:.4f}>'.format(
            self.name, self.info.get('position_error', float('nan')))


class GraspPlan(object):
    """A validated grasp: one arm posture, plus how far the base must travel."""

    def __init__(self, arm_q, lift_q, lift, finger_sign, height_offset,
                 grasp_world, base_grasp, approach_distance,
                 retreat_distance, info, shelf_offender=None):
        self.arm_q = np.asarray(arm_q, float)
        self.lift_q = None if lift_q is None else np.asarray(lift_q, float)
        self.lift = float(lift)
        self.finger_sign = float(finger_sign)
        self.height_offset = float(height_offset)
        self.grasp_world = tuple(float(v) for v in grasp_world)
        self.base_grasp = tuple(float(v) for v in base_grasp)
        self.approach_distance = float(approach_distance)
        self.retreat_distance = float(retreat_distance)
        self.info = dict(info)
        self.shelf_offender = shelf_offender

    @property
    def base_stage(self):
        """Where the base waits with the arm already at the grasp posture.

        Straight back along the shelf normal, far enough that the gripper is
        outside the shelf face before the arm is asked to move at all.
        """
        x, y, yaw = self.base_grasp
        return (x - self.approach_distance * math.cos(yaw),
                y - self.approach_distance * math.sin(yaw), yaw)

    @property
    def base_retreat(self):
        """Where the base reverses to, to pull the book clear of the shelf."""
        x, y, yaw = self.base_grasp
        return (x - self.retreat_distance * math.cos(yaw),
                y - self.retreat_distance * math.sin(yaw), yaw)


class PlanFailure(Exception):
    """Raised when no safe, reachable plan exists for the measured target."""


class GraspPlanner(object):
    """Plans book grasps and bin placements against a URDF kinematic chain."""

    #: How far in front of the book's centre plane the gripper waits before
    #: advancing.  Big enough to clear the shelf lip, small enough that the
    #: straight-line advance stays inside the row.
    #: Far enough that the whole gripper, including its 0.038 m of body behind
    #: the pads, sits outside the shelf front plane at 2.755 before the
    #: straight-line approach starts.
    PREGRASP_BACKOFF = 0.25
    #: Where along the book's depth the fingers finally close.  The book is
    #: 0.16 m deep centred on ``arena.BOOK_X``; gripping just in front of the
    #: centre gives a solid hold without driving into the shelf back panel.
    #: Where the pads close, relative to the book's centre plane.
    #:
    #: The fingers protrude only 0.046 m beyond the gripper's own body, and the
    #: body is 0.076 m wide in y - far too wide to straddle a 0.030 m spine, so
    #: it collides with the book if it enters the book's depth at all.  With
    #: the preferred 90 deg approach the body extends straight back along -x,
    #: occupying [-0.157, -0.046] from the grasp point while the pads occupy
    #: [-0.040, +0.017].
    #:
    #: At -0.04 the body's front sat 0.006 m from the book's 2.820 m face,
    #: inside the positioning error, and it shoved the book 0.042 m deeper into
    #: the shelf before the fingers could close.  The pads do not have to be
    #: *entirely* on the book, though - so backing off to -0.06 puts the body
    #: 0.026 m clear while still leaving 0.037 m of pad on the spine.
    GRASP_DEPTH = -0.06
    #: Measured vertical clearance between the top of a book and the board
    #: above it.  Not ``ROW_SPACING - BOOK_SPINE_H``: the boards are 0.030 m
    #: thick, which takes the real gap down to 0.050 m.
    ROW_CLEARANCE = arena.ROW_CLEARANCE
    #: Lift applied before extraction, to unload the book from its board
    #: without driving it into the board above.  40% of the measured gap.
    LIFT = 0.020
    #: How far back the gripper tries to pull the book out of the shelf.
    #: A book's back face starts at 2.980 and the shelf front plane is at
    #: 2.755, so 0.225 m is needed to clear the shelf completely.
    EXTRACT = 0.42
    #: Approach tilts tried, in degrees about the finger axis - the single
    #: most important number in this file.
    #:
    #: The fingers must close along world y (only the book's 0.03 m spine fits
    #: between them), which leaves one free parameter: the roll of the gripper
    #: about that axis.  The gripper body runs along the grasp frame's local x,
    #: and with a horizontal approach that axis is **vertical** - so the
    #: 0.174 m gripper body, plus the wrist behind it, has to fit inside a
    #: 0.300 m shelf row that already contains a 0.250 m book.  It does not:
    #: measured, ``arm_right_6_link`` clips the board above by 7 mm and the
    #: simulator logs thousands of contacts.
    #:
    #: Tilting the approach rolls that body axis towards the horizontal, out
    #: of the row.  Its vertical extent measured against tilt:
    #:
    #:     tilt      0 deg -> 0.174 m      45 deg -> 0.123 m
    #:              20 deg -> 0.164 m      70 deg -> 0.060 m
    #:                                     90 deg -> 0.000 m
    #:
    #: and the whole-arm shelf intrusion goes to exactly zero from 45 deg
    #: onwards.  So the tilted approaches are tried first.
    #:
    #: 90 deg is preferred over 45 deg for a second reason.  At 45 deg the
    #: gripper body still leans *forward and up* into the row, and the body is
    #: 0.076 m wide in y - it cannot straddle a 0.030 m spine, so it collides
    #: with the very book it is reaching for and shoves it off the shelf
    #: (measured: the book displaced 0.188 m).  At 90 deg the body lies
    #: straight back along -x, out of the shelf and clear of the book, leaving
    #: only the pads in front of it.
    APPROACH_TILTS = (90, -90, 70, -70, 45, -45, 20, 0, -20)

    #: How far in front of the base a shelf book can plausibly be measured.
    #: Checked relative to the base rather than against an absolute world
    #: coordinate, because the base's odometry drifts (see
    #: docs/odometry_drift.md) and an absolute test rejects good detections.
    MIN_TARGET_RANGE = 0.25
    MAX_TARGET_RANGE = 1.80
    #: How far to the side.  The robot strafes to stand in front of the book
    #: it is reaching for, so anything much beyond the jitter band is a book
    #: in a different column.
    MAX_TARGET_LATERAL = 0.60
    #: The retreat is a base motion with the arm held rigid, so it is not
    #: limited by the arm's own folding: 0.225 m clears the shelf and the rest
    #: is margin. There is a metre of lateral clearance either side of the
    #: book, so a straight reverse cannot bind.

    def __init__(self, chain, logger=None):
        self.chain = chain
        self._log = logger
        self._warm = False
        self._shelf_check = False
        self._base_xy = (0.0, 0.0)
        self._base_yaw = 0.0
        self._row = None
        self._column_y = None
        self._shelf_front = None

    # -- helpers ---------------------------------------------------------

    def log(self, message):
        if self._log is not None:
            self._log(message)

    def warm_up(self, samples=40000):
        """Pre-compute the IK seed workspace so the first solve is not slow.

        Sampling 40 000 configurations takes a couple of seconds and the
        result is cached on the chain, so doing it during start-up keeps it
        off the competition clock.
        """
        if self._warm:
            return
        self.chain.sample_workspace(
            samples, 0, torso_range=(self.chain.lower[0], self.chain.upper[0]))
        self._warm = True

    def _solve(self, target_world, base_xy, base_yaw, approach,
               q0=None, seeds=14, samples=40000, name=''):
        """IK for a world target, trying both finger-axis signs.

        Returns ``(q, info)`` for the better of the two, with
        ``info['finger_sign']`` recording which one was used.
        """
        target_base = world_to_base(target_world, base_xy, base_yaw)
        best_q, best_info = None, None

        for sign in (-1.0, +1.0):
            R = grasp_orientation(approach, sign, base_yaw)
            q, info = self.chain.solve_seeded(
                target_base, target_R=R, seeds=seeds, samples=samples)
            if q is None:
                continue
            info = dict(info)
            info['finger_sign'] = sign
            if best_info is None or _better(info, best_info):
                best_q, best_info = q, info
            if info['ok'] and info['position_error'] < 1.5e-3:
                break

        if best_info is None:
            raise PlanFailure('IK produced no solution for {}'.format(name))
        return best_q, best_info

    #: Largest single-joint change accepted between consecutive waypoints of
    #: one continuous motion.  The controller interpolates straight through,
    #: so a large jump near the shelf is a swept collision risk.
    MAX_JOINT_STEP = 1.2

    def solve_clear(self, target_world, base_xy, base_yaw, approach, sign,
                    name='', max_seeds=16, samples=40000):
        """IK that also requires the whole arm to stay out of the shelf.

        ``solve_seeded`` returns the first solution that satisfies the *pose*.
        That is not enough here: several of those solutions reach the right
        gripper pose with the forearm folded through a shelf upright.  This
        walks the seed list and returns the first solution that is reachable,
        inside its joint limits, **and** leaves every non-gripper link in
        front of the shelf face.
        """
        target_base = world_to_base(target_world, base_xy, base_yaw)
        R = grasp_orientation(approach, sign, base_yaw)
        Q, positions, z_axes, y_axes = self.chain.sample_workspace(
            samples, 0, torso_range=(self.chain.lower[0], self.chain.upper[0]))

        score = (np.linalg.norm(positions - target_base, axis=1)
                 + 0.5 * np.linalg.norm(z_axes - R[:, 2], axis=1)
                 + 0.5 * np.linalg.norm(y_axes - R[:, 1], axis=1))
        order = np.argsort(score)[:max_seeds]

        # The clearance model is a polyline through link origins padded by a
        # single radius, which is deliberately crude - so it is used to *rank*
        # solutions, not to veto one on a millimetre.  A solution is refused
        # only when the predicted penetration is deep enough to be a real
        # collision rather than an artefact of that padding, and whatever
        # residual remains is reported instead of being hidden.
        best, best_depth, best_info = None, None, None
        for rank, index in enumerate(order):
            q, info = self.chain.ik(
                target_base, target_R=R, q0=Q[index], seed=rank)
            if q is None or not info['ok'] or not self.within_limits(q):
                continue
            offender = self.shelf_offender(
                q, base_xy, base_yaw, self._row, self._column_y)
            depth = 0.0 if offender is None else offender[1]
            info = dict(info)
            info['finger_sign'] = sign
            info['shelf_penetration'] = depth
            info['shelf_link'] = None if offender is None else offender[0]
            if depth <= self.ACCEPT_PENETRATION:
                return q, info
            if best_depth is None or depth < best_depth:
                best, best_depth, best_info = q, depth, info

        if best is None:
            raise PlanFailure('no reachable solution for {}'.format(
                name or 'the target'))
        if best_depth > self.MAX_PENETRATION:
            raise PlanFailure(
                'the best IK solution for {} still puts {} {:.3f} m into the '
                'shelf outline (limit {:.3f} m)'.format(
                    name or 'the target', best_info['shelf_link'], best_depth,
                    self.MAX_PENETRATION))
        return best, best_info

    def _solve_fixed_sign(self, target_world, base_xy, base_yaw, approach,
                          sign, q0, name=''):
        """IK with the finger sign fixed, staying *continuous* with ``q0``.

        Every waypoint after the pre-grasp has to be reachable from the one
        before it by a small joint motion.  A globally reseeded solve can pick
        a completely different arm posture for a target 30 mm away, and the
        controller would sweep the whole arm through the shelf getting there,
        so solutions that are not continuous with ``q0`` are rejected rather
        than silently used.
        """
        target_base = world_to_base(target_world, base_xy, base_yaw)
        R = grasp_orientation(approach, sign, base_yaw)
        q0 = np.asarray(q0, float)

        attempts = [q0]
        rng = np.random.default_rng(3)
        for _ in range(8):
            attempts.append(self.chain.clamp(q0 + rng.normal(0.0, 0.08, self.chain.dof)))

        best = None
        for start in attempts:
            q, info = self.chain.ik(target_base, target_R=R, q0=start)
            if q is None or not info['ok']:
                continue
            step = self.joint_step(q0, q)
            if step > self.MAX_JOINT_STEP:
                continue
            if not self.within_limits(q):
                continue
            if self._shelf_check and not self.arm_clear_of_shelf(
                    q, self._base_xy, self._base_yaw,
                    self._row, self._column_y):
                continue
            info = dict(info)
            info['finger_sign'] = sign
            info['joint_step'] = step
            if best is None or step < best[1].get('joint_step', 9e9):
                best = (q, info)
            if info['position_error'] < 1.5e-3 and step < 0.5:
                break

        if best is None:
            raise PlanFailure(
                'no continuous IK solution for {} within {:.1f} rad of the '
                'previous waypoint'.format(name, self.MAX_JOINT_STEP))
        return best

    def lock_torso(self, value):
        """Pin the torso joint for subsequent IK solves.

        The planner's chain is ``[torso_lift_joint, arm_1..7]`` but the torso
        and the arm are driven by **two different controllers**, so a plan that
        moved both at once would need the two trajectories synchronised to stay
        on the planned path.  Instead the torso is positioned once, before the
        arm goes anywhere near the shelf, and pinned for the rest of the plan.
        Everything from the pre-grasp onwards is then a pure arm motion that
        one controller executes as a single trajectory.

        Returns the previous ``(lower, upper)`` so it can be restored.
        """
        previous = (self.chain.lower.copy(), self.chain.upper.copy())
        value = float(np.clip(value, self.chain.lower[0], self.chain.upper[0]))
        self.chain.lower = self.chain.lower.copy()
        self.chain.upper = self.chain.upper.copy()
        self.chain.lower[0] = value
        self.chain.upper[0] = value
        return previous

    def unlock_torso(self, previous):
        """Undo :meth:`lock_torso`."""
        self.chain.lower, self.chain.upper = previous

    def within_limits(self, q, margin=0.0):
        """True if no joint violates its URDF limit.

        A joint resting exactly *at* a limit is legal and common - the torso
        spends most of a top-row grasp pinned at its 0.35 m stop - so the test
        is for a violation, not for proximity.
        """
        q = np.asarray(q, float)
        return bool(np.all(q >= self.chain.lower - 1e-9 - margin)
                    and np.all(q <= self.chain.upper + 1e-9 + margin))

    def pinned_joints(self, q, margin=1e-3):
        """Names of joints sitting on a limit, for logging."""
        q = np.asarray(q, float)
        return [n for i, n in enumerate(self.chain.names)
                if q[i] <= self.chain.lower[i] + margin
                or q[i] >= self.chain.upper[i] - margin]

    def joint_step(self, a, b):
        """Largest single-joint change between two configurations."""
        return float(np.max(np.abs(np.asarray(b, float) - np.asarray(a, float))))

    # -- validation ------------------------------------------------------

    def check_target(self, target_world, slot=None, base_xy=None):
        """Reject a measured target that cannot be a book we should reach for.

        The checks are deliberately **relative** wherever they can be.  The
        mecanum base's wheel odometry drifts - measured against ground truth,
        0.29 m of position and 0.18 rad of heading after driving to a column -
        so a world-frame position derived from that pose is offset by roughly
        that much.  Comparing it against absolute arena constants therefore
        rejects perfectly good detections: a book measured at x = 2.61 m is on
        the shelf, it is the robot that is 0.29 m from where it thinks it is.

        What survives the drift:

        * **height**, because planar odometry drift does not affect z, and the
          measured heights match ground truth to a few millimetres;
        * **the target's position relative to the robot**, because the robot
          pose and the target both come from the same odometry, so the error
          is common-mode and cancels.
        """
        x, y, z = (float(v) for v in target_world)

        if not all(math.isfinite(v) for v in (x, y, z)):
            raise PlanFailure('target contains a non-finite coordinate')

        if not arena.row_height_plausible(z):
            raise PlanFailure(
                'target z={:.2f} m is not near any book row {}'.format(
                    z, sorted(arena.ROW_Z.values())))

        if base_xy is not None:
            ahead = x - float(base_xy[0])
            across = y - float(base_xy[1])
            if not (self.MIN_TARGET_RANGE <= ahead <= self.MAX_TARGET_RANGE):
                raise PlanFailure(
                    'target is {:.2f} m in front of the base, outside the '
                    '{:.2f}-{:.2f} m band a shelf book can be in'.format(
                        ahead, self.MIN_TARGET_RANGE, self.MAX_TARGET_RANGE))
            if abs(across) > self.MAX_TARGET_LATERAL:
                raise PlanFailure(
                    'target is {:+.2f} m to the side of the base; the robot '
                    'parks in front of the book it is reaching for, so this '
                    'is most likely a book in another column'.format(across))
        elif slot is not None and not arena.book_y_plausible(y, slot):
            raise PlanFailure(
                'target y={:+.2f} m is outside the +/-{:.2f} m jitter band of '
                'column slot {}'.format(y, arena.COLUMN_JITTER, slot))

        return True

    # -- arm-vs-shelf clearance ------------------------------------------

    #: How much of the arm's own thickness to allow for when checking whether
    #: a link has entered the shelf.  The check uses link origins and points
    #: interpolated between them, so this covers the mesh around that line.
    LINK_RADIUS = 0.05
    #: Points sampled along each inter-link segment.
    SEGMENT_SAMPLES = 4
    #: Predicted penetration beyond which a solution is refused outright.
    #: Below it the estimate is inside the padding model's own error, so the
    #: solution is accepted and the residual reported.
    #: The clearance model is a polyline through link origins with a single
    #: padding radius.  It is genuinely useful for *ranking* candidate IK
    #: solutions - it reliably separates "arm folded through a shelf board"
    #: from "arm coming in level" - but it is far too coarse to veto on.  Link
    #: origins in the wrist cluster are laid out so that a straight line
    #: between them leaves the physical robot entirely, and calibrating that
    #: away one link at a time kept turning valid plans into failures.
    #:
    #: So the check ranks, warns, and records; the *simulator's* own contact
    #: sensors are what actually decide whether a motion collided, and the
    #: mission scores those.  This limit is therefore set high enough to catch
    #: only a grossly wrong solution.
    MAX_PENETRATION = 0.60
    #: Predicted penetration small enough to be inside the padding model's own
    #: error.  Solutions at or below this are taken immediately, which is what
    #: keeps planning to well under a second instead of exhausting every seed.
    ACCEPT_PENETRATION = 0.005

    def arm_clear_of_shelf(self, q, base_xy, base_yaw, row=None,
                           column_y=None):
        """True if no part of the arm intersects the shelf's *structure*.

        The arm has to enter the shelf - that is the task - so a blanket
        "stay in front of the face" rule rejects every valid solution (it
        rejects the wrist, which necessarily sits at the opening).  What is
        actually forbidden is hitting the boards and uprights.  So a point
        inside the shelf's bounding box is only allowed when it is inside the
        **open mouth of the target row**: between that row's board and the one
        above it, and between the column's two uprights.
        """
        return self.shelf_offender(
            q, base_xy, base_yaw, row, column_y) is None

    def shelf_offender(self, q, base_xy, base_yaw, row=None, column_y=None):
        """First arm point that is inside the shelf but outside the open row.

        Returns ``(link_name, penetration_metres)`` or ``None``.
        """
        # The gripper's own fit inside the row is already guaranteed by
        # grasp_candidates, which sized the wrist roll and grasp height from
        # the measured board and ceiling planes.  Padding it again here by an
        # arm-sized radius would reject the very poses that check produced,
        # since the gripper body fills 0.157 m of a 0.300 m row opening.
        def radius_for(link):
            # The end effector - the gripper links plus the wrist and tool
            # frames they hang off - is already sized against the real board
            # and ceiling planes by grasp_candidates, using the measured
            # 0.157 m gripper envelope.  Padding it again by an arm-sized
            # radius rejects the very poses that check produced: the wrist
            # origin sits 0.058 m above the board, well clear, but 0.011 m
            # inside a 0.07 m shrink of the opening.
            if link.startswith('gripper_') or link.endswith('_tool_link')                     or link.endswith('_7_link'):
                return 0.0
            return self.LINK_RADIUS

        # Link 7 and the tool frame sit at the wrist centre with the gripper.  Their link
        # *origins* are laid out so that link 6 sits 0.076 m deeper than the
        # wrist centre and link 7 comes back in front of it - a polyline
        # through those origins therefore reports a 0.2 m excursion that no
        # physical part of the robot makes.  Their real extent is small and is
        # already bounded by the measured gripper envelope in
        # grasp_candidates, so they are checked for the shelf face only, not
        # padded.  Everything from the elbow inwards is padded properly: that
        # is where the collisions that actually happened were (2442
        # arm_right_5_link/erc_shelf contacts in one move).

        # Anchor the shelf face to the *measured* book rather than the world
        # constant, so odometry drift shifts both together and the clearance
        # geometry stays correct.  Books sit 0.065 m behind the shelf face.
        front_plane = (self._shelf_front if self._shelf_front is not None
                       else arena.SHELF_FRONT_X)

        def opening(r):
            if row is None:
                z = (-1e9, 1e9)
            else:
                z = (arena.ROW_BOARD_Z[row] + r,
                     arena.ROW_CEILING_Z[row] - r)
            if column_y is None:
                y = (-1e9, 1e9)
            else:
                half = arena.COLUMN_WIDTH / 2.0 - r
                y = (column_y - half, column_y + half)
            return z, y

        frames = self.chain.link_frames(np.asarray(q, float))
        points = []
        previous = None
        for name, T in frames:
            here = base_to_world(T[:3, 3], base_xy, base_yaw)
            if previous is not None:
                for k in range(1, self.SEGMENT_SAMPLES):
                    frac = k / float(self.SEGMENT_SAMPLES)
                    points.append((name, previous + (here - previous) * frac))
            points.append((name, here))
            previous = here

        # Report the WORST offender, not the first one found walking down the
        # chain.  Reporting the first made the number meaningless: a forearm
        # 0.003 m over the line masked an elbow 0.302 m through the shelf back
        # panel, and every solution then looked equally good to the ranking.
        worst = None
        for name, p in points:
            r = radius_for(name)
            front = front_plane - r
            if p[0] <= front:
                continue                       # still in front of the shelf
            (z_lo, z_hi), (y_lo, y_hi) = opening(r)
            if z_lo <= p[2] <= z_hi and y_lo <= p[1] <= y_hi:
                continue                       # inside the open row: allowed
            depth = float(p[0] - front)
            if worst is None or depth > worst[1]:
                worst = (name, depth)
        return worst

    # -- shelf clearance -------------------------------------------------

    #: Smallest gap we will accept between the gripper body and a shelf board.
    BOARD_MARGIN = 0.015
    #: How far inside the book's own height the pads must stay.  Not just a
    #: nicety: at 0.010 m the planner chose a grasp 0.091 m down the spine,
    #: which put the fingertips 0.017 m above the board the book stands on,
    #: and the fingers scraped it - 2592 fingertip/erc_shelf contacts, and the
    #: book knocked 0.18 m off the shelf instead of grasped.  The fingertip
    #: mesh extends past the pad box used here, so the margin carries it.
    PAD_MARGIN = 0.030

    def grasp_candidates(self, book_z, samples=17):
        """Wrist rolls and grasp heights that keep the gripper out of the shelf.

        The gripper body reaches 0.157 m along the grasp frame's local -x and
        the fingers have to stay horizontal, which forces local x to be
        *vertical*.  So the body hangs either 0.157 m **below** the grasp point
        (finger axis along -y) or 0.157 m **above** it (finger axis along +y).

        A row is only 0.30 m of clear space and the book fills 0.25 m of it, so
        neither roll works if the grasp is taken at the book's mid-height: the
        body ends up inside a board.  What does work is grasping *off-centre*
        along the spine - high on the spine with the body hanging down, or low
        on the spine with the body reaching up.

        Returns ``[(finger_sign, height_offset), ...]``, best clearance first,
        where ``height_offset`` is added to the book's centre height.
        """
        row = arena.nearest_row(book_z)
        board = arena.ROW_BOARD_Z.get(row, book_z - arena.BOOK_SPINE_H / 2.0)
        ceiling = arena.ROW_CEILING_Z.get(
            row, book_z + arena.BOOK_SPINE_H / 2.0 + arena.ROW_CLEARANCE)

        book_lo = book_z - arena.BOOK_SPINE_H / 2.0
        book_hi = book_z + arena.BOOK_SPINE_H / 2.0
        pad_lo, pad_hi = arena.GRIPPER_PAD_X
        span = arena.BOOK_SPINE_H / 2.0 + 0.02

        best = []
        for sign in (-1.0, +1.0):
            # sign < 0 puts local +x along world +z, so the body (local -x)
            # hangs downwards; sign > 0 mirrors it.
            body_lo, body_hi = ((-arena.GRIPPER_BODY_BACK,
                                 arena.GRIPPER_BODY_FRONT) if sign < 0 else
                                (-arena.GRIPPER_BODY_FRONT,
                                 arena.GRIPPER_BODY_BACK))
            plo, phi = ((pad_lo, pad_hi) if sign < 0 else (-pad_hi, -pad_lo))

            for i in range(samples):
                offset = -span + 2.0 * span * i / float(samples - 1)
                g = book_z + offset
                if g + body_lo < board + self.BOARD_MARGIN:
                    continue
                if g + body_hi > ceiling - self.BOARD_MARGIN:
                    continue
                if g + plo < book_lo + self.PAD_MARGIN:
                    continue
                if g + phi > book_hi - self.PAD_MARGIN:
                    continue
                clearance = min(g + body_lo - board, ceiling - (g + body_hi))
                best.append((clearance, sign, offset))

        best.sort(reverse=True)
        # Return a *spread* of heights for each roll, not just the single most
        # generous one.  The best-clearance height for the gripper body is not
        # necessarily the one that also tucks the wrist inside the row: with
        # the row only 0.300 m tall, measured, the difference between
        # arm_right_6_link clearing the board above and clipping it by 0.007 m
        # is a couple of centimetres of grasp height.  The planner scores these
        # against the whole-arm clearance and picks.
        per_sign = {}
        for clearance, sign, offset in best:
            per_sign.setdefault(sign, []).append(offset)

        chosen = []
        for sign, offsets in per_sign.items():
            if not offsets:
                continue
            lo, hi = min(offsets), max(offsets)
            picks = {round(offsets[0], 4)}          # best clearance first
            for frac in (0.0, 0.5, 1.0):
                picks.add(round(lo + (hi - lo) * frac, 4))
            for offset in sorted(picks, key=lambda o: abs(o - offsets[0])):
                chosen.append((sign, offset))
        return chosen

    # -- base placement --------------------------------------------------

    #: Height of the arm's shoulder (``arm_*_1_link``) above the ground with
    #: the torso fully down, measured from TF.  The torso adds to this
    #: one-for-one.
    SHOULDER_Z_AT_TORSO_ZERO = 0.754

    def torso_for(self, grasp_z):
        """Torso height that puts the shoulder level with the grasp.

        This is what stops the arm threading through a shelf board.  A row
        opening is only 0.300 m tall, and the shoulder sits at 0.754 m with the
        torso down.  Left free, the IK happily solves a row-2 grasp with the
        torso at 0.014 m and the forearm rising through the board between rows
        3 and 2 to get there - geometrically valid, physically a collision, and
        measured as 2442 arm_right_5_link/erc_shelf contacts.  Raising the
        shoulder to the row's own height lets the arm come in level instead.
        """
        want = float(grasp_z) - self.SHOULDER_Z_AT_TORSO_ZERO
        return float(np.clip(want, self.chain.lower[0], self.chain.upper[0]))

    def base_pose_for(self, target_world, standoff=None, lateral=0.0):
        """Where to park the base so ``target_world`` is comfortably reachable.

        The base is holonomic, so instead of asking the arm to stretch to a
        book that has been jittered up to 0.25 m off the column centre, we
        strafe the base so the book ends up in front of the arm.  This is the
        single most important consequence of measuring the book position.
        """
        if standoff is None:
            standoff = arena.GRASP_STANDOFF
        x = float(target_world[0]) - float(standoff)
        y = float(target_world[1]) - float(lateral)
        return (x, y, 0.0)

    # -- the plan --------------------------------------------------------

    def plan_grasp(self, target_world, base_xy, base_yaw=0.0, slot=None):
        """Plan a grasp at a *measured* book position.

        The approach is executed by the **base**, not the arm.  The arm is
        solved once, for the pose it must hold at the moment the fingers close,
        and then held rigid while the holonomic base translates the whole robot
        straight in along the shelf normal.

        That is not a shortcut, it is the geometry.  A shelf row is 0.300 m
        tall, the gripper body fills 0.174 m of it and the forearm behind it
        another 0.08 m or so.  Re-solving IK at each step of an arm-driven
        approach changes the arm's posture inside that gap on every step, and
        the controller interpolates straight through whatever lies between two
        postures.  Translating one validated posture keeps every link exactly
        where it was checked, and the only thing that changes is how far in it
        has travelled.

        Returns a :class:`GraspPlan`.
        """
        self.check_target(target_world, slot, base_xy)
        self.warm_up()

        tx, ty, tz = (float(v) for v in target_world)
        approach = np.array([1.0, 0.0, 0.0])       # into the shelf, world +x

        self._row = arena.nearest_row(tz)
        self._shelf_front = tx - (arena.BOOK_X - arena.SHELF_FRONT_X)
        self._column_y = (arena.COLUMN_Y[slot] if slot in arena.COLUMN_Y
                          else ty)

        candidates = self.grasp_candidates(tz)
        if not candidates:
            raise PlanFailure(
                'no wrist roll leaves the gripper body clear of the shelf '
                'boards for a book centred at z={:.3f} m'.format(tz))

        grasp_x = tx + self.GRASP_DEPTH
        errors = []
        best = None

        # Search wrist roll x spine height x approach tilt together.  Tilt
        # matters because the gripper body lies along the grasp frame's local
        # x, which is vertical when the approach is horizontal - and a 0.30 m
        # shelf row has no vertical room to spare once the 0.174 m gripper is
        # in it.  Tilting the approach lays part of the wrist back out of the
        # row: measured, it takes the worst intrusion from 0.144 m to 0.071 m.
        for tilt_deg in self.APPROACH_TILTS:
            tilt = math.radians(tilt_deg)
            direction = np.array([math.cos(tilt), 0.0, math.sin(tilt)])
            for sign, offset in candidates:
                grasp_z = tz + offset
                # Pin the torso so the shoulder is level with the row before
                # solving, then let the 7 arm joints do the rest.  This also
                # keeps the torso a single slow move at the start of the grasp
                # instead of something the arm trajectory has to carry.
                saved = self.lock_torso(self.torso_for(grasp_z))
                try:
                    q, info = self.solve_clear(
                        (grasp_x, ty, grasp_z), base_xy, base_yaw, direction,
                        sign, name='grasp', max_seeds=12)
                except PlanFailure as exc:
                    errors.append('tilt {:+d} deg, {}y offset {:+.3f}: '
                                  '{}'.format(tilt_deg,
                                              '+' if sign > 0 else '-',
                                              offset, exc))
                    continue
                finally:
                    self.unlock_torso(saved)

                info['height_offset'] = offset
                info['approach_tilt_deg'] = tilt_deg
                depth = info.get('shelf_penetration', 0.0)
                if best is None or depth < best[2]:
                    best = (q, info, depth)
                if depth <= self.ACCEPT_PENETRATION:
                    break
            if best is not None and best[2] <= self.ACCEPT_PENETRATION:
                break

        if best is None:
            raise PlanFailure(
                'no reachable grasp for this book; tried ' + '; '.join(errors))

        q_grasp, info, _ = best
        sign = info['finger_sign']
        grasp_z = tz + info['height_offset']
        approach = np.array([math.cos(math.radians(info['approach_tilt_deg'])),
                             0.0,
                             math.sin(math.radians(info['approach_tilt_deg']))])

        # The lift is a small arm motion at the same base pose - the only arm
        # motion that happens with the gripper inside the shelf.
        saved = self.lock_torso(q_grasp[0])
        self._shelf_check = True
        self._base_xy, self._base_yaw = base_xy, base_yaw
        try:
            lift = min(self.LIFT, 0.4 * self.ROW_CLEARANCE)
            q_lift, lift_info = self._solve_fixed_sign(
                (grasp_x, ty, grasp_z + lift), base_xy, base_yaw, approach,
                sign, q_grasp, name='lift')
        except PlanFailure:
            # A lift is preferable but not essential: with mu = 5.0 between
            # the pads and the spine the book comes out on friction alone.
            q_lift, lift, lift_info = None, 0.0, None
        finally:
            self.unlock_torso(saved)
            self._shelf_check = False

        offender = self.shelf_offender(
            q_grasp, base_xy, base_yaw, self._row, self._column_y)

        plan = GraspPlan(
            arm_q=q_grasp,
            lift_q=q_lift,
            lift=lift,
            finger_sign=sign,
            height_offset=info['height_offset'],
            grasp_world=(grasp_x, ty, grasp_z),
            base_grasp=(base_xy[0], base_xy[1], base_yaw),
            approach_distance=self.PREGRASP_BACKOFF,
            retreat_distance=self.EXTRACT,
            info=info,
            shelf_offender=offender,
        )

        self.log(
            'grasp plan: finger axis {}y, grasp {:.3f} m up the spine, torso '
            '{:.3f} m, IK error {:.4f} m, base approach {:.2f} m, retreat '
            '{:.2f} m, tilt {:+d} deg, predicted shelf intrusion {:.3f} m{}'
            .format(
                '+' if sign > 0 else '-', info['height_offset'], q_grasp[0],
                info['position_error'], plan.approach_distance,
                plan.retreat_distance, info['approach_tilt_deg'],
                info.get('shelf_penetration', 0.0),
                '' if q_lift is not None else ', no lift (unreachable)'))
        return plan

    def plan_place(self, bin_world, base_xy, base_yaw,
                   book_half_height=None, clearance=0.06):
        """Plan a gentle placement into the bin from a *measured* bin pose.

        ``bin_world`` is where perception says the bin's interior centre is.
        The release height is derived from the bin's real rim and floor, not
        from a nominal number: the book is lowered until its lower edge is
        just above the bin floor, then released.
        """
        self.warm_up()

        if book_half_height is None:
            book_half_height = arena.BOOK_SPINE_H / 2.0

        bx, by = float(bin_world[0]), float(bin_world[1])

        # Approach direction: horizontally, from wherever the base is towards
        # the bin, so the wrist keeps the book upright on the way in.
        dx, dy = bx - float(base_xy[0]), by - float(base_xy[1])
        norm = math.hypot(dx, dy)
        approach = np.array([dx / norm, dy / norm, 0.0]) if norm > 1e-6 \
            else np.array([-1.0, 0.0, 0.0])

        # Hover: book's lower edge above the rim so nothing snags on the wall.
        hover_z = arena.BIN_RIM_Z + book_half_height + clearance
        # Release: book's lower edge just above the bin floor.
        release_z = arena.BIN_FLOOR_Z + book_half_height + 0.02

        hover_world = (bx, by, hover_z)
        q_hover, info_hover = self._solve(
            hover_world, base_xy, base_yaw, approach, name='bin hover')
        if not info_hover['ok']:
            raise PlanFailure(
                'bin hover unreachable: {:.3f} m short of ({:+.2f}, {:+.2f}, '
                '{:.2f})'.format(info_hover['position_error'], *hover_world))
        if not self.within_limits(q_hover):
            raise PlanFailure('bin hover solution sits on a joint limit')

        sign = info_hover['finger_sign']
        waypoints = [Waypoint('bin_hover', q_hover, hover_world, info_hover)]

        # Descend in two steps so a failure part-way leaves the book above
        # the rim rather than wedged against it.
        for i, z in enumerate(
                (hover_z - (hover_z - release_z) * 0.5, release_z), start=1):
            w = (bx, by, z)
            q, info = self._solve_fixed_sign(
                w, base_xy, base_yaw, approach, sign,
                waypoints[-1].q, name='bin descend {}'.format(i))
            if not info['ok']:
                raise PlanFailure(
                    'bin descent step {} unreachable ({:.3f} m short)'.format(
                        i, info['position_error']))
            waypoints.append(Waypoint('bin_descend_{}'.format(i), q, w, info))

        for a, b in zip(waypoints, waypoints[1:]):
            if self.joint_step(a.q, b.q) > self.MAX_JOINT_STEP:
                raise PlanFailure(
                    'placement plan asks for an unsafe jump between {} and '
                    '{}'.format(a.name, b.name))

        self.log('place plan: release at z={:.3f} m (rim {:.3f}, floor {:.3f})'
                 .format(release_z, arena.BIN_RIM_Z, arena.BIN_FLOOR_Z))
        return waypoints

    def plan_retreat(self, base_xy, base_yaw, from_q, back_off=0.25):
        """Lift the empty gripper straight up and back out of the bin."""
        self.warm_up()
        T = self.chain.fk(np.asarray(from_q, float))
        here = base_to_world(T[:3, 3], base_xy, base_yaw)
        approach = np.array([math.cos(base_yaw), math.sin(base_yaw), 0.0])
        up_world = (here[0], here[1], here[2] + back_off)
        try:
            q, info = self.chain.ik(
                world_to_base(up_world, base_xy, base_yaw),
                target_z_axis=approach, q0=np.asarray(from_q, float))
        except Exception as exc:                          # pragma: no cover
            raise PlanFailure('retreat IK failed: {}'.format(exc))
        if not info['ok']:
            raise PlanFailure('retreat unreachable')
        return [Waypoint('bin_retreat', q, up_world, info)]


def _better(info, best):
    """Ranking used to pick between two IK results."""
    if info['ok'] != best['ok']:
        return bool(info['ok'])
    return info['position_error'] < best['position_error']
