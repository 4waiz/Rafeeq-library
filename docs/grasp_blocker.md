# Grasp retention: what stopped it, and what fixed it

**Status: settled.**  The book is grasped above its centre of mass, pulled
clear of the shelf, carried and delivered into the collection bin.  A recorded,
unedited run logged

```
OFFICIAL COMPLETION EVENT: book_col_3_row_2_red contacted the collection bin at t+107.8 s
```

on the organisers' `/bin_contacts` topic - not on the gripper opening.  The
carry is still timing-critical: the grip lasts a measured 6-11 s once the book
hangs free and on that run the carry - `CARRY_TO_BIN` to `RELEASE` - took
7.8 s, so a run can still drop the book.  That margin is the subject of the
last two sections.

This page began as an "open blocker" and proposed asking the organisers whether
a two-finger friction grasp of `erc_book` could hold at all.  Every one of its
premises was disproven by measurement against Gazebo ground truth, so it has
been rewritten twice: first as a list of open causes, now as the settled
account of what each cause was, how it was found and what fixed it.  Ground
truth was used only by the diagnostic scripts, as a target and an oracle; the
shipped solution never reads it (`tools/diagnostics/README.md`).

## Environment

| Item | Value |
|---|---|
| Organiser baseline | `dfl-rlab/erc_sim_2026` @ `2aa5a4a` (`v1.0.3-1-g2aa5a4a`) |
| Image | `erc-2026:humble-harmonic` (repo `docker/Dockerfile`) |
| Stack | Ubuntu 22.04.5, ROS 2 Humble, Gazebo Sim 8.15.0 |
| Layout seeds | `ERC_SEED=12345` and `ERC_SEED=101`; the delivered run was `ERC_SEED=101`, `shelf_column_number:=2`, `book_colour:=red` |
| Measured real-time factor | 0.36 with software rendering; 0.65-0.70 headless on the GPU through Mesa's d3d12 driver under WSL; 0.56 with the Gazebo GUI up |
| Book | 0.3 kg box, 0.25 x 0.03 x 0.16 m, `mu` = `mu2` = 5.0, `max_contacts` 4 (`src/erc_description/models/book/sdf/erc_book.sdf`) |
| Gripper | one actuated prismatic joint (`gripper_right_finger_joint`, effort limit 10 N) and seven joints that mimic it, six of them revolute with an effort limit of **0.1 N m** (`src/erc_description/urdf/tiago_pro.urdf`) |

## What the old page claimed, and what was measured instead

| Old claim | Measured |
|---|---|
| Contact is only ever reported on one fingertip | Both fingertips report contact, 140-260 hits each per 1.2 s window, once the gripper is centred on the spine |
| The four-contact budget starves the second finger | Not observed once the grip is centred; it was a symptom of off-centre grips |
| The gripper cannot apply a clamping force, so nothing can hold | The book is held through the lift, pulled 0.17 m clear of the shelf, and - with the rest of this page applied - carried to the bin and delivered |
| Squeezing harder does not help | True of off-centre grips, where one pad pushed the book sideways; and true again for a different reason once centred, see cause 6 |
| A friction pinch of `erc_book` may simply be impossible, and the organisers should be asked | It is possible.  The limit is not whether the hold exists but how long it lasts: about a millimetre a second of creep, 6-11 s from the moment the book hangs free |

## The causes, in the order they were found

1. **The colour pixels were deprojected from the wrong frame.**  Both head
   camera sensors are mounted on `head_front_camera_link` with no pose, so
   Gazebo renders colour and depth from the same point and the two images are
   pixel-aligned.  The colour image is nevertheless *labelled*
   `head_front_camera_color_optical_frame`, which TF places 0.015 m to the
   side, and deprojecting in that label shifted every book measurement
   sideways.  Measured against ground truth, the target came back +0.019 m out
   relative to the robot, and the gripper closed 0.0141 m off a spine it
   clears by only 0.0144 m per side at full opening.  The error was invisible
   to the number-plate check because the plate carried exactly the same offset
   and the difference cancelled it.

   Fixed with the `deproject_frame` parameter, now
   `head_front_camera_depth_optical_frame`
   (`src/rafeeq_solution/rafeeq_solution/perception_node.py`).  With the frame
   corrected the gripper arrives centred enough that both pads meet the spine
   together: the first pad reports contact at a 0.0307 m gap on a 0.030 m
   spine.

2. **The pinch was below the book's centre of mass.**  The pads can only reach
   the front 0.06 m or so of a 0.16 m-deep book, and a two-pad pinch has almost
   no stiffness about its own axis, so once the book's rear leaves the board it
   swings until its centre of mass hangs under the pinch.  Pinched 0.054 m
   below centre - the first clean candidate the planner found, and what it used
   to take - the book is an inverted pendulum.  Traced in ground truth: it came
   0.17 m out of the shelf still held, then flipped about 130 degrees over the
   pads as its rear left the board, and fell to the floor.

   The planner now searches grasps *above* the centre first, highest first, and
   only accepts one whose lift also solves (`GraspPlanner.plan_grasp` in
   `grasp.py`).  Those grasps need the opposite wrist roll (-y) and their own
   approach tilts (-90 or -70 degrees), which is why the tilts are ordered per
   roll: trying +90/+70 first, which only suit +y, cost a measured 30 s of
   failed IK per plan.  Pinched 0.072 m up the spine - the highest candidate in
   the runs this was traced on - the book swings through about 40 degrees
   instead of 130, with a quarter of the energy, and stays in the jaw.

   Highest first, not lowest, even though a lower pinch leaves more slide
   margin: at +0.054 m the book swung twice as far off the board edge (pitch
   1.52 -> 0.45 rad, against 1.52 -> 1.06 rad at +0.072 m) and slipped out
   during the retreat itself.

3. **The base jerked the hanging book.**  The navigation node stepped straight
   to 0.6 rad/s at the start of the 180-degree turn back to the bin; with the
   gripper 0.8 m out, that is a 0.45 m/s sideways jerk at the pads, and it
   threw out a book that had survived the whole extraction.  Legs driven with
   the book in the jaw now ask for a gentle goal - at most 0.12 m/s and
   0.25 rad/s, ramped at 0.15 m/s^2 and 0.25 rad/s^2, every component ramped
   (`gentle_*` parameters in `navigation_node.py`).

4. **The retreat dropped the book off the board edge.**  At full retreat speed
   the rear of the book drops off the board and it rocks about +/-0.3 rad in
   the pads - on one run of two, enough to shake it out.  The retreat is a
   gentle goal too, so the rear corner rides down over the edge instead.

   The retreat distance was tuned against the same measurements.  It is
   `EXTRACT = 0.34` m (`grasp.py`): 0.42 m was more than the 0.225 m needed to
   clear the shelf front plane and cost about 2.5 s of a grip that only lasts
   6-11 s, while 0.28 m was too short once the carry tolerance is taken into
   account - the leg may stop 0.06 m short, the book's rear is measured leaving
   the board between 0.233 m and 0.247 m of retreat, and a leg finishing inside
   that band left the following turn dragging a book still resting on the
   shelf.

5. **Held on its edge, the book creeps out of the pads.**  With causes 1-4
   fixed, the book came out of the shelf, passed the retention check and was
   carried through the whole 180-degree turn - then slid out.  Ground truth
   shows why: book-minus-gripper height went -0.118, -0.120, -0.122, -0.124,
   -0.125, -0.126, -0.127, -0.128, -0.134 m over the carry, about a millimetre
   a second, with jumps of 2-3.5 cm whenever the hang swung.  The pinch axis is
   horizontal, so gravity is resisted by friction alone, and every rotation
   about that axis lets the contact go a little.  Three remedies were tried and
   none worked:

   | Tried | Result |
   |---|---|
   | Squeeze 0.016 instead of 0.010 | Still slipped 3.5 cm on a swing at the hold check, and dropped |
   | Squeeze 0.004 instead of 0.010 | Crept at the same rate and fell 5 s into the carry instead of 10 |
   | Pinch +0.054 m instead of +0.072 m (more slide margin) | Swung twice as far off the board edge (pitch 1.52 -> 0.45 rad) and slipped out during the retreat itself |

   So was a fourth: rolling the wrist a quarter turn once the book was clear of
   the shelf, to turn the pinch axis vertical and lay the book flat on the
   lower pad.  Offline the roll is clean (`arm_*_7`'s axis is the gripper body
   axis and the fingertip point does not move), but live the book was flung
   0.26 m clear of the gripper at the start of the roll - about 0.5 m/s from a
   wrist turning at 0.26 rad/s.  All four were reverted.

6. **The pads are held at their angle by 0.1 N m.**  The organisers' gripper is
   a linkage: one actuated prismatic joint (`gripper_right_finger_joint`,
   effort limit 10 N) and seven joints that mimic it.  Six of those are
   revolute - the inner finger, outer finger and fingertip hinges on each side
   - and each declares an effort limit of **0.1 N m**.  Every mimic is also
   declared to `gz_ros2_control` as a `<param name="mimic">`, which enforces it
   with a velocity servo, and the simulator caps that servo at the joint's
   effort limit.

   That explains everything in cause 5 at once.  Squeezing harder presses the
   pads harder, the reaction torque on each hinge grows with it, and once the
   servo saturates the book's weight back-drives the fingertip - which is why
   0.016 slipped *more* than 0.010.  The book's weight on a saturated hinge
   back-drives it slowly, which is the creep; a swing loads it suddenly, which
   is the jumps; and the wrist roll put the whole weight across the hinges,
   which is the fling.  Friction itself needs very little: with mu = 5, about
   0.3 N of pad force per side carries the 0.3 kg book.

   The squeeze is therefore neither raised nor lowered.  `CLOSE_SQUEEZE` stays
   at 0.010 (`manipulation_node.py`), because a lighter squeeze measured no
   better.  What was added is a floor, not a ceiling: `CLOSE_TARGET_GAP` =
   0.0199 m.  The fingers step in until a pad reports contact and the squeeze
   is measured from that report, but the report lands on a 0.004-unit grid, so
   the realised grip swung a whole increment between runs - five takes closed
   to 0.0199 m (10.1 mm of interference on a 0.030 m spine) and one closed to
   0.0230 m (7.0 mm), and that loose one gave the shortest carry of the six.
   The clamp only ever tightens a slack close to the value most runs already
   achieve; it never squeezes harder than that.

7. **The creep cannot be removed, so the carry was made short enough to finish
   inside it.**  There is no attach or detachable-joint mechanism anywhere in
   the organisers' world or robot to fall back on, and the hinge limit is part
   of the supplied URDF.  The remaining lever is time.  The old return - drive
   to the start zone, search for the bin, line up on it, plan a placement - took
   25-40 s against a grip of 6-11 s, and dropped the book every time.  What
   replaced it:

   * **The bin is identified visually before the shelf trip.**  `FACE_BIN` runs
     immediately after `HOME`, from the start zone (`mission_node.py`).  The
     bin is still measured from the camera, only earlier.
   * **The return is a drive, not a manipulation.**  After the lift the base
     drives straight to the pose that puts the gripper - holding the book
     exactly as it is - over the bin's mouth, and the book is released there.
     The arm does not move, so nothing swings the book.
   * **The drive starts while the hold check runs.**  The retention check takes
     about 1.9 s (it waits 1.0 s, then requires contact inside a 1.5 s window),
     which is a fifth of the budget to spend standing still.  The gripper's
     pose comes back with the lift result, so the release pose is already known
     and the drive can begin.
   * **The torso lowers on the way**, by at most 0.15 m
     (`CARRY_MAX_TORSO_DROP`).  At 0.035 m/s that is 4.3 s, inside the drive
     the release waits for anyway.
   * **The aim point is 0.05 m short of the bin's middle**
     (`CARRY_NEAR_OFFSET`).  0.15 m was tried to shorten the drive further: the
     book hangs about 0.05 m behind the pinch, so its centre sat barely 0.08 m
     inside the near wall and the first run to carry the book the whole way
     released it onto the rim.
   * **The carry arrives to 0.06 m and 0.04 rad**, not to millimetres, and with
     a harder approach gain (2.0) so it does not crawl the last 0.2 m at
     0.05-0.12 m/s.  The bin's mouth is 0.56 x 0.31 m and the release is a drop
     into it, so that is enough.

   The speeds that carry the book are set by acceleration, not by the cap.
   Measured live, the honest unit is how far the book survives:

   | Acceleration | Speed cap | Book survived |
   |---|---|---|
   | 0.30 m/s^2 | 0.45 m/s | 0.92 m |
   | 0.15 m/s^2 | 0.12 m/s | 0.95 m |
   | 0.15 m/s^2 | 0.45 m/s | 1.75 m |

   Halving the acceleration nearly doubled the distance while the cap barely
   mattered, so the carry runs at 0.08 m/s^2 and 0.12 rad/s^2 with caps of
   0.45 m/s and 0.60 rad/s, ramped in the odom frame and growth-limited only,
   so the base can still stop.  The turn rate is a separate parameter on
   purpose: raising the ordinary legs to 0.90 rad/s also sped up the turn made
   with the book hanging, and the book came out after 3.4 s instead of 8.8 s on
   the run before it.

## What the carry costs, and why a run can still drop the book

The grip and the drive are the same length, and that is the whole margin:

| Measured | Value |
|---|---|
| How long the book stays held once it hangs free | 6-11 s |
| Drive from the shelf to the release over the bin | 7.8 s on the delivered run (`CARRY_TO_BIN` +99.3 s to `RELEASE` +107.1 s) |
| Retention check, run while the drive is already moving | about 1.9 s; 1.2 s on the delivered run |
| Creep rate while hanging | about 1 mm/s, with 2-3.5 cm jumps on a swing |

On the delivered run the book was clear of the shelf with retention confirmed
at t+99.3 s and contacted the bin at t+107.8 s.  A run whose grip lands at the
6 s end of the band, or whose extraction leaves the book swinging, will drop it
before the bin.  Nothing here makes delivery certain; it makes the drive short
enough to fit inside a hold that is known to be temporary.

## Diagnostics

* `tools/diagnostics/staged_grasp.py` - drives base and arm through the whole
  grasp sequence and logs every `/contacts` pair alongside the book's
  displacement.
* `tools/diagnostics/deproj.py` - deprojects every detected book and prints
  measured against true world positions.  This is what exposed cause 1.
* `tools/diagnostics/book_clearance.py` - clearance from the target book for
  every approach tilt and wrist roll over the whole base advance.
* The truth watcher used for causes 2-5 samples the target book's pose and the
  gripper's pose once a second from the lift onwards, alongside which
  fingertips report contact.  Its output for each trial is quoted above.

Ground truth appears only in these scripts.  `src/rafeeq_solution/` contains no
call to `gz model`, no subscription to a ground-truth pose topic and no
hard-coded book position.

## The delivered run

`ERC_SEED=101`, `shelf_column_number:=2`, `book_colour:=red`.  One continuous
take, no speed-up, 4:34 long, with the on-screen trial timer reading 221.3 s at
the delivery.

Two clocks, and they are not interchangeable.  Every `t+` on this page is
*simulated* seconds, the clock the mission runs on (`use_sim_time`); the
on-screen timer is wall clock, started at `solution.launch.py`.  107.8
simulated seconds inside 221.3 wall seconds is a realised real-time factor of
0.487 for this take.

| t | Stage entered |
|---|---|
| +21.7 s | `DETECT_BIN` - the collection bin identified from the start zone |
| +31.9 s | `DETECT_COLUMN` - number plate 2 read above physical column 3 |
| +48.9 s | `DETECT_BOOK` - the red book found in row 1 and measured from the camera |
| +82.9 s | `CLOSE_GRASP` - the pads close on the spine |
| +91.9 s | `LIFT` |
| +94.1 s | `RETREAT` - the book pulled clear of the shelf |
| +98.1 s | `CONFIRM_HOLD` - the retention check, with the drive to the bin already moving |
| +99.3 s | `CARRY_TO_BIN` - the hold confirmed |
| +107.1 s | `RELEASE` |
| +107.8 s | `book_col_3_row_2_red` contacted the bin on `/bin_contacts` |
| +109.3 s | `CONFIRM_DELIVERY` |

The delivered model is named `book_col_3_row_2_red` because physical slot 3
carries the printed plate "2" and the SDF's `row_2` is competition row 1.  That
name was read back off the completion event afterwards as a consistency check;
nothing in the run selected on it.

The take ends on the organisers' `/bin_contacts` event, which is the task being
finished, so there is no `RUN ... COMPLETE` line and no machine-readable trial
record for it.

## Consequence for scoring

The grasp (3) and shelf-navigation (3) points are awarded at the retention
check, which is the first moment a grasp can be proven.  The return with the
book (3) is awarded on the `/bin_contacts` event, not on the gripper opening.
The direct carry releases the book above the rim rather than lowering it, so
under this repo's own stated rule (`_classify_placement` in `mission_node.py`)
it is recorded as a drop (2) and not a gentle placement (4).  See
`docs/competition_readiness.md`.

## Offline coverage

162 tests, no ROS and no Gazebo needed:

```bash
python3 -m pytest src/rafeeq_solution/test -q
```
