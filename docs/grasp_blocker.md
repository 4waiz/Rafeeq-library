# Grasp retention: what stopped it, and what fixed it

**Status: the grasp now reaches, closes on and extracts the target book;
holding it through the whole return is still being validated.**

This page used to describe an "open blocker" and proposed asking the organisers
whether a two-finger friction grasp of `erc_book` could hold at all.  Every one
of its premises has since been disproven by measurement against Gazebo ground
truth, so it has been rewritten.  Ground truth was used only by the diagnostic
scripts, as a target and an oracle; the shipped solution never reads it.

## Environment

| Item | Value |
|---|---|
| Organiser baseline | `dfl-rlab/erc_sim_2026` @ `2aa5a4a` (`v1.0.3-1-g2aa5a4a`) |
| Image | `erc-2026:humble-harmonic` (repo `docker/Dockerfile`) |
| Stack | Ubuntu 22.04.5, ROS 2 Humble, Gazebo Sim 8.15.0 |
| Layout seeds | `ERC_SEED=12345`, `ERC_SEED=101` |
| Measured real-time factor | 0.362 on the validation machine |
| Book | 0.3 kg box, 0.25 x 0.03 x 0.16 m, mu = 5.0, `max_contacts` 4 |

## What the old page claimed, and what was measured instead

| Old claim | Measured |
|---|---|
| Contact is only ever reported on one fingertip | Both fingertips report contact, 140-260 hits each per 1.2 s window, once the gripper is centred on the spine |
| The four-contact budget starves the second finger | Not observed once the grip is centred; it was a symptom of off-centre grips |
| The gripper cannot apply a clamping force, so nothing can hold | The book is held through the lift and pulled 0.17 m clear of the shelf |
| Squeezing harder does not help | True of off-centre grips, where one pad pushed the book sideways; irrelevant once centred |

## The causes, in the order they were found

1. **The colour pixels were deprojected from the wrong frame.**  Both head
   camera sensors render from `head_front_camera_link`, pixel-aligned, but the
   colour image is labelled `head_front_camera_color_optical_frame`, 0.015 m to
   the side.  Every book measurement was shifted sideways by that much, and the
   pads (0.0144 m of clearance per side) closed beside the spine instead of
   around it.  Fixed with the `deproject_frame` parameter; the gripper now lands
   within 0.0006 m of the spine centre and both pads meet it together.

2. **The pinch was below the book's centre of mass.**  The pads can only reach
   the front ~0.06 m of a 0.16 m-deep book, and a two-pad pinch has almost no
   stiffness about its own axis.  Pinched 0.054 m below centre - the first
   clean grasp the planner found - the book is an inverted pendulum.  Traced in
   truth: it came 0.17 m out of the shelf still held, then flipped about 130
   degrees over the pads as its rear left the board, and fell to the floor.
   The planner now searches grasps *above* the centre first (opposite wrist
   roll, tilt -90 or -70), and only accepts one whose lift also solves.
   Pinched 0.072 m above centre, the book swings about 30-40 degrees into a
   stable hang instead and stays in the jaw.

3. **The base jerked the hanging book.**  The navigation node stepped straight
   to 0.6 rad/s at the start of the 180-degree turn back to the bin; with the
   gripper 0.8 m out, that is a 0.45 m/s sideways jerk at the pads, and it threw
   out a book that had survived the extraction.  Legs driven with the book in
   the jaw now ask for a gentle goal: at most 0.12 m/s and 0.25 rad/s, every
   component ramped.

4. **The retreat drops the book off the board edge.**  At full retreat speed
   the rear of the book drops off the board and it rocks about +/-0.3 rad in
   the pads - on one run of two, enough to shake it out.  The retreat is now a
   gentle goal too, so the rear corner rides down over the edge.

5. **Held on its edge, the book creeps out of the pads.**  With causes 1-4
   fixed, the book came out of the shelf, passed the retention check and was
   carried through the whole 180-degree turn - then slid out.  Truth shows why:
   book-minus-gripper height went -0.118, -0.120, -0.122, -0.124, -0.125,
   -0.126, -0.127, -0.128, -0.134 m over the carry, about a millimetre a
   second, with jumps of 2-3.5 cm whenever the hang swung.  The pinch axis is
   horizontal, so gravity is resisted by friction alone, and every rotation
   about that axis lets the contact go a little.  Two remedies were tried and
   did not work:

   | Tried | Result |
   |---|---|
   | Squeeze 0.016 instead of 0.010 | Still slipped 3.5 cm on a swing at the hold check, and dropped |
   | Pinch +0.054 m instead of +0.072 m (more slide margin) | Swung twice as far off the board edge (pitch 1.52 -> 0.45 rad) and slipped out during the retreat itself |

   Both were reverted, and so was a third attempt: rolling the wrist a
   quarter turn once the book was clear of the shelf, to turn the pinch axis
   vertical and lay the book flat on the lower pad.  Offline the roll is clean
   (`arm_*_7`'s axis is the gripper body axis and the fingertip point does not
   move), but live the book was flung 0.26 m clear of the gripper at the start
   of the roll - about 0.5 m/s from a wrist turning at 0.26 rad/s.

6. **The pads are held at their angle by 0.1 N m.**  The organisers' gripper
   is a linkage: one actuated prismatic joint (`gripper_*_finger_joint`,
   effort limit 10 N) and ten joints that mimic it, among them the fingertip
   hinges that carry the pads.  Every mimic is declared to `gz_ros2_control`,
   which enforces it with a velocity servo, and the simulator caps that servo
   at the joint's effort limit - **0.1 N m** for the fingertip and finger
   hinges.  That explains everything in cause 5 at once: squeezing harder
   loads the hinges more and saturates them sooner (0.016 slipped *more* than
   0.010), the book's weight on a saturated hinge back-drives it slowly (the
   creep), a swing loads it suddenly (the slips), and the roll put the whole
   weight across the hinges (the fling).  Friction needs very little: with
   mu = 5, about 0.3 N of pad force per side carries the 0.3 kg book.  So the
   squeeze is being taken *down*, to keep the hinge servos out of saturation.

## Diagnostics

* `tools/diagnostics/staged_grasp.py` - drives base and arm through a grasp and
  logs every `/contacts` pair.
* The truth watcher used for causes 2-4 samples the target book's pose and the
  gripper's pose once a second from the lift onwards, alongside which
  fingertips report contact; its output for each trial is quoted above.

## Consequence for scoring

With the grasp retained through extraction, the grasp (3) and shelf-navigation
(3) points are earned on the live simulator.  The return (3) and placement
(2-4) points depend on the carry, which is what is being validated now.  See
`docs/competition_readiness.md`.
