# Open blocker: the grasp reaches the book but does not retain it

**Status: reproducible, characterised, unresolved.**
Recorded so it can be handed to the organisers or picked up directly.

## Environment

| Item | Value |
|---|---|
| Organiser baseline | `dfl-rlab/erc_sim_2026` @ `2aa5a4a` (`v1.0.3-1-g2aa5a4a`) |
| Image | `erc-2026:humble-harmonic` (repo `docker/Dockerfile`) |
| Stack | Ubuntu 22.04.5, ROS 2 Humble, Gazebo Sim 8.15.0 |
| Layout seed | `ERC_SEED=12345` |
| Measured real-time factor | 0.362 on the validation machine |

## What works

Verified repeatedly on the live simulator:

* The base carries the arm into a shelf row with **zero** `erc_shelf` contacts,
  after the wrist-roll / grasp-height / approach-tilt search described in
  `grasp.py`. Before that search the same motion logged 2 000–12 000
  `arm_right_*`/`erc_shelf` contacts per attempt.
* The gripper arrives with its pads straddling the target book's spine:
  measured pads at world y `[0.1691, 0.2003]` and `[0.2524, 0.2844]` against a
  book spine at `[0.2142, 0.2442]`, i.e. 12–14 mm clear either side.
* Closing the gripper produces a **real, sustained contact** with the target
  book on `/contacts`, and the manipulation node reports
  `holding=book_col_3_row_3_red`.

## What does not work

The book is not retained. Lifting 0.020 m, or reversing the base 0.42 m,
separates the gripper from the book; the book stays on the shelf, displaced
0.020–0.042 m by the approach.

## Evidence pointing at the cause

1. **Contact is only ever reported on one fingertip.** Across every attempt
   only `gripper_right_fingertip_left_link` appears against the book, never the
   right one, even when both pads are geometrically inside the spine (pads at
   `0.2263 ± 0.008`, spine `[0.2142, 0.2442]`).
2. **The book's collision declares `<max_contacts>4</max_contacts>`**
   (`erc_description/models/book/sdf/erc_book.sdf`). A four-point budget can be
   consumed entirely by the first finger, leaving the opposing finger with no
   contact constraint — and therefore no opposing normal force, which is what a
   friction grasp needs.
3. **The gripper cannot apply a clamping force.** `gz_ros2_control`'s
   `GazeboSimSystem::write()` implements a **position** command interface as a
   `JointVelocityCmd` computed from position error
   (`gz_ros2_control/src/gz_system.cpp`, the `POSITION` branch). The joint is
   therefore driven kinematically: it reaches whatever it is commanded to
   reach. Measured, commanded and achieved finger positions agree to five
   decimal places (0.03400 → 0.03400) whether or not a book is between the
   pads, and the reported `effort` state is a constant −0.0935 N at every
   position. There is no force to squeeze with.
4. Increasing the squeeze (finger joint 0.024 → 0.014, i.e. 6 mm → 16 mm of
   interference on a 30 mm spine) did not change the outcome.

## Minimal reproducer

```bash
ERC_SEED=12345 ros2 launch erc_bringup simulation.launch.py headless:=true
# in a second shell, with the workspace sourced:
ros2 run rafeeq_solution manipulation_node --ros-args -p use_sim_time:=true
python3 tools/diagnostics/staged_grasp.py          # drives base + arm, logs contacts
```

`tools/diagnostics/staged_grasp.py` prints the book's pose before and after,
the gripper's measured pose, and every `/contacts` pair observed. Ground truth
is used there only as the test's target and oracle; the shipped solution never
reads it.

## What we would ask the organisers

Whether a two-finger friction grasp of `erc_book` is expected to hold given
(a) the `max_contacts` budget on the book and (b) position commands being
realised as kinematic velocity setpoints rather than forces. If it is, what
finger command and approach they validated it with.

## Consequence for scoring

Everything gated on the grasp is unearned: the 3 points for the grasp itself,
the 3 for shelf navigation (awarded on a successful grasp), the 3 for the
return, and the 2–4 for placement. The perception stages are unaffected and
are demonstrated. See `docs/competition_readiness.md`.
