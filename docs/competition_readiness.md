# Competition readiness

Every Phase 1 requirement, what implements it, how it was checked, where the
evidence is, and an honest status.

**Status vocabulary**

| Status | Meaning |
|---|---|
| **verified** | Demonstrated on the live simulator, with evidence recorded |
| **verified, intermittent** | Demonstrated live, and measured to fail on some runs; the cause is characterised, not guessed |
| **implemented, unverified** | Code exists and passes offline tests, not yet proven live |
| **blocked** | Implemented, demonstrated *not* to work, cause characterised |
| **missing** | Not done |

"Verified" means it has been done at least once on the live simulator with the
evidence kept. It does not mean every run does it. Where a stage is known to
depend on timing, it is marked **verified, intermittent** and the measured
reason is given.

Validation baseline for everything below: organiser `dfl-rlab/erc_sim_2026`
@ `2aa5a4a` (`v1.0.3-1-g2aa5a4a`), image `erc-2026:humble-harmonic`,
Ubuntu 22.04.5 / ROS 2 Humble / Gazebo Sim 8.15.0. Gazebo now renders on the
GPU through Mesa's `d3d12` driver under WSL: measured real-time factor
0.65–0.70 headless and 0.56 with the GUI up, against 0.36 in software. See
[`measured_environment.md`](measured_environment.md).

---

## Reference run

The run this assessment is written against. Recorded unedited, one take, with
`tools/record_trial.sh`.

It replaces an earlier take of the same trial. That run was genuine, but its
footage was not usable: the viewer built its scene before the robot had been
spawned into the world, so the recording showed the arena — shelf, table,
bin — with no robot in it for the whole run. The recorder now starts the world
`headless:=true` and attaches the viewer (`gz sim -g`) only once `tiago_pro` is
in the model list, then *proves* the robot is in the viewer's scene before the
trial starts by asking the viewer to follow it and reading
`/gui/currently_tracked`: a named follow target means the viewer has the robot,
an empty one means it does not. It restarts the viewer once and then abandons
the take rather than record another empty arena.

| Item | Value |
|---|---|
| Arguments | `ERC_SEED=101`, `shelf_column_number:=2`, `book_colour:=red` |
| Bin identified, from the start zone, before the shelf trip | `DETECT_BIN` t+21.7 s, satisfied by t+23.5 s |
| Number plate 2 read above physical column 3 | `DETECT_COLUMN` t+31.9 s, satisfied by t+34.7 s |
| Red book confirmed in row 1 | `DETECT_BOOK` t+48.9 s, satisfied by t+51.9 s |
| Grasp closed, book lifted | `CLOSE_GRASP` t+82.9 s, `LIFT` t+91.9 s |
| Clear of the shelf, retention confirmed | `RETREAT` t+94.1 s, `CONFIRM_HOLD` t+98.1 s, carry away at t+99.3 s |
| Released over the bin, delivery reported by the organisers' own sensor | `RELEASE` t+107.1 s, contact at t+107.8 s |
| Completion event | `OFFICIAL COMPLETION EVENT: book_col_3_row_2_red contacted the collection bin at t+107.8 s` |
| Video | 4:34 (274 s), 1920x1080 at 15 fps, 11.7 MiB; the on-screen trial timer reads 221.3 s at the delivery and freezes on `BOOK DELIVERED` |

Every state entry in that take, in simulated seconds: `WAIT_READY` 0.0,
`HOME` 2.1, `FACE_BIN` 17.1, `BIN_POSE` 20.7, `DETECT_BIN` 21.7,
`FACE_SHELF` 23.5, `SCAN_POSE` 28.9, `DETECT_COLUMN` 31.9, `NAV_COLUMN` 34.7,
`COLUMN_REF_POSE` 43.1, `COLUMN_REF` 45.1, `BOOK_POSE` 45.9,
`DETECT_BOOK` 48.9, `NAV_STAGE` 51.9, `REFRESH_BOOK` 56.3, `PLAN_GRASP` 60.7,
`ARM_PREGRASP` 63.9, `NAV_GRASP` 70.9, `REFINE` 81.5, `CLOSE_GRASP` 82.9,
`LIFT` 91.9, `RETREAT` 94.1, `CONFIRM_HOLD` 98.1, `CARRY_TO_BIN` 99.3,
`RELEASE` 107.1, `CONFIRM_DELIVERY` 109.3, `BACK_OFF` 109.5. The table above is
read off that list: `mission_node.transition` logs `[STATE]` when a state is
*entered*, so a stage is satisfied at the moment the next one begins.

Two clocks appear above and they do not disagree. The mission's `t+` figures
are **simulated** seconds (`elapsed_total`, `use_sim_time`, written to the
trial record as `simulated_seconds_total`); the on-screen timer in the video is
wall clock, started when `solution.launch.py` is launched
(`tools/record_trial.sh`). 107.8 s of simulation in 221.3 s of wall clock is a
realised factor of 0.487 for a take with the viewer and a screen recorder
running — and a lower bound, because the timer starts before the mission clock
does.

The delivered model name is the consistency check, not the selector: physical
column slot 3 is where plate 2 was read, organiser model row 2 is competition
row 1, and the colour is red. `scoring.DeliveryMonitor.colour_matches` records
that agreement after the fact and is never used to find a book.

---

## 1. Launch interface

| Requirement | Implementation | Verification | Status |
|---|---|---|---|
| `ros2 launch erc_bringup simulation.launch.py` runs unmodified | No organiser file is modified — `git diff --name-only 2aa5a4a HEAD` touches only `src/rafeeq_solution/`, `tools/`, and new top-level directories | Diff checked; simulation launched from the organiser file for every trial, and typed on camera for the recorded run — with the organisers' own `headless:=true` argument, the viewer attached separately once the robot is in the world | **verified** |
| `ros2 launch rafeeq_solution solution.launch.py shelf_column_number:=N book_colour:=C` works with no other mandatory argument | [`solution.launch.py`](../src/rafeeq_solution/launch/solution.launch.py) — only those two are required; everything else defaults | Run as the judges would for every trial in `results/` and for the recorded run | **verified** |
| No browser, no manual arm tuning, no separate start command | `auto_start` defaults true; the mission starts itself once readiness is *observed* | Trial logs show `WAIT_READY → HOME` with no operator input | **verified** |
| Inputs validated (column 1–5, colour red/blue/green/yellow) | `MissionNode._validate_inputs`, before anything is commanded | `test_mission_flow.py::test_invalid_*` and the 20-combination parametrised test | **verified** |
| One arm only | `arm_side` parameter selects one; the other is tucked, never used to grasp | `arm_poses.yaml` `tucked`/`tucked_left`; trial logs | **verified** |
| Declared, installable dependencies | `package.xml` lists `control_msgs`, `action_msgs`, `ros_gz_interfaces`, `python3-opencv`, `python3-numpy`, `python3-yaml`, … | Built with `colcon build` in the stock organiser image | **verified** |

## 2. Perception evidence

| Requirement | Implementation | Verification | Status |
|---|---|---|---|
| `/erc/shelf_column_identification` (`Int32`) published **only after** the marker is identified | `perception_node._do_column` publishes after `confirm_frames` consistent deprojected votes | Trial logs: `COLUMN CONFIRMED: number 2 is above physical column 3` precedes publication (`results/T01_seed101_col2_red.log` — the same seed, column and colour as the recorded run) | **verified** |
| `/erc/shelf_row_identification` (`Int32`) after the book is identified | `perception_node._do_book` | Trial logs: `BOOK CONFIRMED: red book in row 1 (ordering confirmed by height)` — the row published is the competition row, not the organisers' model row | **verified** |
| Both latched for late subscribers | `TRANSIENT_LOCAL` QoS + 2 s re-publish timer | Topic QoS inspected live | **verified** |
| Shelf image shows a box around the **column**, not just the marker | `vision.column_box_from_plate` grows the plate box down over its column | `erc_images/shelf_column_*.png` — tall box over the column plus a small marker box | **verified** |
| Book image shows the target book and its row | `perception_node._do_book` annotates the book, the row, the measured position, and the offset from the column centre **measured against the plate that was read** — against the nominal centre it would print mostly odometry drift and look like a fault | `erc_images/target_book_*.png` | **verified** |
| Images are live, timestamped, generated during the trial | Written by the running node at detection time, filename carries the timestamp, the frame carries the team name and result | File timestamps match the trial logs | **verified** |
| Output directory resolves without a root symlink or a hard-coded path | `perception_node._resolve_image_dir` — parameter, then `ERC_IMAGES_DIR`, then a walk up to the workspace's `erc_images/` | Runs unprivileged in the container and from a plain clone | **verified** |

## 3. Autonomy

| Requirement | Implementation | Verification | Status |
|---|---|---|---|
| No state advances on a timeout | `mission_node` — every `check_*` gates on an observation; timeouts retry (bounded) or `FAILED` | `test_mission_flow.py` asserts this for readiness, manipulation, navigation, grasp, delivery | **verified** |
| Command acceptance / motion completion / grasp verification are separate | Manipulation replies `busy` → `done`/`failed` with a sequence number; grasp verified separately from the close command | `test_mission_flow.py::test_a_reply_to_an_older_command_is_ignored` and live logs | **verified** |
| Motions complete against real feedback, not elapsed time | `FollowJointTrajectory` **actions** for arm/torso/head, cross-checked against fresh `/joint_states`, with a bounded settle window | Live failures name the joint and the error, e.g. *"finished 1.196 rad off target"* | **verified** |
| Trajectories respect real joint limits | Time-parameterised from the URDF's per-joint `velocity` limits (`kinematics.joint_velocity_limits`), run at 0.80 of them | Torso at 0.035 m/s no longer under-runs its trajectory | **verified** |
| Measured book position drives the grasp | `perception_node` publishes the measured world point; `mission_node` passes it to `manipulation_node`; `grasp.plan_grasp` plans from it | Recorded run: the grasp was planned from the measured point and closed on `book_col_3_row_2_red`, held with the tool at (0.573, −0.025, 1.671) — x and y in the base frame, z in the world. That take's measured book point is in its `*_solution.log`, which is not in this repository; the same seed, column and colour in `results/T01_seed101_col2_red.log` measured it at (+2.882, −0.336, 1.581) | **verified** |
| Column filter cannot select a same-colour book from another column | The marker plate is re-read from the book-scan pose itself (`COLUMN_REF`), and a book belongs to this column when its own measured `y` is within the jitter band of that plate. Plate and books share the pose error, so it cancels. | Live: plate at `-0.428` where truth is `0.000`, book at `-0.599`; the difference `-0.171` against a true `-0.154`. `test_contracts.py::test_the_neighbouring_columns_book_is_not_chosen` and four more | **verified** |
| Membership tested per book, not per cluster | Perspective can merge two columns into one image cluster; such a cluster has a plausible mean and the wrong contents | `test_contracts.py::test_a_cluster_straddling_two_columns_is_split_by_the_plate`, from the seed 404 failure | **verified** |
| The plate above the target column can be read at all | It is the steepest in the arena, so its digit is foreshortened; the aspect band runs to 1.40 and the template score does the real work | `test_contracts.py::test_a_foreshortened_plate_digit_is_still_read` — the live digit was 56x49 px, aspect 1.14, matching at 0.827 | **verified** |
| Transforms taken at image time | `world_point` uses `header.stamp`; falls back to the latest transform **only while the base is stationary**, and drops the frame while moving | On this bridge images are stamped ~1.7 s ahead of `/tf`; behaviour logged once at info level | **verified** |
| RGB↔depth 15 mm baseline corrected | `world_point` shifts the depth lookup by `fx·baseline/range` | Measured frame offset (0.015, 0, 0) | implemented, unverified |
| Grasp verified from real observation | Sustained fingertip↔book contact on `/contacts`, re-checked after the lift and again after the 0.34 m base extraction (`grasp.EXTRACT`) | Live: contact registers and the retention step reports `held=book_col_3_row_2_red tool=0.573,-0.025,1.671 torso=0.350` | **verified** |
| The grasp holds the book rather than toppling it | `grasp.plan_grasp` searches grasps **above** the book's centre of mass first (measured +0.072 m up the spine, wrist roll −y, approach tilt −90 or −70 deg) and accepts one only if its lift also solves. Pinched 0.054 m below centre the book is an inverted pendulum: in ground truth it came 0.17 m out of the shelf still held, flipped over the pads and fell | Recorded run: lifted at t+91.9 s, clear of the shelf and still held when the carry began at t+99.3 s. `grasp_blocker.md` cause 2 | **verified** |
| The close is a measured grip, not a fixed command | The fingers step in until a pad reports contact, then squeeze by `CLOSE_SQUEEZE` (0.010), clamped so the final pad gap is never slacker than `CLOSE_TARGET_GAP` (0.0199 m on a 0.030 m spine) | Measured across six takes: five closed to 0.0199 m, one to 0.0230 m, and the loose one gave the shortest carry of the six | **verified** |
| Book acquired, retained, transported | Above-centre grasp, ramped 0.34 m extraction, then the direct carry | Recorded run: acquired, carried 2.3 m and delivered; the carry itself took 7.8 s (`CARRY_TO_BIN` t+99.3 s → `RELEASE` t+107.1 s). The grip is measured to keep a freely hanging book only 6–11 s, so this is timing-critical — see §5 and gap 1 | **verified, intermittent** |
| Bin visually identified, not substituted with a nominal pose | `_do_bin` requires size, shape, solidity **and** a deprojected position inside the bin's real bounds; failure fails the state, and the search runs from the start zone *before* the shelf trip | Recorded run: bin identified in `DETECT_BIN`, t+21.7 s to t+23.5 s, before the mission turned to the shelf. `test_mission_flow.py::test_bin_detection_failure_does_not_fall_back_to_a_nominal_pose`, `::test_the_bin_is_found_before_the_shelf` | **verified** |
| Delivery verified on `/bin_contacts`, not inferred | `scoring.DeliveryMonitor` ignores the permanent bin↔table contact and fires only on a *book*; the mission refuses to call the run delivered just because the gripper opened | Recorded run: the official completion event at t+107.8 s, while the gripper was still opening in `RELEASE`. `test_scoring.py::test_bin_resting_on_the_table_is_not_a_delivery`, `test_mission_flow.py::test_delivery_requires_the_official_contact_not_just_letting_go` | **verified** |
| Book's measured distance is the book, not the shelf behind it | `image_utils.patch_depth` takes the near surface rather than a median over the patch, and perception adds half the book's depth so it reports the centre the planner assumes | Against Gazebo ground truth: face within 0.011 m on the target column; systematic error in book x from −0.053 m to −0.002 m over five trials | **verified** |
| Base keeps clear of the shelf when the book moves | `GRASP_STANDOFF` 0.60 — it had been absorbing the depth bias, and correcting the book without it halved the base's clearance | `test_contracts.py::test_the_base_keeps_its_clearance_from_the_shelf` | **verified** |
| Arm clears the target book on the way in | `grasp.book_offender` sweeps the whole base advance; acceptance takes the worse of the shelf and book figures. Margin is twice the link radius, since the model walks link origins | Enumerated every tilt and roll for the book that collided: chosen plan went from 0.065 m to 0.179 m of clearance. `test_contracts.py::test_the_plan_that_collided_is_now_rejected`. The sweep covers the way *in* only: on the recorded run `arm_right_6_link` touched the same book on the way out, during `RETREAT` — the one counted collision of that take, see §5 | **verified** |
| Bounded retries and explicit failure | `grasp_attempts` cap, `RECOVER` state, `FAILED` is terminal and reported | Live: 3 grasp attempts then `FAILED` | **verified** |
| Stale results cannot advance a new state | Sequence numbers on every manipulation reply; perception clears votes on mode change | `test_mission_flow.py::test_a_reply_to_an_older_command_is_ignored` | **verified** |

## 4. Navigation

| Requirement | Implementation | Verification | Status |
|---|---|---|---|
| Orients itself from the start condition, no extra argument | Spawn yaw is a known constant of the arena; the mission turns to face the shelf as its first move | Every trial | **verified** |
| Omni-directional obstacle checking | `ScanSector` bins both lidars into 36 sectors; the commanded direction is checked, not just "ahead" | Code + `blocked` transitions in live logs | implemented, partially verified |
| Stale odometry/scan stops the base | `data_fresh` / `scans_fresh` gate the controller | Not yet exercised live | implemented, unverified |
| `blocked` is not `reached` | `nav_gate` treats them separately and fails on a persistent block | `test_mission_flow.py::test_navigation_blocked_is_not_arrival` | **verified** |
| Ordinary legs are fast, the grasp approach is not | 0.50 m/s and 0.90 rad/s for ordinary legs; the millimetre approach is capped at 0.30 m/s and holds 0.006 m | Live trials; `navigation_node` parameters | **verified** |
| A leg driven with a book in the jaw cannot jerk it out | The carry uses its own caps (0.45 m/s, 0.60 rad/s) ramped at 0.08 m/s² and 0.12 rad/s² in the odom frame, growth-limited only so the base can still stop, arriving to 0.06 m and 0.04 rad | What loses the book is acceleration, not speed: measured 0.30 m/s² lost it after 0.92 m and 0.15 m/s² after 1.75 m, against a carry that needs 2.3 m. Recorded run completed the carry | **verified** |
| A bin fix arriving mid-carry cannot restart the drive | `check_carry_to_bin` records a fresh measurement and keeps the release pose: re-issuing the goal resets the ramp to standstill, which is a ~9 m/s² deceleration spike at 0.45 m/s | `test_mission_flow.py::test_a_bin_fix_on_the_move_does_not_restart_the_drive`, `::test_a_small_bin_correction_is_not_chased` | **verified** (offline); the re-measure has never fired live, because perception will not deproject while the base is moving |
| Planned vs executed path published for RViz | `/rafeeq/nav/planned_path` and `/rafeeq/nav/executed_path` (`nav_msgs/Path`), labelled as waypoints — not a Nav2 plan | Topics published; screenshots outstanding | implemented, unverified |
| Predictable shutdown; a held book is not dropped | `manipulation_node.shutdown` cancels goals and deliberately leaves the gripper closed | Code reviewed; not exercised live | implemented, unverified |

## 5. Scoring

### What the rubric awards, and where the code awards it

| Rubric stage | Points | Awarded in | Live status |
|---|---|---|---|
| Identify shelf column — topic publication | 1 | `check_detect_column`, after the plate is read | **verified** |
| Identify shelf column — annotated image | 2 | `check_detect_column`, only when a file is actually on disk | **verified** |
| Navigate to the correct shelf | 3 | `check_confirm_hold`, on a *proven* grasp | **verified** |
| Identify book row — topic publication | 1 | `check_detect_book` | **verified** |
| Identify book row — annotated image | 2 | `check_detect_book`, only when a file is on disk | **verified** |
| Grasp and pick up the target book | 3 | `check_confirm_hold` | **verified** |
| Return with the book | 3 | `check_confirm_delivery`, on the `/bin_contacts` event | **verified, intermittent** |
| Drop into the bin | 2 | `check_confirm_delivery` → `_classify_placement` | **verified, intermittent** |
| Gently place into the bin | 4 | `_classify_placement`, and only from a recorded release height low enough to have been a set-down | **not earned** — see below |

### What a delivering run is actually worth

A run that reaches the completion event by the route above earns the seven
stage awards (1 + 2 + 3 + 1 + 2 + 3 + 3 = 15) and the 2-point drop, a **gross
of 17** against the 19 available, less 0.5 for every unintended collision.
The 2 points that separate 17 from 19 are the difference between a drop and a
gentle placement, and they are not being claimed. The recorded take took one
counted collision, which puts its own estimate at **16.5** — see below.

Each stage is awarded once, from the observation that justifies it, and the
evidence string is written into the trial record (`ScoreCard.summary`). Nothing
is awarded for arriving somewhere: the 3 navigation points ride on the grasp
and the 3 return points ride on the bin's own contact sensor.

### The placement is a drop, and is scored as one

`_classify_placement` calls a placement gentle only when the book was lowered
to within `BOOK_SPINE_H / 2 + 0.05` = 0.175 m of the bin floor (0.740 m), and
only when a release height is actually known. On the direct carry the height is
now known, and it is nowhere near low enough:

* `release_z` is recorded on the direct carry too. `_direct_carry` writes it as
  the height the gripper is already holding the book at, less whatever the
  torso takes out on the way; before that only `manipulation_node.build_place`
  recorded one, and the direct carry wrote its placement reason from a release
  height of zero, which read as *"released at z=0.000 m, −0.740 m above the bin
  floor"*. `_classify_placement` now reports an unrecorded height as
  unrecorded — `the release height was not recorded, so the placement cannot be
  shown to have been a gentle one` — and takes the lower award, because a
  set-down it cannot prove is not one it should claim.
* The geometry is what makes it a drop. The torso aims to bring the gripper
  down to `CARRY_RELEASE_TOOL_Z` = 1.45 m but may lower at most 0.15 m on the
  way (`CARRY_MAX_TORSO_DROP`), so a high lift releases from higher still. On
  the recorded take the gripper held the book at z = 1.671 m and the torso
  spent its whole 0.15 m, so the release height `_direct_carry` records works
  out at 1.521 m — 0.781 m above the bin floor, against the 0.175 m the gentle
  branch asks for. Once the book has swung under the pinch it hangs about
  0.28 m below the gripper, which puts its centre near 1.24 m: roughly 0.29 m
  above the 0.950 m rim.

This is deliberate, not an oversight. The planned placement is the only way to
earn the 4 points, and it costs 25–40 s of return, lining up and planning
against a grip measured to keep a hanging book for 6–11 s. Chasing the extra 2
points puts at risk the 5 the drop route earns — 3 for the return and 2 for the
drop — on a path that has never yet delivered.

### Rubric mechanics and collisions

| Requirement | Implementation | Verification | Status |
|---|---|---|---|
| Rubric implemented exactly (1+2, 3, 1+2, 3, 3, 2 **or** 4) | `scoring.STAGE_POINTS` / `PLACEMENT_POINTS` | `test_scoring.py` checks the 19 / 17 maxima | **verified** |
| −0.5 per unintended collision, continuous contact counted once, 1 s cooldown, floor at zero | `scoring.CollisionMonitor` | Five dedicated tests | **verified** |
| Intentional grasp contact not penalised, and only for the gripper | `CollisionMonitor.is_intentional` exempts gripper links touching a book while a manipulation state is active; an **arm** link striking a book counts wherever it happens | `test_scoring.py::test_grasping_the_target_book_is_not_a_collision`, `::test_an_arm_link_striking_a_book_is_never_intentional`, `::test_the_gripper_touching_a_book_outside_manipulation_still_counts` | **verified** |
| Estimates flagged as judge-dependent | Every summary carries a disclaimer; placement classification records its rule | `test_scoring.py::test_summary_is_serialisable_and_explains_itself` | **verified** |

Collisions are the one term that can move without any stage changing. Every
contact the monitor counts is −0.5, so two contacts cost more than the gap
between a drop and a gentle placement. Across the five re-run trials in
`results/` four runs are collision-free; the fifth is the base reversing over
the book it had just dropped (`base_link` against `book_col_4_row_4_red`
during `RETREAT`, −0.5, taking that run from 6.0 to 5.5), which cannot happen
on a run that keeps hold of it. The delivering take counted exactly one:

```
collision #1: tiago_pro::arm_right_6_link::arm_right_6_link_collision
hit book_col_3_row_2_red during RETREAT (-0.5)
```

An arm link against the target book, which `is_intentional` never excuses, and
the same stage the one collision in `results/` happened in: both counted
contacts on record are made backing away, and `grasp.book_offender` models the
way in. By the score card's own arithmetic that is −0.5 on a gross of 17, a net
of **16.5**.

The score card itself is not available for that take, and neither is a trial
record. `tools/record_trial.sh` deliberately ends the take on the organisers'
`/bin_contacts` event — the book in the bin is the task — while the score card
and the machine-readable record are written by `mission_node._finish`, several
states later at `DONE`. So the 16.5 above is assembled here from the run's own
log lines rather than read off a summary the run printed. That log is kept
beside the video as `*_solution.log`; neither is in this repository.

### What the estimate is not

It is a local estimate, printed and recorded so a judge can audit or overrule
it. Gentle-versus-drop and collision intent are judge calls; the rules used
here are stated in `scoring.py` and above so they can be checked rather than
trusted.

## 6. Deliverables

| Item | Status |
|---|---|
| `docs/measured_environment.md` | **verified** |
| `docs/competition_readiness.md` (this file) | **verified** |
| `docs/grasp_blocker.md` | **verified** |
| Offline test suite (162 tests, `python3 -m pytest src/rafeeq_solution/test -q`, no ROS and no Gazebo needed, pyflakes clean) | **verified** |
| Documented trials with machine-readable records | **5 recorded** in `results/` — five layout seeds, five columns, all four colours. Column and row identified, published and photographed on all five. All five **predate** the above-centre grasp and the direct carry, and none of them delivers: their recorded estimates are 6.0, 6.0, 6.0, 6.0 and 5.5, the identification points alone. They are evidence for §2 and the identification half of §3, not for §5. |
| Root `README.md` rewrite | **verified** |
| `report/Rafeeq_ERC2026_Phase1_Report.pdf` | **verified** — 6 pages (title + 5 body), 0.17 MB, every figure read from `results/` at build time. Predates the delivering run. |
| `docs/odometry_drift.md` | **verified** |
| `docs/assets/rafeeq_logo.jpeg` (README and report title page) | **verified** |
| Demonstration video | **recorded** — 4:34 (274 s), 1920x1080 at 15 fps, 11.7 MiB. One continuous unedited take, no speed-up, team title card first, both launch files typed on camera, live trial timer, ending on the official completion event with the timer frozen at 221.3 s on `BOOK DELIVERED`. Produced by `tools/record_trial.sh`, which keeps the trial's opening frame (`*_first_frame.png`) and the solution and simulator logs beside it. Not committed to this repository. |
| Trial records regenerated against the delivering solution | not yet done — and the delivering take writes none of its own: the recorder ends it on the completion event, before `_finish` reaches the record (§5) |
| `v1.0.0` release against the validated commit | not yet prepared |
| Fresh-clone reproduction test | not yet run |

## Known gaps and risks

1. **The carry is timing-critical, and not every run will deliver.** The pinch
   is held by mimic joints whose URDF effort limit is 0.1 N m, there is no
   attach or detachable-joint mechanism anywhere in the organisers' world or
   robot, and a pinched book creeps out of the pads: measured, a freely
   hanging book stays held 6–11 s. The direct carry is built to finish inside
   that — the drive starts while the retention check is still running, the
   torso lowers on the move, and the arm never moves — and on the recorded run
   it did: the drive ran from t+99.3 s to the release at t+107.1 s, 7.8 s of a
   6–11 s grip, with the bin's own sensor reporting the book at t+107.8 s.
   The margin is seconds, not tens of seconds:
   the navigation node's own notes record carries that finished "within a
   second or two of the book letting go". A run that loses the book on the way
   fails honestly (`the book was dropped on the way to the bin`) and keeps the
   6 identification points plus the 6 for the grasp and shelf navigation.
2. **The success rate is not measured.** There is one recorded delivery. The
   five machine-readable records in `results/` all predate the above-centre
   grasp and the direct carry, so nothing in this repository states how often
   the current solution delivers. Re-running the five-seed set and recording
   the outcome is the next piece of evidence needed, and until it exists no
   claim about repeatability should be made.
3. **The placement scores 2, not 4.** By construction — see §5. The 4-point
   gentle placement would need the planned placement path, which the grip is
   not measured to survive.
4. **The fallback return has never delivered.** If the bin is not identified
   from the start zone, or the lift leaves the gripper below
   `CARRY_MIN_TOOL_Z` = 1.28 m (so the hanging book would not clear the
   0.950 m rim), the run takes the long way: drive to the start zone, search
   for the bin, line up, plan a placement. That path is implemented and
   covered offline (`test_mission_flow.py::test_a_book_held_too_low_for_the_rim_takes_the_long_way`,
   `::test_a_bin_out_of_reach_from_the_table_edge_takes_the_long_way`), and
   it is expected to drop the book: it takes 25–40 s against a 6–11 s grip. It
   is kept because failing to find the bin early is not a reason to abandon
   the grasp, not because it is likely to score.
5. **Collisions are the term most likely to move.** Two counted contacts cost
   more than the drop-versus-gentle gap. Four of the five trials in `results/`
   are collision-free and the fifth is the base reversing over a book that run
   had already dropped; the delivering take took one of its own, an arm link
   against the target book during `RETREAT`, −0.5 (§5). Both counted contacts
   on record were made backing away from the shelf, and the clearance model
   sweeps the approach, not the withdrawal — so this is not a term that has
   been engineered down yet, only measured.
6. ~~**A left-arm collision at start-up.**~~ ~~**The working arm striking the
   target book.**~~ **Both fixed.** The tucked left arm swept within 0.084 m
   of the table and clipped it on all five trials; `tucked_left` is now the
   right arm's tuck mirrored, taking the worst approach to 0.425 m. The
   working arm then turned out to pass the target book with 0.065 m to
   spare — less than its own link radius — because the planner ranked
   candidates on shelf clearance alone; the book is now in the clearance model
   and the chosen plan clears it by 0.179 m. That fix covers the way *in*: on
   the delivering take `arm_right_6_link` still touched the target book on the
   way out, during `RETREAT`, which the advance sweep does not model. See
   gap 5.
7. ~~**Book identification is layout-dependent.**~~ **Resolved.** It was worse
   than layout-dependence: the filter was picking a same-colour book from a
   neighbouring column, confidently and repeatably, because it compared
   measurements against positions the robot only believed. Now 5/5 across five
   seeds, with the measured offset from the column centre averaging 0.145 m
   against an organiser jitter of ±0.25 m — a neighbouring column would be
   about 1 m out. Ground-truth spot check on the worst previous case:
   −0.254 m recorded against −0.246 m true. See
   [`odometry_drift.md`](odometry_drift.md).
8. ~~**The grasp does not retain the book.**~~ **Resolved**, and rewritten in
   [`grasp_blocker.md`](grasp_blocker.md): the deprojection frame, the
   below-centre pinch, the jerked carry, the retreat over the board edge and
   the saturating fingertip hinges, in the order they were found.
9. **Wheel odometry drifts far enough to matter** — 0.29 m and 0.18 rad over a
   single traverse, measured against ground truth. Several checks were made
   robot-relative because of it (see `odometry_drift.md`), and the release
   pose is bounded to the bin's real interior rather than trusted
   (`CARRY_BIN_MARGIN` 0.08 m, `CARRY_BIN_MARGIN_Y` 0.09 m), but the coarse
   world waypoints still accumulate error unchecked. The intended correction —
   re-measuring the bin while driving to it — cannot fire, because perception
   refuses to deproject while the base is moving.
