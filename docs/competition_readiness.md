# Competition readiness

Every Phase 1 requirement, what implements it, how it was checked, where the
evidence is, and an honest status.

**Status vocabulary**

| Status | Meaning |
|---|---|
| **verified** | Demonstrated on the live simulator, with evidence recorded |
| **implemented, unverified** | Code exists and passes offline tests, not yet proven live |
| **blocked** | Implemented, demonstrated *not* to work, cause characterised |
| **missing** | Not done |

Validation baseline for everything below: organiser `dfl-rlab/erc_sim_2026`
@ `2aa5a4a` (`v1.0.3-1-g2aa5a4a`), image `erc-2026:humble-harmonic`,
Ubuntu 22.04.5 / ROS 2 Humble / Gazebo Sim 8.15.0, measured real-time factor
0.362. See [`measured_environment.md`](measured_environment.md).

---

## 1. Launch interface

| Requirement | Implementation | Verification | Status |
|---|---|---|---|
| `ros2 launch erc_bringup simulation.launch.py` runs unmodified | No organiser file is modified — `git diff --name-only 2aa5a4a e5154d6` touches only `src/rafeeq_solution/`, `tools/`, and new top-level directories | Diff checked; simulation launched from the organiser file for every trial | **verified** |
| `ros2 launch rafeeq_solution solution.launch.py shelf_column_number:=N book_colour:=C` works with no other mandatory argument | [`solution.launch.py`](../src/rafeeq_solution/launch/solution.launch.py) — only those two are required; everything else defaults | Run as the judges would for every trial in `results/` | **verified** |
| No browser, no manual arm tuning, no separate start command | `auto_start` defaults true; the mission starts itself once readiness is *observed* | Trial logs show `WAIT_READY → HOME` with no operator input | **verified** |
| Inputs validated (column 1–5, colour red/blue/green/yellow) | `MissionNode._validate_inputs`, before anything is commanded | `test_mission_flow.py::test_invalid_*` and the 20-combination parametrised test | **verified** |
| One arm only | `arm_side` parameter selects one; the other is tucked, never used to grasp | `arm_poses.yaml` `tucked`/`tucked_left`; trial logs | **verified** |
| Declared, installable dependencies | `package.xml` lists `control_msgs`, `action_msgs`, `ros_gz_interfaces`, `python3-opencv`, `python3-numpy`, `python3-yaml`, … | Built with `colcon build` in the stock organiser image | **verified** |

## 2. Perception evidence

| Requirement | Implementation | Verification | Status |
|---|---|---|---|
| `/erc/shelf_column_identification` (`Int32`) published **only after** the marker is identified | `perception_node._do_column` publishes after `confirm_frames` consistent deprojected votes | Trial logs: `COLUMN CONFIRMED: number 3 is above physical column 3` precedes publication | **verified** |
| `/erc/shelf_row_identification` (`Int32`) after the book is identified | `perception_node._do_book` | Trial logs: `BOOK CONFIRMED: red book in row 2` | **verified** |
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
| Trajectories respect real joint limits | Time-parameterised from the URDF's per-joint `velocity` limits (`kinematics.joint_velocity_limits`) | Torso at 0.035 m/s no longer under-runs its trajectory | **verified** |
| Measured book position drives the grasp | `perception_node` publishes the measured world point; `mission_node` passes it to `manipulation_node`; `grasp.plan_grasp` plans from it | Trial log: *"measured at world (+2.861, −0.171, 1.252), −0.171 m off the column centre"*, and the base strafes to it | **verified** |
| Column filter cannot select a same-colour book from another column | The marker plate is re-read from the book-scan pose itself (`COLUMN_REF`), and a book belongs to this column when its own measured `y` is within the jitter band of that plate. Plate and books share the pose error, so it cancels. | Live: plate at `-0.428` where truth is `0.000`, book at `-0.599`; the difference `-0.171` against a true `-0.154`. `test_contracts.py::test_the_neighbouring_columns_book_is_not_chosen` and four more | **verified** |
| Membership tested per book, not per cluster | Perspective can merge two columns into one image cluster; such a cluster has a plausible mean and the wrong contents | `test_contracts.py::test_a_cluster_straddling_two_columns_is_split_by_the_plate`, from the seed 404 failure | **verified** |
| The plate above the target column can be read at all | It is the steepest in the arena, so its digit is foreshortened; the aspect band runs to 1.40 and the template score does the real work | `test_contracts.py::test_a_foreshortened_plate_digit_is_still_read` — the live digit was 56x49 px, aspect 1.14, matching at 0.827 | **verified** |
| Transforms taken at image time | `world_point` uses `header.stamp`; falls back to the latest transform **only while the base is stationary**, and drops the frame while moving | On this bridge images are stamped ~1.7 s ahead of `/tf`; behaviour logged once at info level | **verified** |
| RGB↔depth 15 mm baseline corrected | `world_point` shifts the depth lookup by `fx·baseline/range` | Measured frame offset (0.015, 0, 0) | implemented, unverified |
| Grasp verified from real observation | Sustained fingertip↔book contact on `/contacts`, re-checked after the lift and after a 0.42 m base retreat | Live: contact registers and `holding=book_col_3_row_3_red` is reported | **verified** (detection) |
| Book acquired, retained, transported | Same | Contact does not survive the lift or retreat | **blocked** — see [`grasp_blocker.md`](grasp_blocker.md) |
| Bin visually identified, not substituted with a nominal pose | `_do_bin` requires size, shape, solidity **and** a deprojected position inside the bin's real bounds; failure fails the state | `test_mission_flow.py::test_bin_detection_failure_does_not_fall_back_to_a_nominal_pose` | implemented, unverified (never reached in a live run) |
| Delivery verified on `/bin_contacts`, not inferred | `scoring.DeliveryMonitor` ignores the permanent bin↔table contact and fires only on a *book* | `test_scoring.py::test_bin_resting_on_the_table_is_not_a_delivery` | implemented, unverified |
| Bounded retries and explicit failure | `grasp_attempts` cap, `RECOVER` state, `FAILED` is terminal and reported | Live: 3 grasp attempts then `FAILED` | **verified** |
| Stale results cannot advance a new state | Sequence numbers on every manipulation reply; perception clears votes on mode change | `test_mission_flow.py::test_a_reply_to_an_older_command_is_ignored` | **verified** |

## 4. Navigation

| Requirement | Implementation | Verification | Status |
|---|---|---|---|
| Orients itself from the start condition, no extra argument | Spawn yaw is a known constant of the arena; the mission turns to face the shelf as its first move | Every trial | **verified** |
| Omni-directional obstacle checking | `ScanSector` bins both lidars into 36 sectors; the commanded direction is checked, not just "ahead" | Code + `blocked` transitions in live logs | implemented, partially verified |
| Stale odometry/scan stops the base | `data_fresh` / `scans_fresh` gate the controller | Not yet exercised live | implemented, unverified |
| `blocked` is not `reached` | `nav_gate` treats them separately and fails on a persistent block | `test_mission_flow.py::test_navigation_blocked_is_not_arrival` | **verified** |
| Planned vs executed path published for RViz | `/rafeeq/nav/planned_path` and `/rafeeq/nav/executed_path` (`nav_msgs/Path`), labelled as waypoints — not a Nav2 plan | Topics published; screenshots outstanding | implemented, unverified |
| Predictable shutdown; a held book is not dropped | `manipulation_node.shutdown` cancels goals and deliberately leaves the gripper closed | Code reviewed; not exercised live | implemented, unverified |

## 5. Scoring

| Requirement | Implementation | Verification | Status |
|---|---|---|---|
| Rubric implemented exactly (1+2, 3, 1+2, 3, 3, 2 **or** 4) | `scoring.STAGE_POINTS` / `PLACEMENT_POINTS` | `test_scoring.py` checks 19 / 17 maxima | **verified** |
| Navigation points ride on outcomes, not waypoints | Awarded in `CONFIRM_HOLD` and `CONFIRM_DELIVERY` | `test_scoring.py::test_navigation_points_are_not_awarded_without_a_grasp` | **verified** |
| −0.5 per unintended collision, continuous counted once, 1 s cooldown, floor at zero | `scoring.CollisionMonitor` | Five dedicated tests | **verified** |
| Intentional grasp contact not penalised | `CollisionMonitor.is_intentional` | `test_scoring.py::test_grasping_the_target_book_is_not_a_collision` | **verified** |
| Estimates flagged as judge-dependent | Every summary carries a disclaimer; placement classification records its rule | `test_scoring.py::test_summary_is_serialisable_and_explains_itself` | **verified** |

## 6. Deliverables

| Item | Status |
|---|---|
| `docs/measured_environment.md` | **verified** |
| `docs/competition_readiness.md` (this file) | **verified** |
| `docs/grasp_blocker.md` | **verified** |
| Offline test suite (70 pytest + 23 vision) | **verified** |
| Documented trials with machine-readable records | **3 recorded** in `results/`; 2 more were run but exceeded the harness's per-trial cap before reaching a terminal state and wrote no record. **Short of the 5 the brief asks for.** |
| Root `README.md` rewrite | **verified** |
| `report/Rafeeq_ERC2026_Phase1_Report.pdf` | **verified** — 5 body pages + title, 0.16 MB, every figure read from `results/` at build time |
| `docs/odometry_drift.md` | **verified** |
| Demonstration video | **missing** — requires a successful run; see `grasp_blocker.md` |
| `v1.0.0` release against the validated commit | not yet prepared |
| Fresh-clone reproduction test | not yet run |

## Known gaps and risks

1. **The grasp does not retain the book.** Everything downstream of it — the
   grasp points, both navigation awards, and the placement points — is
   therefore unearned. Characterised in [`grasp_blocker.md`](grasp_blocker.md).
2. **A left-arm collision at start-up.** The tucked left arm brushes the table
   while the base turns to face the shelf, costing −0.5. Reproducible; the tuck
   pose needs re-selecting with the table in the clearance model.
3. **The demonstration video cannot honestly be produced yet**, because there
   is no successful run to record.
4. Bin detection and delivery verification are implemented and unit-tested but
   have never executed live, because no run has got that far.
5. **Book identification is layout-dependent.** In 2 of the 3 recorded trials
   the target colour was not detected among the books visible from the scan
   pose, and the run ended before the arm was used. The measured trials give
   3/3 for column identification but only 1/3 for the book. This is the second
   priority after the grasp.
6. **Wheel odometry drifts far enough to matter** — 0.29 m and 0.18 rad over a
   single traverse, measured against ground truth. Several checks were made
   robot-relative because of it (see `odometry_drift.md`), but the coarse
   world waypoints still accumulate error unchecked.
