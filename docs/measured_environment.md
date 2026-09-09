# Measured environment reference

Every number here was **measured from the running simulator** on the organiser
baseline recorded below, not copied from documentation. Re-measure with the
scripts in `tools/` if the organiser baseline changes.

## Validation baseline

| Item | Value |
|---|---|
| Organiser repo | `https://github.com/dfl-rlab/erc_sim_2026` |
| Organiser commit | `2aa5a4a` ("Update topic names") |
| Relative to tags | `v1.0.3-1-g2aa5a4a` (one commit after `v1.0.3`) |
| Vendored in this repo at | commit `2aa5a4a`, unmodified (verified by `git diff --name-only`) |
| Container image | `erc-2026:humble-harmonic` (from `docker/Dockerfile`) |
| OS / ROS / Gazebo | Ubuntu 22.04.5, ROS 2 Humble, Gazebo Sim 8.15.0 (Harmonic) |
| Host | Windows 11 + WSL2 (Ubuntu 22.04) + Docker Engine |
| Layout seeding | `ERC_SEED` env var seeds `random` in `simulation.launch.py` |

## What the organiser actually randomises

Read directly from `src/erc_bringup/launch/simulation.launch.py`:

| Randomised | Mechanism | Range |
|---|---|---|
| Number plate ↔ column | `random.shuffle(shelf_number_markers)` (L286) | any permutation of 1–5 |
| Book colour ↔ row | `random.shuffle(colours_this_column)` per column (L243) | each column holds all 4 colours, one per row |
| Book horizontal position | `random.uniform(-0.25, +0.25)` added to column centre (L250) | **±0.25 m in world y** |

`COLUMN_JITTER_RANGE = COLUMN_WIDTH * 0.25 = 0.25 m`.

Number **markers are not jittered** — they sit exactly on the column centres
(measured: y = +2.000, +1.000, 0.000, −1.000, −2.000).

Books are **not** at column centres. Measured example (seed 12345):

```
book_col_1_*  y = +2.1776, +2.1601, +2.1514, +2.2159   (nominal +2.0)
book_col_3_*  y = +0.2290, −0.2044, +0.2393, −0.0725   (nominal  0.0)
book_col_5_*  y = −2.2366, −1.9046, −1.9947, −2.0423   (nominal −2.0)
```

## Shelf and book geometry

| Quantity | Measured | Note |
|---|---|---|
| Book front face | world x = 2.900 | all books |
| Book size | 0.25 × 0.03 × 0.16 m | spawned pitch +90°, so spine faces the robot |
| Spine presented | 0.03 m wide × 0.25 m tall | the graspable dimension is **0.03 m** |
| Book friction | `mu = mu2 = 5.0` | organiser v1.0.3 (`f3da578`) |
| Book centre z, top→bottom | 1.5769, 1.2469, 0.9169, 0.5869 | settled poses, seed-independent |
| Organiser model naming | `book_col_C_row_R_colour`, R ∈ 2..5 | physical shelf row index |
| Competition row (ours) | 1..4 top-down over occupied rows | competition row N ↔ model row N+1 |

## Collection bin and table

Computed from the STL meshes with the measured world pose.

| Quantity | Value |
|---|---|
| Bin spawn pose (world SDF) | (−1.0, 0.0, **1.3**) — spawn only |
| Bin settled pose (measured) | (−1.0, 0.0, **0.845**) — it drops onto the table |
| Bin world AABB | x ∈ [−1.255, −0.695], y ∈ [−0.155, +0.155], z ∈ [0.740, **0.950**] |
| **Bin rim height** | **z = 0.950 m** |
| Bin interior floor | z = 0.740 m (the table top) |
| Table world AABB | x ∈ [−1.400, −0.600], y ∈ [−0.700, +0.700], z ∈ [0.010, 0.740] |

The previous `arena.BIN_XYZ = (-1.0, 0.0, 1.3)` used the **spawn** pose. It was
0.455 m too high, which is why the solved `place` pose sat far above the bin.

## Gripper

Measured by commanding the finger joint and reading the fingertip link TF.

| Commanded joint (m) | Measured joint | Fingertip gap (m) |
|---|---|---|
| 0.000 | 0.00000 | **0.0280** |
| 0.020 | 0.02000 | 0.0444 |
| 0.035 | 0.03500 | 0.0565 |
| 0.050 | 0.05000 | 0.0682 |
| 0.069 | 0.06900 | 0.0820 |

Linear fit: **`gap(q) ≈ 0.0280 + 0.783 · q`**, so `q(gap) ≈ (gap − 0.0280) / 0.783`.

Consequences:

* A 0.03 m book needs `q ≤ 0.0026` for the fingers to touch it at all.
* The previous config used `closed: 0.021` → a **0.0444 m gap around a 0.030 m
  book**. The fingers never made contact. This was the grasp failure root cause.
* Commanding `q = 0.0` gives 0.028 m — a 2 mm interference on a 30 mm spine,
  which is the intended friction grip.
* Because the finger stalls on the book, the **measured** resting position is a
  direct width measurement: `width ≈ 0.028 + 0.783 · q_measured`. This is what
  `manipulation_node` now uses to verify a grasp.

Command limits (authoritative): `gripper_command_clamp.py` **rejects** (does not
clip) any point outside **[0.000, 0.069]**; the URDF joint limit is
[−0.001, 0.070]. The root `README.md` example showing `0.04 = open` predates
organiser `v1.0.3` (`c09cf5e`, "Add range limits to grippers") and is stale —
see `docs/organiser_baseline.md`.

The gripper exposes `position`, `velocity` **and `effort`** state interfaces.
Measured effort is a constant −0.0935 when unloaded at any position, so effort
alone is not a usable grasp signal here; **finger stall position is**.

## Robot frames

| Transform | Value |
|---|---|
| `base_footprint` → `base_link` | z = **+0.0762 m** |
| `base_link` → `head_front_camera_color_optical_frame` | (0.0702, 0.0325, 1.0833) at torso = 0 |
| `base_link` → `base_front_laser_link` | (+0.2751, −0.1830, +0.1324) |
| `base_link` → `base_rear_laser_link` | (−0.2751, +0.1830, +0.1324) |
| Colour → depth optical frame | (0.015, 0, 0) — a 15 mm baseline, no rotation |

Both cameras are 640×360 with effectively identical intrinsics
(`fx` 337.2096252 colour vs 337.2096401 depth), so a colour pixel indexes the
depth image directly; the 15 mm baseline is corrected in `image_utils`.

## Arm reachability (right arm, 8-DOF chain incl. torso)

The gripper's finger axis must lie along world ±y to close across the 0.03 m
spine. **Both roll signs are physically equivalent for a symmetric two-finger
grasp but are different IK targets**, and this matters:

| Row (book z) | fingers +y | fingers −y |
|---|---|---|
| 1 (1.5769) | **unreachable** | 0.0002 m |
| 2 (1.2469) | 0.0010 m | 0.0004 m |
| 3 (0.9169) | 0.0004 m | 0.0002 m |
| 4 (0.5869) | 0.0002 m | 0.0004 m |

The old `reach_row_1` pose was solved with the +y roll, which is why the top row
never worked. The grasp planner now tries both rolls and takes the reachable one.

Maximum grasp height with a valid book-grasp orientation, from 120 000 sampled
configurations: **world z ≈ 1.80 m** (right arm). All four rows are within reach.

## Official interfaces used

| Topic | Type | Use |
|---|---|---|
| `/bin_contacts` | `ros_gz_interfaces/msg/Contacts` | official delivery/completion observation |
| `/contacts` | `ros_gz_interfaces/msg/Contacts` | per-link robot contacts, used for collision scoring |
| `/scan_front_raw`, `/scan_rear_raw` | `sensor_msgs/msg/LaserScan` | two 270° lidars, together 360° |
| `/gripper_right_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | **public** gripper topic (organiser clamp owns it) |
| `/{arm_right,torso,head}_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | action interfaces, used for real completion feedback |

`gripper_*_controller_raw/follow_joint_trajectory` exists but is deliberately
**not** used: driving it would bypass the organiser's clamp.
