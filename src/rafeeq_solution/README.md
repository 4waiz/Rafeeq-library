# rafeeq_solution

Rafeeq Library Robot — Emirates Robotics Competition 2026, Phase 1.

Autonomous book retrieval with the TIAGo Pro: read the shelf column numbers,
drive to the right column, find the book of the requested colour, pick it,
carry it back to the start zone, and place it in the red collection bin.

## Running it

Two terminals inside the competition container.

```bash
# Terminal 1 — the simulation (unmodified competition package)
ros2 launch erc_bringup simulation.launch.py

# Terminal 2 — our solution
ros2 launch rafeeq_solution solution.launch.py shelf_column_number:=2 book_colour:=red
```

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `shelf_column_number` | *required* | Shelf column number to retrieve from, 1–5 |
| `book_colour` | *required* | `red`, `green`, `blue` or `yellow` |
| `arm_side` | `right` | Which arm grasps |
| `image_dir` | `/opt/erc_ws/src/erc_images` | Where annotated images go |
| `row_one_is_top` | `true` | Whether shelf row 1 is the topmost row |
| `auto_start` | `true` | Start immediately, or wait for `/rafeeq/mission/start` |

## Competition outputs

| Topic | Type | When |
|---|---|---|
| `/erc/shelf_column_identification` | `std_msgs/Int32` | Once the target number plate is read |
| `/erc/shelf_row_identification` | `std_msgs/Int32` | Once the target book's row is resolved |

Both are latched (`TRANSIENT_LOCAL`) and re-published every 2 s, so a scorer
that subscribes late still receives the value.

Annotated images are written to `image_dir` with a millisecond timestamp
(`shelf_column_*.png`, `target_book_*.png`, `red_bin_*.png`). The rules ask
for `/erc_images/`; that path is not volume-mounted, so the node writes into
the mounted workspace and symlinks `/erc_images` at it. Both paths work, and
the files survive the container.

## Architecture

Four nodes, no direct coupling — everything goes through the mission node.

```
                  ┌──────────────┐
                  │ mission_node │  state machine, owns the run
                  └──────┬───────┘
        mode / goal / command │ status
        ┌───────────────┬─────┴─────────┬────────────────┐
        ▼               ▼               ▼                │
┌────────────────┐ ┌──────────────┐ ┌──────────────────┐ │
│ perception_node│ │navigation_node│ │manipulation_node│ │
└───────┬────────┘ └──────┬───────┘ └────────┬─────────┘ │
        │                 │                  │           │
   RGB-D camera        /cmd_vel        joint_trajectory  │
   /erc/* outputs      /odom, /scan     controllers      │
        └─────────────────┴──────────────────┴───────────┘
```

| Node | Responsibility |
|---|---|
| `perception_node` | Number plates, book colour + row, red bin, annotated images, `/erc/*` topics |
| `navigation_node` | Holonomic waypoint driving on odometry |
| `manipulation_node` | Named arm / gripper / torso / head poses |
| `mission_node` | The state machine that sequences all of it |

Supporting modules: `arena.py` (constants and frame maths), `vision.py`
(pure OpenCV detectors, no ROS), `image_utils.py` (message conversion).

## Two deliberate departures from the brief

**No Nav2.** The image installs it, but `simulation.launch.py` never brings it
up — no map server, no AMCL, no costmaps, no `map` frame. The arena is also
completely static: walls, shelf, table and bin never move, and the only thing
randomised per run is which number plate hangs above which column. So the
navigation node drives odometry waypoints directly. The base is a mecanum
holonomic drive, so it strafes along the shelf without turning away from it.

**No MoveIt.** The repository ships no MoveIt configuration for TIAGo Pro —
no SRDF, no planning group, no kinematics plugin. MoveIt is installed but has
nothing to plan against. Since the books are a fixed size at four fixed
heights, manipulation replays named joint poses instead.

Both are worth revisiting for the report's future-work section. Neither is
worth blocking a working submission on.

## Arm poses must be tuned once

`config/arm_poses.yaml` ships **starting guesses**, not verified solutions.
They are all inside the URDF joint limits so they are safe to command, but no
one has confirmed they put the gripper on a book — and without an IK solver
they cannot be derived offline.

Tune them once in the simulator:

```bash
ros2 run rafeeq_solution tune_arm
```

Jog joints (`2 -0.1`), set them (`s 4 -0.55`), move the torso (`t 0.25`),
head (`h 0 0.3`) and gripper (`g 0.06`). Press `p` to print a YAML block to
paste back into `config/arm_poses.yaml`. Budget twenty minutes for all four
rows; after that they are fixed, because the shelf geometry never changes.

**This is the one step between the current code and a completed run.**

## Offline tests

Both suites run on a laptop with no ROS and no Gazebo. Run them before every
competition attempt.

```bash
python3 scripts/test_vision_offline.py   # 23 checks — detectors
python3 scripts/test_mission_offline.py  # 53 checks — state machine
```

`test_vision_offline.py` renders synthetic scenes at the *true* camera scale
(640×360, fx≈337 px) using the real number-marker textures, and checks digit
reading at 2.0–3.5 m, row ranking, false-positive rejection and bin detection.

`test_mission_offline.py` stubs rclpy and drives the state machine through a
full nominal run, all 20 column/row combinations, and the failure paths.

These caught two real bugs during development:

* At the 0.75 m grasp standoff the camera **cannot see all four shelf rows at
  once** — they span 1.24 m and the vertical FOV only covers 0.80 m there.
  Row-by-vertical-order would have silently mis-ranked. Hence
  `BOOK_SCAN_STANDOFF = 1.5 m`, plus a measured-height fallback.
* `tf2_geometry_msgs` was never imported, so every `Buffer.transform()` would
  have thrown at runtime.

## Status against the live simulation (2026-08-26)

Verified in Gazebo, not just offline.

| Stage | Status |
|---|---|
| `simulation.launch.py` comes up, 7 controllers active | ✅ |
| Shelf column detection | ✅ correct on 4 different random layouts |
| Book colour + row detection | ✅ correct on 4 different random layouts |
| Depth / TF deprojection | ✅ book height measured to ~5 mm |
| `/erc/*` topics published, 3 annotated images saved | ✅ |
| Navigation to column and back to start zone | ✅ |
| State machine runs all 24 states to `DONE` | ✅ |
| **Grasp** | ❌ **the book does not move** |
| Bin localisation | ❌ off by 0.85 m |

Column/row detection was cross-checked against Gazebo's own spawn list
(`gz model --list`) on four independent randomised layouts and matched every
time, including the `row_2`..`row_5` (SDF) to 1–4 (competition) offset. That
settles the row convention: **`row_one_is_top=true` is correct.**

## Known gaps

* **The grasp does not hold yet — this is the one remaining blocker.**
  The arm now genuinely reaches the book: `config/arm_poses.yaml` holds real
  IK solutions (see below), and an isolated grasp test moves the book by a few
  millimetres, where the old hand-guessed poses moved it by exactly zero. So
  the gripper arrives and makes contact, but the book is nudged rather than
  picked up, at every gripper close value from 0.000 to 0.021.

  Two findings that explain it, both worth acting on:

  1. **The gripper is underactuated.** `gripper_right_finger_joint` drives a
     screw link; the fingertips hang off `gripper_right_inner_finger_*` and
     `gripper_right_fingertip_*` revolute joints that the URDF never couples
     to the commanded joint. So commanding a finger position does not
     directly set the fingertip separation — the tips are moved by contact
     physics. Tuning the `closed` value alone will not fix this.
  2. **There is no approach trajectory.** The manipulation node sends a single
     joint-space waypoint, so the arm interpolates straight from `tucked` to
     the reach pose. It does not approach the shelf along a straight line, so
     the fingers can swing in across the book instead of closing around it.

  The fix is a two-stage motion, which the IK module can now express: solve a
  pre-grasp pose ~0.10 m in front of the book, move there, then solve the
  grasp pose and move in along the approach axis before closing.

  Do not read `MISSION COMPLETE` as a successful grasp. Each manipulation step
  reports done when its trajectory duration elapses — deliberately, since a
  gripper stalling on a book *is* a successful grasp — so a failed grasp looks
  identical in the log. **Always verify with `gz model -m <book> -p`.**

## Arm poses are now solved, not guessed

`rafeeq_solution/kinematics.py` parses the competition URDF and does damped
least-squares IK against it, because TIAGo Pro ships no MoveIt config here.
Its forward kinematics was validated against the running simulator across
three arbitrary arm configurations and matched to **0.07 mm**.

Regenerate the poses with `python3 scripts/solve_arm_poses.py`. Residuals:

| Pose | Position error | Finger tilt |
|---|---|---|
| `reach_row_1` | 0.41 mm | 4.0° |
| `reach_row_2` | 0.99 mm | 4.4° |
| `reach_row_3` | 0.61 mm | 0.1° |
| `reach_row_4` | 0.39 mm | 0.0° |
| `place` | 0.19 mm | 0.0° |

Two things mattered for getting these:

* **Orientation must be fully constrained, not just the approach axis.** The
  fingertips separate along the grasp frame's local *y*. A book spine is
  0.03 m wide but 0.25 m tall and the gripper opens 0.069 m, so a gripper
  rolled 90° cannot close on it at all — even though the IK reports success.
* **Seeding matters more than the solver.** Seeded from uniform noise, DLS
  declared the top shelf row unreachable and 144 mm short. Monte-Carlo
  workspace sampling showed grasp-valid poses up to 1.86 m, well above the
  1.595 m top row. Re-seeding from sampled configurations with roughly the
  right orientation found it to 0.41 mm. The workspace was never the problem.

**IK solves reachability, not collision.** Nothing checks the arm against the
shelf, the robot's own body, or neighbouring books.
* **Bin localisation is poor.** The live run reported the bin at world
  `(-1.07, +0.85, 0.82)` against a true `(-1.0, 0.0, 1.3)`. Cause: the
  `scan_bin` head pose was tilted at the floor, so only a clipped sliver of the
  bin was in frame at the right edge. Head tilt has been corrected from -0.25
  to -0.05 rad; not yet re-verified.
* **`/erc/shelf_column_identification` semantics.** We publish the detected
  number — which equals the requested one — only once the plate is actually
  read, so it evidences detection rather than echoing the input. The physical
  slot is published separately on `/rafeeq/perception/column_slot`.

## Running it on Windows

ROS 2 is not on Windows — it is inside the container, inside WSL2. Typing
`ros2 launch ...` into PowerShell always fails. Use the wrapper at the repo
root instead:

```powershell
.\erc.ps1 sim              # Gazebo with GUI
.\erc.ps1 sim headless     # no GUI, roughly 2x faster
.\erc.ps1 solution 2 red
.\erc.ps1 status
.\erc.ps1 logs sol
.\erc.ps1 reset            # kill everything, keep the workspace build
```

Two traps it exists to avoid: always pass **both** compose files (the GPU
overlay is what injects the NVIDIA runtime), and **never** `docker compose
down` — `install/` and `build/` live inside the container, so `down` destroys
the whole workspace build.
