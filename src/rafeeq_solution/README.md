# rafeeq_solution

Rafeeq Library Robot — Emirates Robotics Competition 2026, Phase 1.

Autonomous book retrieval with the TIAGo Pro: read the shelf column numbers,
drive to the right column, find the book of the requested colour, measure where
it actually is, pick it, and put it in the red collection bin.

The top-level [`README.md`](../../README.md) covers what has been demonstrated
and how to run a trial. This page is about the package.

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
| `arm_side` | `right` | Which arm grasps. The other is tucked. |
| `image_dir` | `<workspace>/erc_images` | Where annotated images go |
| `results_dir` | `<workspace>/results` | Where the trial record is written |
| `row_one_is_top` | `true` | Whether shelf row 1 is the topmost row |
| `auto_start` | `true` | Start immediately, or wait for `/rafeeq/mission/start` |
| `use_sim_time` | `true` | Follow the Gazebo clock. Leave true. |
| `run_id` | generated | Identifier for the trial record |

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

Delivery is read from the organisers' `/bin_contacts`, and collisions from
their `/contacts`. Neither is inferred from our own actions: the gripper
opening is not a delivery.

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
| `perception_node` | Number plates, book colour + row, book 3D position, red bin, annotated images, `/erc/*` topics |
| `navigation_node` | Holonomic waypoint driving on odometry, with separate speed and tolerance profiles per leg |
| `manipulation_node` | Arm / gripper / torso / head, every motion verified against feedback |
| `mission_node` | The state machine that sequences all of it — 33 states, none of which advances on a timeout |

Supporting modules, all ROS-free and unit-tested: `arena.py` (measured
constants and frame maths), `kinematics.py` (URDF-parsed damped least-squares
IK), `grasp.py` (measured pose → validated grasp plan), `vision.py` (OpenCV
detectors), `image_utils.py` (message conversion, pinhole deprojection),
`scoring.py` (practice scoring, collisions, delivery).

## Two deliberate departures from the brief

**No Nav2.** The image installs it, but `simulation.launch.py` never brings it
up — no map server, no AMCL, no costmaps, no `map` frame. The arena's furniture
is static, so the navigation node drives odometry waypoints directly. The base
is a mecanum holonomic drive, so it strafes along the shelf without turning
away from it. What *is* randomised per run — which plate hangs above which
column, which row holds each colour, and each book's position by ±0.25 m — is
handled by measuring, not by mapping.

**No MoveIt.** The repository ships no MoveIt configuration for TIAGo Pro — no
SRDF, no planning group, no kinematics plugin. `kinematics.py` parses the
competition URDF and solves the 8-DOF chain (torso + 7 arm joints) with damped
least squares, seeded from a sampled workspace; `grasp.py` turns a measured
book position into a validated posture. Its forward kinematics was validated
against the running simulator across three arbitrary arm configurations and
matched to **0.07 mm**.

## Grasp and place poses are solved, not stored

`config/arm_poses.yaml` holds only what is genuinely fixed: the two drive tucks,
the torso and head poses used for looking around, and the gripper constants.

There are no `reach_row_N` poses in it any more, deliberately. The organiser
jitters every book by up to ±0.25 m in world *y*, which is three times the
gripper's 0.082 m maximum opening, so a fixed pose aimed at a column centre
misses almost every book however well it was solved. Grasp and placement poses
are solved at run time from the measured target. A full plan takes well under a
second once the seed workspace is warm, which happens during start-up.

`scripts/solve_arm_poses.py` remains for offline experiments against the URDF.
`ros2 run rafeeq_solution tune_arm` still jogs joints interactively, which is
useful for inspecting a posture — it is no longer a step in getting a run to
work.

The tucks are worth one note, because they were a real defect. The previous
`tucked` pose left the right gripper at base-frame *x* = +0.706 and the left arm
was never commanded at all, which put both grippers inside the shelf: measured,
12 852 `gripper_left_fingertip`/`erc_shelf` contacts in a single move. The
current tucks were found by searching the sampled workspace and verified by
forward kinematics; measured on the running simulator, the worst approach of
any left-arm link to the table went from 0.084 m to 0.425 m.

## Offline tests

Both suites run on a laptop with no ROS and no Gazebo. Run them before every
competition attempt.

```bash
python3 -m pytest test -q                # 162 checks — contracts, mission flow, scoring, lint
python3 scripts/test_vision_offline.py   # 23 checks — detectors
```

`test_vision_offline.py` renders synthetic scenes at the *true* camera scale
(640×360, fx≈337 px) using the real number-marker textures, and checks digit
reading at 2.0–3.5 m, row ranking, false-positive rejection and bin detection.

`test_mission_flow.py` stubs rclpy and drives the state machine through a full
nominal run, all 20 column/colour combinations, and the failure paths.
`test_contracts.py` pins the geometric decisions — clearance, column
membership, plate reading — against the live cases that once got them wrong.

These caught real bugs during development, including:

* At the 0.75 m grasp standoff the camera **cannot see all four shelf rows at
  once** — they span 1.24 m and the vertical FOV only covers 0.80 m there.
  Row-by-vertical-order would have silently mis-ranked. Hence
  `BOOK_SCAN_STANDOFF = 1.5 m`, plus a measured-height fallback.
* A perspective cluster straddling two columns has a plausible mean and the
  wrong contents, so membership is tested per book against the plate that was
  actually read, not per cluster.
* `tf2_geometry_msgs` was never imported, so every `Buffer.transform()` would
  have thrown at runtime.

## Status

The full task has been completed on the live simulator and recorded: see
[the top-level README](../../README.md) and
[`docs/grasp_blocker.md`](../../docs/grasp_blocker.md), which traces every cause
that used to stop the grasp and what was measured to fix it.

What still needs care rather than celebration:

* **The grip is a clock.** The book is held by friction between two pads on
  mimic joints with a 0.1 N m URDF effort limit, and there is no attach or
  detachable-joint mechanism anywhere in the organisers' world or robot. Once
  the book hangs free it stays held for a measured **6–11 s**. The whole return
  is built to finish inside that: the bin is identified before the shelf trip,
  the drive starts while the hold check runs, and the book is dropped over the
  bin rather than placed. Runs can still drop it, and when they do the mission
  fails explicitly.
* **What breaks the grip is acceleration, not speed.** Measured by distance
  survived: 0.30 m/s² lost the book after 0.92 m, 0.15 m/s² after 1.75 m. Legs
  carried with a book ramp at 0.08 m/s² and 0.12 rad/s², in the odom frame.
* **A drop is not a gentle placement.** The rubric pays more for a gentle
  placement, and the current delivery is a release above the bin floor.
* **IK solves reachability, not collision.** The advisory model in `grasp.py`
  ranks candidates on link-origin clearance against the shelf boards and the
  target book; it is not a swept-volume check.
* **Do not read a log banner as a grasp.** Grasp success is decided by
  sustained fingertip↔book contact on `/contacts`, re-checked after the lift
  and after the base retreat, and delivery by `/bin_contacts`. If you are
  debugging by hand, verify with `gz model -m <book> -p`.

## Running it on Windows

ROS 2 is not on Windows — it is inside the container, inside WSL2. Typing
`ros2 launch ...` into PowerShell always fails. Use the wrapper at the repo
root instead:

```powershell
.\erc.ps1 sim              # Gazebo with GUI
.\erc.ps1 sim headless     # no GUI, faster
.\erc.ps1 solution 2 red
.\erc.ps1 status
.\erc.ps1 logs sol
.\erc.ps1 reset            # kill everything, keep the workspace build
```

Two traps it exists to avoid: always pass **both** compose files (the GPU
overlay is what injects the NVIDIA runtime), and **never** `docker compose
down` — `install/` and `build/` live inside the container, so `down` destroys
the whole workspace build.
