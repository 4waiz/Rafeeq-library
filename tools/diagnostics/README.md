# Diagnostics

Throwaway-quality scripts that produced the measurements in `docs/`. They are
kept because every non-obvious number in this repository should be
reproducible, not because they are polished.

All of them assume a running simulation and a sourced workspace:

```bash
ERC_SEED=777 ros2 launch erc_bringup simulation.launch.py headless:=true
source install/setup.bash
python3 tools/diagnostics/<script>.py
```

| Script | What it measures | Backs |
|---|---|---|
| `odometry_vs_truth.py` | Robot pose from `/odom` against Gazebo ground truth | [`docs/odometry_drift.md`](../../docs/odometry_drift.md) |
| `deproj.py` | Deprojects every detected book and prints measured vs true world positions | `odometry_drift.md` |
| `depthkind.py` | Whether the depth image is planar z or Euclidean range | `measured_environment.md` |
| `grab.py` | Grabs a live frame and runs the book detector, showing which blobs pass each filter | perception tuning |
| `layout.py` | Ground-truth colour, row and position of the books in one column | trial analysis |
| `staged_grasp.py` | Drives the base and arm through the whole grasp sequence, logging contacts and the book's displacement | [`docs/grasp_blocker.md`](../../docs/grasp_blocker.md) |

## On ground truth

Several of these read object poses from Gazebo. That is deliberate and it is
confined to here: ground truth is used as the **oracle** a diagnostic is judged
against, and in `staged_grasp.py` also as the target, so that manipulation can
be exercised without depending on perception.

**The shipped solution never reads it.** `src/rafeeq_solution/` contains no
call to `gz model`, no subscription to a ground-truth pose topic, and no
hard-coded book position. What it knows about the arena is the static geometry
documented in `arena.py`; everything the organiser randomises is measured from
the camera.
