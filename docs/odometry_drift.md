# Measured finding: mecanum wheel odometry drifts enough to break metric perception

**Status: measured, reproducible, and the reason for several design decisions.**

## The measurement

After driving from the start zone to a shelf column — roughly 1.4 m forward and
1 m sideways — the robot's belief about where it is was compared against
Gazebo's ground truth for the `tiago_pro` model:

| | x (m) | y (m) | yaw (rad) |
|---|---|---|---|
| Ground truth | 1.1469 | 1.1602 | **0.1807** |
| Derived from `/odom` | 1.3810 | 0.9952 | 0.0000 |
| **Error** | **−0.234** | **+0.165** | **+0.181** |

That is 0.29 m of position error and **10.4 degrees of heading error** after a
single short traverse. It is not a bug in the bridge or in our frame
conversions: a mecanum drive slips laterally by design, and wheel odometry
cannot observe that slip.

## Why it matters more than it looks

Perception deprojects a pixel into the camera frame, transforms it to `odom`
via TF, and then to the world frame using the known spawn pose. The `odom`
frame *is* the drifting one, so every world-frame position inherits the error.
At book range that came to about **0.44 m laterally** — which is:

* larger than the organiser's ±0.25 m book jitter, so a book measured against
  the nominal column centre falls outside its own column's band and is
  rejected; and
* close to half the 1 m column pitch, so `nearest_column_slot` can snap a
  marker to the *wrong* column. Observed directly: the same layout seed
  identified "number 3 above physical column 2" on one run and "above physical
  column 4" on another.

The snapping is also what hid the problem for a long time — a marker measured
0.44 m out still snaps back to the right column most of the time, so column
identification looked reliable while the underlying measurement was not.

## What was changed because of it

The principle throughout: **compare things that share the same drifting frame,
so the error is common-mode and cancels.**

| Was | Now |
|---|---|
| Book column membership tested against `arena.COLUMN_Y[slot]` in world | Tested against the **robot's own y**. Both come from the same odometry. |
| Navigate to the nominal centre of the identified slot | Navigate to the **measured position of the plate that was read**. |
| `grasp.check_target` compared x and y against absolute arena constants | Compares the target's position **relative to the base** (range ahead, offset to the side). Height stays absolute — planar drift does not affect z, and measured heights match ground truth to a few millimetres. |
| Shelf-clearance geometry anchored to `arena.SHELF_FRONT_X` | Anchored to the **measured book**, offset by the known 0.065 m recess. |

The grasp targeting was already immune: `perception` converts camera → odom →
world and `grasp` converts world → base using the base's odometry-derived
world pose, so the two conversions cancel exactly.

## What has not been addressed

Absolute world waypoints are still used for the coarse moves (face the shelf,
return to the start zone, approach the bin). They are tolerant of tenths of a
metre, but they are the remaining place where drift accumulates unchecked. The
principled fixes, in increasing order of effort:

1. Re-observe a landmark and correct the pose estimate before each stage that
   needs metric accuracy — the shelf face and the number plates are both
   suitable and already detected.
2. Fuse the base IMU (`/base_imu`, already bridged) for heading, which is where
   most of the error is.
3. Stand up AMCL against a map of the static arena. Nav2 is installed but the
   organiser launch never brings it up, so this is the largest change.

## Reproducing it

```bash
ERC_SEED=777 ros2 launch erc_bringup simulation.launch.py headless:=true
# drive the robot to a column, then:
python3 tools/diagnostics/odometry_vs_truth.py
```

Ground truth is read from Gazebo for the comparison only. The shipped solution
never reads it.
