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
| Book column membership decided by which cluster sat nearest the image centre | Decided against the **marker plate re-read from the same pose**, seconds before the books - see below. |

The grasp targeting was already immune: `perception` converts camera → odom →
world and `grasp` converts world → base using the base's odometry-derived
world pose, so the two conversions cancel exactly.

## What has not been addressed

Absolute world waypoints are still used for the coarse moves (face the shelf,
return to the start zone, approach the bin). They are tolerant of tenths of a
metre, but they are the remaining place where drift accumulates unchecked. The
principled fixes, in increasing order of effort:

1. ~~Re-observe a landmark before each stage that needs metric accuracy.~~
   **Done for the book scan** — the plate above the target column is re-read
   from the book-scan pose and the books are judged against it. The section
   below has the measurements. The same trick would suit the bin approach and
   has not been applied there.
2. Fuse the base IMU (`/base_imu`, already bridged) for heading, which is where
   most of the error is.
3. Stand up AMCL against a map of the static arena. Nav2 is installed but the
   organiser launch never brings it up, so this is the largest change.

## The failure this finally caused, and the landmark fix

Everything above says the same thing: a measurement may only be compared with
something that shares its drift. Book selection broke that rule in a way that
took a ground-truth check to see.

Running seed 101, column 2, colour red, the solution reported:

```
BOOK CONFIRMED: red book in row 2 ... measured at world (+3.004, +0.467, 1.254),
+0.467 m off the column centre
```

It reported it three times running, from three separate looks, agreeing to
0.02 m. Ground truth says the target column's red book is `book_col_3_row_2_red`
at y = **−0.154**, in row 1. What was being reported was `book_col_2_row_3_red`
at y = **+0.969** — the red book of the *next column over*. The robot strafed to
it and closed the gripper on empty air.

The filter picked whichever cluster of books lay nearest the middle of the
frame. That is the target column only if the robot is aimed at it; with 0.18 rad
of heading error it frequently is not. Repeatability was no defence, because
drift is a bias and not noise — three looks from the same wrong pose agree
beautifully.

### What replaced it

The marker plate is re-read from the book-scan pose itself, in a dedicated
`COLUMN_REF` mission state, immediately before the books are looked at. Plate
and books are then measured from one stationary base seconds apart, so they
carry the *identical* pose error, and the target column is simply whichever
cluster of books sits nearest that plate. The drift cancels in the subtraction.

Measured on the run that had been failing:

| | Measured y | Ground truth |
|---|---|---|
| Plate above the target column | −0.428 | 0.000 |
| Target book | −0.599 | −0.154 |
| **Book relative to its plate** | **−0.171** | **−0.154** |

Both absolute figures are 0.43 m wrong. The difference — the only quantity the
filter uses — is good to 0.017 m.

### The bug that had been hiding it

The re-read did not work at first: the robot read its neighbours' plates and
never its own. Standing in front of a column makes the plate above it the
steepest in the arena, 0.76 m up at 1.5 m range, so its digit is foreshortened
vertically. Measured from the book-scan pose, the digit `2` came out 56 × 49 px
— an aspect of **1.14**. It matched its own template at **0.827** and was then
discarded by an aspect filter that stopped at 1.05, on the reasoning that the
digits are all taller than wide. True read square on; false read from below.
The band is now 0.25–1.40, with the template score left to do the real work.


## Reproducing it

```bash
ERC_SEED=777 ros2 launch erc_bringup simulation.launch.py headless:=true
# drive the robot to a column, then:
python3 tools/diagnostics/odometry_vs_truth.py
```

Ground truth is read from Gazebo for the comparison only. The shipped solution
never reads it.
