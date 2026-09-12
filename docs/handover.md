# Handover

`main`, 19 commits ahead of `origin/main` and **none of them pushed**.
`origin/main` is `e5154d6`, one commit on top of the unmodified organiser
baseline `2aa5a4a` (`v1.0.3-1-g2aa5a4a`). There are no tags in this repository.

This page is the runbook: how to build it, how to run it, how to record a
trial, what the state machine does now, where everything it writes ends up, and
what to do when the simulator misbehaves. The engineering write-ups live
alongside it — [`grasp_blocker.md`](grasp_blocker.md),
[`odometry_drift.md`](odometry_drift.md),
[`measured_environment.md`](measured_environment.md),
[`competition_readiness.md`](competition_readiness.md).

## Status

The robot completes the task. One recorded, unedited run delivered the book:

| | |
|---|---|
| Run | `ERC_SEED=101`, `shelf_column_number:=2`, `book_colour:=red` |
| Delivery | `OFFICIAL COMPLETION EVENT: book_col_3_row_2_red contacted the collection bin at t+107.8 s` |
| Confirmed by | the organisers' `/bin_contacts` topic, not by the gripper opening |
| Collisions | exactly one for the whole run — `arm_right_6_link` against `book_col_3_row_2_red` during `RETREAT`, scored at −0.5 |
| Video | 4:34, one continuous take, 1920×1080 at 15 fps, on-screen trial timer reading 221.3 s at the delivery and frozen on `BOOK DELIVERED` |

**The 107.8 s and the 221.3 s are two different clocks, and they are supposed
to disagree.** Every `t+` the mission logs is *simulated* seconds — the nodes
run on `use_sim_time` — while the on-screen timer is wall clock, started when
`solution.launch.py` is launched. 107.8 simulated seconds inside 221.3 wall
seconds is a realised real-time factor of 0.487 for this take. Quoting one
against the other is the easiest mistake to make about this run.

Stages of that run — the simulated second each stage was *entered*, off the
mission's own `[STATE]` lines: looking for the bin from the start zone
t+21.7 s; reading the number plate, 2 above physical slot 3, t+31.9 s; finding
the red book in row 1 t+48.9 s; closing on it t+82.9 s; lifting t+91.9 s;
retention confirmed and the carry away t+99.3 s; the gripper opening over the
bin t+107.1 s; official contact t+107.8 s.

The model named in the completion event is the organisers' SDF name, which
counts from the other end: physical slot 3 is the slot carrying the printed
plate `2`, and SDF `row_2` is competition row 1. That the two agree was checked
after the fact; nothing in the solution ever selects on a model name.

**It is not guaranteed, and the reason is timing.** The pinch holds the book
against gravity but has almost no stiffness about its own axis; once the book
hangs free it creeps out of the pads, and measured, it stays held for 6–11 s.
The carry is built to finish inside that window and usually does — on this take
it took 7.8 simulated seconds from `CARRY_TO_BIN` to `RELEASE`, which is inside
the window but not by much. Runs can still drop the book. Treat a failed take
as normal and record another.

Everything behind that sentence — why the grasp is taken above the book's
centre of mass, why the squeeze is deliberately light, why the fingertip
hinges' 0.1 N m effort limit governs the whole problem — is in
[`grasp_blocker.md`](grasp_blocker.md). Do not change a grasp or carry constant
without reading it first.

## Build and run

Prerequisites and the container are the organisers'; the full original
instructions are preserved in [`organiser_README.md`](organiser_README.md).

```bash
git clone https://github.com/4waiz/Rafeeq-library.git
cd Rafeeq-library

./docker/up.sh --build          # build the image, start the container
./docker/attach.sh              # shell inside it

# inside the container
cd /opt/erc_ws
colcon build --symlink-install
source install/setup.bash
```

Only `src/` is bind-mounted; `build/` and `install/` live inside the container.
`docker compose down` destroys them, which is why `tools/erc.sh` never calls
it. Use `./tools/erc.sh reset` (restarts the container, keeps `install/`) or
`./tools/erc.sh recreate` (destroys and rebuilds, about 20 s for all 33
packages) instead.

Then the two commands the judges run, in two terminals:

```bash
ros2 launch erc_bringup simulation.launch.py

ros2 launch rafeeq_solution solution.launch.py \
    shelf_column_number:=2 book_colour:=red
```

`shelf_column_number` is 1–5 and `book_colour` is one of `red`, `blue`,
`green`, `yellow`. Both are required; everything else has a working default,
and `solution.launch.py` deliberately does not start the simulator, so the two
can be launched separately on camera. The mission starts itself on observed
readiness — set `auto_start:=false` to arm it and trigger with
`/rafeeq/mission/start` instead. The full argument table is in the
[README](../README.md#launch-arguments).

`ERC_SEED` makes the arena layout repeatable — the same digit order, the same
colour-to-row assignment and the same book jitter:

```bash
ERC_SEED=101 ros2 launch erc_bringup simulation.launch.py
```

`tools/erc.sh` wraps all of this from the WSL host (`up`, `build`, `sim`,
`solution`, `shell`, `status`, `logs`, `stop`, `window`, `shot`). Its two
reasons for existing are that it always passes both compose files — the GPU
overlay is what injects the NVIDIA runtime — and that it never runs
`docker compose down`.

### Offline tests, before every attempt

```bash
python3 -m pytest src/rafeeq_solution/test -q                 # 162 tests
python3 src/rafeeq_solution/scripts/test_vision_offline.py    # 23 tests
```

No ROS, no Gazebo, no robot needed for either. Both pass on the current tree.

### Rendering

Gazebo renders on the GPU through Mesa's `d3d12` driver under WSL, which is
what `record_trial.sh` sets up (`GALLIUM_DRIVER=d3d12`,
`MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, `/usr/lib/wsl/lib` on
`LD_LIBRARY_PATH`). Real-time factor is about 0.65–0.70 headless and 0.56 with
the GUI up, against 0.36 in software. Only the rendering changes; the physics
and the solution are identical. `./tools/erc.sh status` measures the live
factor by timing `/clock` over 8 s of wall time.

Those are spot readings. Averaged end to end — viewer attached and `ffmpeg`
grabbing the display throughout — the delivering take realised 0.487, which is
what to budget against the five-minute limit rather than the spot figure.

## Recording a trial

```bash
# inside the simulation container
cp docs/assets/rafeeq_logo.png /tmp/rafeeq_logo.png
tools/record_trial.sh <column 1-5> <colour> [seed]
```

`tools/record_trial.sh` records a trial exactly as the Phase 1 document asks:
a team title card before anything is launched, `simulation.launch.py` and then
`solution.launch.py` typed a character at a time and launched on camera, a
live trial timer, one continuous take, no cuts and no speed-up. It runs on a
virtual X display of its own (`Xvfb` on `:99`), so the recording holds the
simulator and the two launch terminals and nothing else from the machine.
`ffmpeg` grabs that display for the whole take; nothing is composited or
trimmed afterwards — the final step only re-encodes the capture for upload,
keeping every frame at the same rate and timing.

**The world starts headless and the viewer is attached to it afterwards.**
`simulation.launch.py` is launched with `headless:=true`, and only once
`tiago_pro` is in `gz model --list` does the script start `gz sim -g` against
the running server. Launched together the two race: the viewer builds its scene
from whatever the server holds at that instant, and the robot is spawned a
moment later, so when the viewer loses that race the recording shows the arena
with no robot in it — the shelf, the table and the bin, and nothing between
them — for the whole run. One finished take was lost to exactly that. Asking
the server for a scene that already has the robot in it leaves no race to lose.

Not trusting that, the script then **proves** the robot is in the viewer's
scene before the trial starts. It asks the viewer to follow `tiago_pro` and
reads `/gui/currently_tracked`: a named follow target means the viewer has the
robot in its scene, an empty one means it has not. Nothing else answers that
question from outside the viewer. The camera is returned to the side-on
overview afterwards, which is why the check runs before the timer does. If the
scene comes back empty the viewer is killed and started again — a fresh viewer
rebuilds its scene from scratch, and the headless server never notices — and
re-checked. Failing twice, the take is abandoned with `the viewer still has no
robot in its scene` and the viewer's own output.

The viewer window is then placed at 1700 px wide against the 1280 px the
recording gives it, so Gazebo's world and entity-tree panels take the overflow
and sit under the terminal column, leaving the whole recorded 1280 px to the
view of the arena.

| Environment variable | Default | Meaning |
|---|---|---|
| `OUT` | `/tmp/rafeeq_trial.mp4` | Output file |
| `FPS` | `15` | Capture frame rate |
| `TRIAL_LIMIT` | `900` | Seconds before the take is abandoned |
| `LOGO` | `/tmp/rafeeq_logo.png` | Title-card logo, shown beside the team name if `feh` is installed |

What it needs that the image does not ship: `docker/Dockerfile` installs no
`ffmpeg`, `Xvfb`, `xterm`, `xdotool` or `feh`, and the script uses all five.
If a take dies immediately, install them in the container first.

The take ends when the solution log shows `OFFICIAL COMPLETION EVENT` or
`RUN … COMPLETE` or `RUN … FAILED`, then holds five seconds on the final frame
so the result and the frozen timer are readable. The on-screen timer starts
when `solution.launch.py` is launched and freezes on the delivery event — the
bin's own contact sensor reporting the book is the task being finished; what
the arm does afterwards is tidying up.

It prints the result line, the frozen timer value and the video path, and
copies the run's logs next to the video as `${OUT%.mp4}_solution.log` and
`${OUT%.mp4}_simulation.log`, along with the frame the trial started from as
`${OUT%.mp4}_first_frame.png` — so what the recording shows can be checked
without watching the recording. Those copies exist because the work directory
`/tmp/rafeeq_rec` is wiped at the start of the next take, and running takes
back to back once cost an attempt's cause of death before it could be read.

The delivering take's video and logs are not in this repository. The take is
`/tmp/rafeeq_take44.mp4` inside the simulation container, copied to the
Desktop as `Rafeeq_Phase1_TeamKanban_delivered.mp4` (274 s, 11.7 MiB). There is
**no trial record for it**: the recorder ends the take on the organisers'
`/bin_contacts` event — the book in the bin is the task being finished — so the
run never reached `DONE`, never printed `RUN … COMPLETE`, and never wrote its
JSON. Do not go looking for one.

## The mission states

`mission_node.py` has 33 named states plus `IDLE`. The single most important
property of the file is what it does not do: **nothing advances because a timer
expired.** A state that runs out of time either retries, bounded, or fails the
run, and `FAILED` is a real outcome reported as one. Earlier versions treated a
timeout as completion in sixteen places, including the last one, and logged
`MISSION COMPLETE — book delivered to the collection bin` on runs where nothing
had happened.

The current state is published latched on `/rafeeq/mission/state` and logged as
`[STATE] NAME  (t+NN.N s)`; the run's summary goes out on
`/rafeeq/mission/outcome`.

**Start and the early bin search**

| State | Waits on |
|---|---|
| `WAIT_READY` | The simulation clock going live, then `/rafeeq/manip/ready`. The trial clock is not latched until `/clock` is real — latching it before makes the first elapsed reading enormous and fires every timeout at once. Fails after `ready_timeout` (180 s). |
| `HOME` | The home posture. |
| `FACE_BIN` | Turning in place in the start zone to face the bin. |
| `BIN_POSE` → `DETECT_BIN` | Identifying the collection bin **visually, from the start zone, before the shelf trip**. Up to three retries at increasing head tilt. |

The early bin search is the change that made delivery possible. The carry home
is the one stage the grasp cannot be trusted with for long, and a return that
also stopped to find the bin dropped the book every time. Measuring the bin now
lets the return drive straight to the release pose. The bin is still identified
visually — only earlier. If it is not found from the start zone the run does
not fail: it warns, carries on to the shelf, and searches again the ordinary
way once the book is in hand.

**Shelf and grasp**

| State | Waits on |
|---|---|
| `FACE_SHELF` → `SCAN_POSE` | Turning to face world +x, then the marker-scan posture. |
| `DETECT_COLUMN` | The number plate. Publishes `/erc/shelf_column_identification` and claims the annotated image. Five head pan/tilt positions in all (`SWEEP`), then failure. |
| `NAV_COLUMN` | Driving to the **measured** plate position, at book-scan standoff. |
| `COLUMN_REF_POSE` → `COLUMN_REF` | Re-reading the plate from the book-scan pose. This is the drift-matched reference the book filter needs; losing it is a warning, not a failure. |
| `BOOK_POSE` → `DETECT_BOOK` | The target-colour book. Publishes `/erc/shelf_row_identification` and claims its image. Five head tilts in all (`BOOK_TILTS`), which sweep **both ways** from nominal. |
| `NAV_STAGE` | Parking square to the measured book at `ARM_TRANSIT_STANDOFF` (1.35 m). |
| `REFRESH_BOOK` | Measuring the book again from the staging pose, with only the final advance left. |
| `PLAN_GRASP` | A validated grasp posture. Two attempts, then failure. |
| `ARM_PREGRASP` | The arm taking that posture while still clear of the shelf. |
| `NAV_GRASP` | The **base** carrying the fixed posture in, precise leg: 0.30 m/s cap, 0.006 m tolerance. |
| `REFINE` → `CLOSE_GRASP` | The fingers stepping in until a pad reports contact, then squeezing. Three attempts; each failure goes to `RECOVER`. |
| `RECOVER` | Reopening the gripper, then backing clear, then back to `COLUMN_REF_POSE`. |
| `LIFT` | The lift, which ends with its own retention check. |
| `RETREAT` | Reversing `EXTRACT` (0.34 m) on a gentle, ramped goal. |

**Carry and delivery**

| State | Waits on |
|---|---|
| `CONFIRM_HOLD` | The retention check — **and it starts driving while that check runs**. The check takes about 1.9 s out of a 6–11 s budget. The gripper's position came back with the lift, so the release pose is already known; if the check then fails the run ends anyway. Awards the grasp and shelf-navigation points. |
| `CARRY_TO_BIN` | The direct carry. The base drives straight to the pose that puts the held book over the bin's mouth, the torso lowers on the way (at most 0.15 m), and the arm does not move, so nothing swings the book. Fails the run if the hold width drops to zero. |
| `NAV_BIN_AREA` → `BIN_POSE` → `DETECT_BIN` → `NAV_BIN` → `PLACE` | The long way round, taken only when the direct carry is not available. |
| `RELEASE` | Opening the gripper. |
| `CONFIRM_DELIVERY` | **The official contact on `/bin_contacts`, not the fact that we let go.** 20 s, then failure with `the gripper opened over the bin but nothing ever contacted it`. |
| `BACK_OFF` → `DONE` | Backing the base 0.35 m off the bin, then stowing. Neither step may turn a completed delivery into a failed run; a problem is logged instead. |

`_direct_carry` returns nothing — and the run takes the long way — when the bin
was never measured, when the gripper is below `CARRY_MIN_TOOL_Z` (1.28 m, so
the hanging book clears the bin's 0.95 m rim), or when the base cannot stand
anywhere that both clears the table and puts the gripper inside the bin's
walls.

A bin fix arriving mid-carry is **recorded and not acted on**, deliberately.
Re-issuing the goal resets the navigation ramp to standstill — 0.004 m/s while
the base is doing 0.45, a spike of about 9 m/s², which is the one thing that
certainly throws the book out of the pads.

### Motion profiles

What shakes a hanging book out of the pads is acceleration, not speed: measured
live, 0.30 m/s² dropped it after 0.92 m and 0.15 m/s² after 1.75 m. The carry
needs 2.3 m, so the acceleration is halved again and the top speed left high.

| Leg | Cap | Notes |
|---|---|---|
| Ordinary | 0.50 m/s, 0.90 rad/s | Nothing held; the run is judged on a clock. |
| Precise (grasp approach) | 0.30 m/s | Arrives to 0.006 m, 0.008 m across the heading. |
| Gentle (retreat, and any leg with the book) | 0.12 m/s, 0.25 rad/s, ramped at 0.15 m/s² and 0.25 rad/s² | |
| Smooth (the carry) | 0.45 m/s, 0.60 rad/s, ramped at 0.08 m/s² and 0.12 rad/s² | Arrives to 0.06 m and 0.04 rad. |

`smooth_max_angular` must **not** be made to track `max_angular`: raising the
ordinary turn rate to 0.90 rad/s also sped up the turn made with the book
hanging from the pads, and the book came out after 3.4 s instead of 8.8 s on the
run before it.

Arm and torso trajectories run at 0.80 of the URDF velocity limits.

## Where the evidence goes

| What | Where | Set by |
|---|---|---|
| Annotated identification images — `shelf_column_*.png`, `target_book_*.png`, `red_bin_*.png` | [`erc_images/`](../erc_images/) | `image_dir:=`, else `ERC_IMAGES_DIR`, else the workspace's `erc_images/` found by walking up |
| Machine-readable trial record, one JSON per run | [`results/`](../results/)`trial_<run_id>.json` | `results_dir:=`, `run_id:=` |
| Trial video, and that take's two logs and opening frame beside it | `$OUT`, `${OUT%.mp4}_solution.log`, `${OUT%.mp4}_simulation.log`, `${OUT%.mp4}_first_frame.png` | `record_trial.sh` |
| Live launch logs when driven from the host helper | `/tmp/sim.log`, `/tmp/sol.log` inside the container | `tools/erc.sh sim` / `solution`, read with `erc.sh logs sim\|sol` |
| Measurement scripts behind every non-obvious number | [`tools/diagnostics/`](../tools/diagnostics/) | each needs a running simulation and a sourced workspace |
| Phase 1 report | `report/Rafeeq_ERC2026_Phase1_Report.pdf` | `python3 report/build_report.py --preview` |

Inside the container `erc_images/` is `/opt/erc_ws/src/erc_images`, which is in
the mounted workspace, so images appear on the host and survive the container.
`/erc_images` is symlinked to it so the rulebook path also works. No attempt is
made to create that symlink from the solution: it needs write permission on
`/`, and a judge should not have to grant it.

**The trial records in `results/` are stale.** All five predate the working
grasp: every one is `outcome: failed`, `delivered: false`, four of them on
retention. The report reads every figure from `results/*.json` at build time,
so it cannot drift from the runs — which also means it will keep printing those
numbers until fresh records replace them. Delete the records and it says so on
the page rather than printing stale figures. Re-running the five seeds and
rebuilding the report is the outstanding job — and note that the delivering
take added nothing here, because it ends on the delivery event rather than at
`DONE`, and the record is only written when the run finishes.

On ground truth: several diagnostics read object poses from Gazebo, as the
oracle a measurement is judged against, and `staged_grasp.py` also as a target.
**The shipped solution never reads it** — `src/rafeeq_solution/` contains no
call to `gz model`, no ground-truth pose subscription and no hard-coded book
position.

## When the simulator misbehaves

Three failures happen intermittently on this machine and none of them is caused
by the solution. `record_trial.sh` checks for all three before the trial starts,
because any one of them wastes a whole take.

**The arena never spawns.** The script polls `gz model --list` for the 20
`book_col_*` models, up to 36 times at 5 s intervals, before it will start the
trial. If they are not all there it abandons the take with `the arena never
spawned (N of 20 books)` and prints the last 25 lines of the simulator's own
output. Launching the solution into a simulator that never came up is worse
than losing the take: with no `/clock` the mission cannot even time itself out,
so it sits in `WAIT_READY` until the script's own `TRIAL_LIMIT` expires — 27
minutes, once.

**The controller spawners lose their startup race.** The organisers' launch
races its own `controller_manager`, and measured here every spawner can die
with `Failed loading controller` while the manager is still loading them. The
world comes up, the books spawn, and the robot never moves; the solution then
waits out its whole 180 s readiness timeout for something that cannot arrive.
`/joint_states` only flows once `joint_state_broadcaster` is really running, so
that is what the take waits for — up to 40 attempts at 3 s intervals. Failing
that, it abandons the take with `the robot's controllers never came up` and
prints the spawner lines from the simulator log.

**The viewer comes up without the robot in its scene.** This one costs nothing
in the simulation and everything in the recording: the run is genuine, the
robot moves, the book is delivered, and the footage shows an empty arena. It
cost one finished delivering take, which is why the world is now started
headless and the viewer attached afterwards, and why the script asks
`/gui/currently_tracked` whether the viewer can actually see `tiago_pro` before
the timer starts. It restarts the viewer once and then abandons the take. The
robot is never restarted for this — the server is headless and does not know
the viewer went away — so the fix is the viewer, not the world.

All three checks `exit 1` before the timer starts, so no partial take is
produced. For the first two the fix is to stop the simulator and launch it
again.

Running by hand rather than through the recorder, the first two failures look
like: no `/clock`, or a mission parked in `WAIT_READY`. `./tools/erc.sh status`
answers both in one command — it reports whether the clock is advancing, the
last `[STATE]` line, and the most recent evidence images. The third is
invisible from the topics; only a look at the picture finds it, which is what
`${OUT%.mp4}_first_frame.png` is for.

## Things worth knowing before touching this

* **Two simulators on one `ROS_DOMAIN_ID` serve the same topics from different
  worlds.** Several hours were lost to this before it was spotted. The `tools/`
  scripts now kill any previous instance before launching.
* **Camera frames are stamped ahead of `/tf`** on this bridge. Perception only
  takes a transform once the base has been still long enough for it to have
  caught up, and drops frames otherwise. This is also why a bin fix cannot
  arrive during the carry — the base is moving.
* **A stall is not evidence of a grasp here.** Grasp verification reads the
  contact sensors, and delivery reads the organisers' `/bin_contacts` — which
  is filtered to bin-against-*book* contacts, because the bin rests on the
  table and that pair is published at 200 Hz from the moment the world loads.
* **Wall-clock and simulated time are not interchangeable.** A timeout written
  in wall seconds fires before a trajectory written in sim seconds can finish.
* **Never test a measurement against an absolute world coordinate.** Odometry
  drifts 0.29 m and 0.18 rad over a single traverse, which at book range is
  most of a column, and the system was confidently identifying a red book one
  column over. The fix is to compare only things measured from the same pose:
  the marker plate is re-read from the book-scan pose, so plate and books carry
  identical pose error and their difference carries none of it. Full write-up
  in [`odometry_drift.md`](odometry_drift.md). Any check you add against a
  nominal world position will fail intermittently and look like a perception
  bug.
* **The base standoff and the measured book position are coupled.** The
  standoff is measured from the book, so anything that moves the measurement
  moves the base with it — a 0.08 m deeper book once halved the base's
  clearance from the shelf, 0.174 m to 0.094 m, and drift ate the rest. If you
  change anything that moves the measured book, check the base clearance in the
  same breath.
* **Link origins clearing by a hair is not clearance.** Both collisions that
  used to happen on every run — the tucked left arm against the table, the
  wrist against the target book — passed a check that walked link origins. Both
  acceptance margins are now twice the link radius. Not all the way solved: the
  delivering take still recorded one, `arm_right_6_link` against the book it
  was carrying, on the way out of the shelf. It is an arm link, so the narrow
  gripper-links exemption in `scoring.CollisionMonitor.is_intentional` does not
  cover it and it scores against the run.

## Outstanding, and who owns it

| Item | Owner / state |
|---|---|
| Re-run the five seeds and regenerate `results/` and the report | open — the committed records predate the working grasp |
| Confirm the Phase 1 deadline **and timezone** — the RIT page says 1 Aug 2026, the v1.0 PDF says 15 Sep 2026 | you |
| Approve any push, tag or release — 19 commits are unpushed | you |
| Fresh-clone `colcon build` on a Linux host | open — never completed; cloning this tree out of the Windows filesystem into WSL stalls on the vendored packages |
| ~~Grasp retention~~ | **fixed** — see [`grasp_blocker.md`](grasp_blocker.md); the carry remains timing-critical |
| ~~Record the demonstration video~~ | **done** — one delivering take, 4:34, unedited, with the robot visibly in the arena |
| ~~The tucked left arm brushes the table~~ | **fixed** |
| ~~Save the logo~~ | **done** — `docs/assets/rafeeq_logo.png` |

## Release safety

- The submission repository has **no tags**. The organiser clone has inherited
  `v1.0.0`, `v1.0.1`, `v1.0.3` on organiser commits — **an inherited `v1.0.0`
  is not our release.**
- The only remote is `origin → github.com/4waiz/Rafeeq-library`. The organiser
  repository is not a remote, so there is nothing to push to it by accident.
- Full pre-release list in
  [`../SUBMISSION_CHECKLIST.md`](../SUBMISSION_CHECKLIST.md).
