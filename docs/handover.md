# Handover

`main`, on top of the unmodified organiser baseline `2aa5a4a`
(`v1.0.3-1-g2aa5a4a`). **Nothing has been pushed.**

## What changed

The version this started from reported success it had not achieved. Sixteen
mission states treated a timeout as completion — including the final one, which
logged `MISSION COMPLETE — book delivered to the collection bin` on runs where
nothing had happened. The measured book position was published and consumed by
nobody. Both are fixed, along with the geometry that made the grasp impossible
and the perception that was identifying the wrong book.

Every non-obvious number was measured from the running simulator, not assumed.
The measurements are in [`measured_environment.md`](measured_environment.md)
and reproducible with [`tools/diagnostics/`](../tools/diagnostics/).

## What was actually tested, and how

| | Result |
|---|---|
| Offline tests | **120 pytest + 23 vision**, all passing, no ROS or Gazebo needed |
| Static analysis | pyflakes clean, enforced by `test_lint.py` |
| Live trials | **5 recorded**, machine-readable, in [`results/`](../results/) |
| Judges' launch commands | Run unmodified for every trial |
| Fresh clone | **Not completed.** Cloning this tree out of the Windows filesystem into WSL stalls on the ~5 GB of vendored packages. Verified directly against the commit instead: every text blob is LF, the shell scripts carry the executable bit, all required files present, offline tests pass against the committed content. A real `colcon build` from a fresh clone still needs running on a Linux host. |
| Real-time factor | 0.362 measured — sim seconds are ~2.8× wall seconds here |

Measured over the five recorded trials — five different layout seeds, five
different columns, all four colours:

| Stage | Rate |
|---|---|
| Target column identified, published, evidence image | **5/5** |
| Book row identified, published, evidence image | **5/5** |
| Book's measured offset from its column centre | mean **0.151 m**, max **0.250 m** (organiser jitter is ±0.25 m) |
| Reached the shelf and closed on the book | 5/5 attempted |
| Book retained | **0/5** |
| Delivered | **0/5** |
| Mean estimated score | **5.9 / 19**; four of the five trials are collision-free at 6.0 |

Two of those figures are worth reading twice. Identification used to be 1 in 3;
it is now 5 in 5, and the offsets say the *right* book is being found rather
than merely a book — a book from a neighbouring column would be about 1 m out,
not 0.16 m. Spot-checked against Gazebo ground truth on the case that used to
fail worst (seed 404): the recorded offset was −0.250 m against a true −0.246 m.

## The one thing that does not work

**Grasp retention.** The gripper reaches the correct book, closes on it, and
the simulator reports sustained fingertip contact — then the book is not
retained through a 0.020 m lift or a 0.42 m base retreat. Two properties of the
provided environment look responsible, both documented with evidence in
[`grasp_blocker.md`](grasp_blocker.md):

* the book's collision declares `<max_contacts>4</max_contacts>`, so one
  fingertip can consume the whole budget and the opposing finger gets no
  contact constraint and therefore no opposing force;
* `gz_ros2_control` realises a position command as a `JointVelocityCmd`, so the
  finger is driven kinematically and cannot apply a clamping force. Commanded
  and achieved positions agree to five decimals with or without a book between
  the pads, and reported effort is a constant −0.0935 N.

Increasing the squeeze from 6 mm to 16 mm of interference changed nothing.
**This is the first thing to raise with the organisers**, with the reproducer.

Downstream of it, the five trials failed in two ways, both consequences:
three closed on the book and came up empty; two lost the book to the shelf
after a failed attempt disturbed it, and honestly reported that they could no
longer see it rather than driving at a position it had left.

## How the wrong-book problem was solved, because it will come back

Odometry drifts 0.29 m and 0.18 rad over a single traverse. Any test of a
measurement against a position the robot *believes* — a nominal column centre,
the middle of the frame — inherits that error, and at book range it is most of
a column. The system was confidently and repeatably identifying a red book one
column over.

The fix is to compare only things measured from the same pose. The marker plate
is re-read from the book-scan pose itself, so plate and books carry the
identical pose error and the difference between them carries none of it. A book
belongs to this column when its own measured `y` is within the jitter band of
that plate. Full write-up, with the measurements, in
[`odometry_drift.md`](odometry_drift.md).

If you add any check against an absolute world coordinate, it will fail
intermittently and the failure will look like a perception bug.

## The collision that happened on every run

Both arms are tucked before the base drives, because an arm left forward sits
inside the shelf when the robot parks to grasp. The right arm's tuck was fine.
The left arm had a tuck of its own, found by the same workspace search, and it
put the wrist out at a 0.516 m radius hanging at world z = 0.668 — below the
0.740 m table top. The robot spawns at world (0, 0) yawed 90 degrees and turns
in place to face the shelf, and that turn swept the wrist to within 0.084 m of
the table, less than the link's own radius. Result: `arm_left_6_link` against
`erc_table`, −0.5, on all five recorded trials, and the only collision in the
set.

The fix needed no search. In the URDF, arm joints 2 to 7 are geometrically
identical on both arms; only joint 1 is mounted mirrored, with exactly negated
limits. So the right arm's tuck — which has never collided — transfers to the
left arm by negating joint 1 and keeping the rest. Measured afterwards on the
running simulator, with link origins agreeing with forward kinematics to the
millimetre:

| | Before | After |
|---|---|---|
| Worst approach to the table over the turn | 0.084 m | **0.425 m** |
| Largest radius of any wrist link from the base | 0.516 m | 0.238 m |
| Lowest link, world z | 0.668 m | 0.710 m |

What limits it at 0.425 m is `arm_left_2_link`, the shoulder, which does not
move. No contacts of any kind are reported while the pose is held, and a live
run goes `HOME → FACE_SHELF → SCAN_POSE` with a collision count of zero.

Reproduce the measurement with `tools/diagnostics/tuck_clearance.py`.

Across the five trials re-run afterwards, that contact does not occur once,
against five for five before.

Fixing it exposed the next one, which is written up below: the working arm
striking the target book on its way in. With both fixed, four of the five
trials are collision-free and one contact remains in the whole set — the base
reversing over a book the gripper had already dropped. That one exists only
because the grasp drops the book, so it goes when the grasp does. Collisions
across the three batches: **5 → 4 → 1**.

## The collision that came after it

Two trials reported `arm_right_6_link` striking the **target book** during
`NAV_GRASP` — the wrist knocking the book over before the fingers arrived — and
one of them also caught the shelf.

The arm is solved once, for the moment the fingers close, and then held rigid
while the base drives in, so the whole approach is that one posture swept along
the shelf normal. The wrist sits **0.228 m deeper into the shelf than the
grasping point**, so it passes the book on the way past. The planner never
looked: it ranked candidates on shelf penetration alone, took the first tilt
that scored a clean zero, and that plan cleared the book by **0.065 m** — less
than the link's own 0.05 m radius, before any error in a measured book
position. Exactly the failure mode as the tuck: link origins clearing by a
hair while the physical envelope collides.

Enumerating every approach tilt and wrist roll for the same book showed a
strictly better option was always available:

| Tilt | Roll | Shelf intrusion | Clearance from the book |
|---|---|---|---|
| +90 (what it chose) | −y | 0.000 | **0.065 m** |
| +70 | +y | 0.000 | **0.179 m** |
| +45 | +y | 0.000 | 0.158 m |
| −45 | −y | 0.000 | 0.148 m |

The book is now part of the clearance model. `GraspPlanner.book_offender`
sweeps the whole base advance and reports the deepest intrusion into the book's
box by any link that is not the gripper — the pads, fingers and the wrist
frames they hang off are exempt, because they have to touch it — and acceptance
takes the worse of the shelf and book figures instead of the shelf alone. The
margin is twice the link radius: the first radius pays for walking link origins
rather than meshes, the second for the measured book position being a few
centimetres out.

It only bites when it needs to. On books where the first tilt already clears by
more than the margin it is accepted unchanged, reporting `book intrusion
0.000 m`; on the geometry that collided it moves the plan to the +70 tilt.
Re-run over the five trials, neither the arm/book nor the arm/shelf contact
occurs again.

## Things worth knowing before touching this

* **Two simulators on one `ROS_DOMAIN_ID` serve the same topics from different
  worlds.** Several hours were lost to this before it was spotted. `tools/`
  scripts now refuse to launch if a previous instance survived.
* **Camera frames are stamped ~1.7 s ahead of `/tf`** on this bridge. The
  perception node uses the latest transform only once the base has been still
  long enough for it to have caught up, and drops frames otherwise.
* **A stall is not evidence of a grasp here** — see above. Grasp verification
  reads the contact sensors.
* **Wall-clock and simulated time are not interchangeable** at RTF 0.362. A
  timeout written in wall seconds fires before a trajectory written in sim
  seconds can finish.

## Outstanding, and who owns it

| Item | Owner |
|---|---|
| Confirm the Phase 1 deadline **and timezone** — the RIT page says 1 Aug 2026, the v1.0 PDF says 15 Sep 2026 | you |
| ~~Save the logo~~ | **done** — `docs/assets/rafeeq_logo.png` leads the README and the report title page |
| Decide whether to submit with the grasp unresolved, or raise it with the organisers first | you |
| Record the demonstration video — impossible until a run succeeds | you |
| Approve any push, tag or release | you |
| Grasp retention | open — blocker |
| ~~The tucked left arm brushes the table~~ | **fixed** — see below |
| Fresh-clone `colcon build` on a Linux host | open |

## Release safety

- The submission repo has **no tags**. The organiser clone has inherited
  `v1.0.0`, `v1.0.1`, `v1.0.3` on organiser commits — **an inherited `v1.0.0`
  is not our release.**
- The only remote here is `origin → github.com/4waiz/Rafeeq-library`. The
  organiser repository is not a remote, so there is nothing to push to it by
  accident.
- Full pre-release list in [`../SUBMISSION_CHECKLIST.md`](../SUBMISSION_CHECKLIST.md).

## Rebuilding anything

```bash
python3 -m pytest src/rafeeq_solution/test -q          # 120 offline tests
python3 src/rafeeq_solution/scripts/test_vision_offline.py
python3 report/build_report.py --preview               # PDF + page PNGs
```

The report reads every figure from `results/*.json` at build time, so it cannot
drift from the runs. Delete the records and it says so on the page rather than
printing stale numbers.
