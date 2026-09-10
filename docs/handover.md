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
| Offline tests | **109 pytest + 23 vision**, all passing, no ROS or Gazebo needed |
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
| Book's measured offset from its column centre | mean **0.157 m**, max **0.266 m** (organiser jitter is ±0.25 m) |
| Reached the shelf and closed on the book | 5/5 attempted |
| Book retained | **0/5** |
| Delivered | **0/5** |
| Mean estimated score | **5.5 / 19** |

Two of those figures are worth reading twice. Identification used to be 1 in 3;
it is now 5 in 5, and the offsets say the *right* book is being found rather
than merely a book — a book from a neighbouring column would be about 1 m out,
not 0.16 m. Spot-checked against Gazebo ground truth on the case that used to
fail worst (seed 404): the recorded offset was −0.266 m against a true −0.246 m.

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
| Save the logo to `docs/assets/rafeeq_logo.png` (referenced by README and report title page) | you |
| Decide whether to submit with the grasp unresolved, or raise it with the organisers first | you |
| Record the demonstration video — impossible until a run succeeds | you |
| Approve any push, tag or release | you |
| Grasp retention | open — blocker |
| The tucked left arm brushes the table while the base turns to face the shelf: −0.5 on **every** trial, 5 for 5, always the same contact | open — cheap to fix, worth 2.5 points across five runs |
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
python3 -m pytest src/rafeeq_solution/test -q          # 109 offline tests
python3 src/rafeeq_solution/scripts/test_vision_offline.py
python3 report/build_report.py --preview               # PDF + page PNGs
```

The report reads every figure from `results/*.json` at build time, so it cannot
drift from the runs. Delete the records and it says so on the page rather than
printing stale numbers.
