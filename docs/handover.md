# Handover

Commit `2d9c99e` on `main`, on top of the unmodified organiser baseline
`2aa5a4a` (`v1.0.3-1-g2aa5a4a`). Nothing has been pushed.

## What changed

The previous version reported success it had not achieved. Sixteen mission
states treated a timeout as completion — including the final one, which logged
`MISSION COMPLETE — book delivered to the collection bin` on runs where nothing
had happened. The measured book position was published and consumed by nobody.
Both are fixed, along with the geometry that made the grasp impossible.

Every non-obvious number was measured from the running simulator, not assumed.
The measurements are in [`measured_environment.md`](measured_environment.md)
and reproducible with [`tools/diagnostics/`](../tools/diagnostics/).

## What was actually tested, and how

| | Result |
|---|---|
| Offline tests | 70 pytest + 23 vision, all passing, no ROS or Gazebo needed |
| Live trials | **3 recorded** with machine-readable records in [`results/`](../results/); 2 more ran but exceeded the harness cap before a terminal state and wrote nothing |
| Judges' launch commands | Run unmodified for every trial |
| Fresh clone | **Not completed.** `git clone` and `git archive` of this repo from the Windows filesystem into WSL both stalled — the tree vendors ~5 GB of PAL and organiser packages. What *was* verified directly against the commit: every text blob is LF, `tools/erc.sh` and the `docker/*.sh` scripts carry the executable bit, and all required files are present. The offline tests pass against the synced copy of the committed content, but **`colcon build` from a genuinely fresh `git clone` has not been run.** |
| Real-time factor | 0.362 measured — sim seconds are ~2.8× wall seconds here |

Measured rates over the 3 recorded trials:

| Stage | Rate |
|---|---|
| Target column identified, published, evidence image | **3/3** |
| Book row + measured 3D position + evidence image | **1/3** |
| Reached the shelf and closed on the book | 1/1 attempted |
| Book retained | **0** |
| Delivered | **0** |
| Mean estimated score | **1.5 / 19** |

## The two things that do not work

**1. Grasp retention.** The gripper reaches the correct book, closes on it, and
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

**2. Book identification, 1 of 3.** The column is found reliably; finding the
target *colour* among the books visible from the scan pose is not. Two of the
three failures were the target colour simply not being detected in the frame.
This is a solvable perception problem and is the second priority.

## Things worth knowing before touching this

* **Odometry drifts far more than you would expect.** Measured against ground
  truth: 0.29 m and 0.18 rad after one traverse. Several checks are deliberately
  robot-relative because of it — see [`odometry_drift.md`](odometry_drift.md).
  If you add a check against an absolute world coordinate, it will fail
  intermittently and the failure will look like a perception bug.
* **Two simulators on one `ROS_DOMAIN_ID` serve the same topics from different
  worlds.** Several hours were lost to this before it was spotted. `tools/`
  scripts now refuse to launch if a previous instance survived.
* **Camera frames are stamped ~1.7 s ahead of `/tf`** on this bridge. The
  perception node uses the latest transform only once the base has been still
  long enough for it to have caught up, and drops frames otherwise.
* **A stall is not evidence of a grasp here** — see above. Grasp verification
  reads the contact sensors.

## Outstanding, and who owns it

| Item | Owner |
|---|---|
| Confirm the Phase 1 deadline **and timezone** — the RIT page says 1 Aug 2026, the v1.0 PDF says 15 Sep 2026, and today is 9 Sep | you |
| Save the logo to `docs/assets/rafeeq_logo.png` (referenced by README and report title page) | you |
| Decide whether to submit with the grasp unresolved, or raise it with the organisers first | you |
| Record the demonstration video — impossible until a run succeeds | you |
| Approve any push, tag or release | you |
| Grasp retention | open |
| Book detection reliability | open |
| Two more trial records to reach five | open |

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
python3 -m pytest src/rafeeq_solution/test -q          # 70 offline tests
python3 src/rafeeq_solution/scripts/test_vision_offline.py
python3 report/build_report.py --preview               # PDF + page PNGs
```

The report reads every figure from `results/*.json` at build time, so it cannot
drift from the runs. Delete the records and it says so on the page rather than
printing stale numbers.
