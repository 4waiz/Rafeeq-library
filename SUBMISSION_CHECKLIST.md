# Submission checklist — ERC 2026 Phase 1

**Team Kanban · Al Ain University · Mentor: Yazan Alahmed**

Items are marked complete only when they have been verified. Anything that
depends on a human action says so explicitly.

Last verified 12 September 2026, against `HEAD` (`6020dce`) **plus six
uncommitted code files** in the working tree — see "State of play", item 2.

---

## State of play — read first

The robot now completes the task. One recorded, unedited run delivered the
book: seed `ERC_SEED=101`, `shelf_column_number:=2`, `book_colour:=red`, and
the organisers' own `/bin_contacts` topic reported
`book_col_3_row_2_red contacted the collection bin at t+107.8 s`. The model
name was read back afterwards as a consistency check and never used to choose
anything: physical slot 3 carries the printed label "2", and SDF `row_2` is
competition row 1.

Two clocks describe that run and they are not the same clock. The mission's
`t+` figures are **simulated** seconds (`use_sim_time`) — the clock a trial
record writes as `timing.simulated_seconds_total`. The video's timer is **wall**
clock, started at `solution.launch.py`. It read 221.3 s at the delivery, and the
take is 4:34 (274 s) long. 107.8 simulated seconds inside 221.3 wall seconds is
a realised real-time factor of 0.487 for this take; quote the two figures
together or not at all.

That is one run, not a guarantee. The grip is friction against a pad hinge held
by 0.1 N m — the joint's URDF effort limit — the book stays held for a measured
6–11 s once it hangs free, and the carry is sized to finish inside that window:
on this take `CARRY_TO_BIN` to `RELEASE` took 7.8 simulated seconds. The take
also recorded one collision, `arm_right_6_link` against `book_col_3_row_2_red`
during `RETREAT`, which is −0.5 on the local estimate. Runs can still drop the
book. Nothing below claims otherwise.

| # | Open item | Consequence |
|---|---|---|
| 1 | ~~Grasp retention does not work.~~ ~~No demonstration video exists.~~ **Both resolved** — the book is grasped, extracted, carried and delivered, and the take exists. | — |
| 2 | **The code that produced the delivered run is not committed.** `git status` shows six modified code files: `grasp.py`, `manipulation_node.py`, `mission_node.py`, `navigation_node.py`, `test_mission_flow.py` and `tools/record_trial.sh` (249 insertions, 65 deletions). The docs are modified alongside them. | Nothing can be tagged yet: a release cut from `6020dce` would not contain the carry aim that delivers the book, nor the recorder that proves the robot is in the picture. |
| 3 | **The delivering run's evidence is in the repository; its trial record is not.** `media/` now holds the take's solution log, simulator log, opening frame, closing frame (timer frozen on `BOOK DELIVERED`) and `delivered_take.sha256`, which is what ties an uploaded video back to this run — YouTube re-encodes, so check the digests before upload, not after. `erc_images/` now holds the delivering run's own four images, including the `red_bin_*.png` that was missing and the shelf-column image reading *"requested column 2 → physical shelf slot 3"*, `3@slot1, 5@slot2, 2@slot3, 1@slot4, 4@slot5` — which is the evidence for `book_col_3_row_2_red`, not an assertion about it. What is still absent is a machine-readable record for the take: the recorder ends on the organisers' `/bin_contacts` event, before the run writes one. `results/` therefore still holds only the five failing trials of 10 September. The video itself (11.7 MiB) stays out of the history deliberately. | Identification and delivery are now evidenced in-repo; the scoring JSON for the delivering run is not. |
| 4 | **The report is therefore stale.** `report/build_report.py` reads every result from `results/*.json` at build time, so the committed PDF describes the failing trials. | Must be rebuilt after item 3, not edited. |
| 5 | **The deadline is unconfirmed.** The RIT page states 1 August 2026; the v1.0 PDF states 15 September 2026. Today is 12 September 2026. | If 1 August is correct the window has closed; if 15 September is correct, three days remain. **Confirm with the organisers, including their timezone.** |
| 6 | ~~Logo file not in the repository.~~ **Done.** `docs/assets/rafeeq_logo.jpeg` leads the README, the report title page and the video's title card. | — |

---

## Form requirements

| Requirement | Status | Where / note |
|---|---|---|
| Public GitHub repository | ☐ **to verify** | `https://github.com/4waiz/Rafeeq-library` — confirm it is public and loads while signed out |
| Release titled `v1.0.0` | ☐ **not created** | Must be cut from a commit that contains the delivering code (open item 2). **The organiser repository has inherited tags `v1.0.0`, `v1.0.1`, `v1.0.3` — an inherited `v1.0.0` is NOT our release.** See Tag safety. |
| YouTube video, Public or Unlisted, ≤ 5 minutes | ☐ **recorded, not uploaded** | The take is 4:34 — 274 s, 1920×1080 at 15 fps, 11.7 MiB. Uploading it and pasting the link into the form, the README and the report is a human action. |
| Final PDF report, ≤ 5 body pages, < 10 MB | ☐ **built, rebuild required** | `report/Rafeeq_ERC2026_Phase1_Report.pdf` — 6 pages (title + 5 body), 0.17 MB. Its Results section is read from `results/` at build time, which still holds the failing trials. Rebuild with `python3 report/build_report.py` once the delivering run's records are in. |
| Registered team name used consistently | ☑ | "Team Kanban" in README, report, package metadata, evidence images and the video title card |
| Mentor and university named | ☑ | Yazan Alahmed, Al Ain University — README, report, title card |

## The demonstration video

Recorded by [`tools/record_trial.sh`](tools/record_trial.sh), which builds the
take to the Phase 1 rules rather than editing one together afterwards. Each
requirement below was checked against that script and against the delivered
take.

| Requirement | How it is met | Status |
|---|---|---|
| Team title card before the trial | `title.sh` shows the team name, robot name, competition, phase, university, mentor, the trial's column and colour, and "Unedited run, real time, no speed-up", with the team logo beside it, for six seconds before anything is launched | ☑ in the take |
| `simulation.launch.py` launched on camera | `typed.sh` prints the command at a prompt a character at a time and then executes that same string — what is shown is what runs | ☑ |
| `solution.launch.py` launched on camera | Same mechanism, with the judges' two arguments: `shelf_column_number:=2 book_colour:=red` | ☑ |
| Trial timer on screen | `timer.sh` starts when `solution.launch.py` is launched, prints to a tenth of a second beside the live mission state, and freezes on the result | ☑ — read 221.3 s at the delivery and froze on `BOOK DELIVERED`. It is wall clock, not the mission's simulated `t+` |
| One continuous take, no cuts | A single `ffmpeg` `x11grab` of one virtual display runs for the whole take; the only later step re-encodes that same stream for upload | ☑ |
| No speed-up | Captured and re-encoded at the same frame rate; nothing is dropped, trimmed or retimed | ☑ |
| Under five minutes | 4:34 (274 s) | ☑ |
| The tasks are completed on camera | The run ends on the organisers' `/bin_contacts` event, not on the gripper opening; the timer shows `BOOK DELIVERED` and holds on the final frame | ☑ |
| The robot is in the picture | The world is started `headless:=true` and the viewer (`gz sim -g`) attached only once `tiago_pro` is in the model list, then asked to follow it — a named `/gui/currently_tracked` target means the viewer has the robot in its scene, an empty one means it does not. The viewer is restarted once, and the take abandoned if the second attempt is also empty. The frame the trial starts from is saved beside the video as `<video>_first_frame.png` | ☑ |
| Uploaded to YouTube, Public or Unlisted | Human action | ☐ |
| The take's logs kept with the submission | `record_trial.sh` writes `<video>_solution.log` and `<video>_simulation.log` beside the `.mp4`; neither those nor the video are in this repository yet | ☐ |

Three honest notes to keep with the video:

* It is one run. The take is evidence that the task can be completed, not that
  it is completed every time.
* The script abandons a take early, with the simulator's own log, if the arena
  never spawns, if the organisers' controller spawners lose their startup race,
  or if the viewer came up without the robot in its scene. All three happen
  intermittently on this machine, so a recording session is expected to cost
  several attempts.
* Watch a finished take before treating it as evidence. The take above is a
  re-recording. The take it replaced recorded a genuine delivery that nobody
  could see: the viewer had lost its startup race with the robot's spawn, so the
  run played out in an empty arena. The logs were right and the video was
  worthless. The check described above exists because of it — and that earlier
  take's figures (4:29, a 232.7 s timer, `book_col_2_row_3_red` at t+105.1 s)
  are superseded and must not be quoted anywhere.

## Tag safety

Re-verified 12 September 2026.

- ☑ `git tag -l` is empty in the submission repository
- ☑ The only remote is `origin → github.com/4waiz/Rafeeq-library`
  (the organiser repository is **not** a remote here)
- ☐ Before tagging, re-run `git tag -l` and `git remote -v`
- ☐ Tag the exact validated commit, never a branch head assumed to match
- ☐ Never move or force-push a tag that already exists on the remote

## Code and evidence

| Item | Status | Note |
|---|---|---|
| Judges' exact launch commands work | ☑ verified | Both commands run every trial in `results/`, and both were typed and launched on camera in the delivered take |
| Two required inputs validated (1–5, four colours) | ☑ verified | Rejected before any motion; 20-combination parametrised test |
| No organiser file modified | ☑ verified | `git diff --name-only 2aa5a4a HEAD` touches only `src/rafeeq_solution/`, `tools/`, docs and new top-level directories |
| One arm used for retrieval | ☑ verified | Other arm tucked |
| `/erc/shelf_column_identification` published after identification | ☑ verified | Latched `Int32` from `perception_node`; trial logs |
| `/erc/shelf_row_identification` published after identification | ☑ verified | Latched `Int32`; trial logs |
| Column evidence image boxes the **column** | ☑ verified | `erc_images/shelf_column_*.png` |
| Book evidence image shows the book and its row | ☑ verified | `erc_images/target_book_*.png` |
| Bin evidence image | ☐ **not committed** | `perception_node` writes `red_bin_*.png` when the bin is identified; no run that reached the bin has had its images copied in, so `erc_images/` contains none |
| Evidence images committed (not gitignored) | ☑ for the old runs | Ten images, five column and five book, all from 10 September. The delivering run's three images are **not** among them |
| Machine-readable trial records | ☐ **stale** | `results/` holds five records across five seeds, five columns and all four colours; identification is 5 of 5 on them, but every one records `"outcome": "failed"` and `"delivered": false`. **The delivering take has no trial record at all** — the recorder deliberately ends the take on `/bin_contacts`, which is the task being finished, so the mission never reaches the banner that writes one. There is no `RUN … COMPLETE` line for it either. Its evidence is the video and `<video>_solution.log` |
| Offline tests pass | ☑ verified 12 Sep | `python3 -m pytest src/rafeeq_solution/test -q` → **162 passed**; `python3 src/rafeeq_solution/scripts/test_vision_offline.py` → **23 passed, 0 failed**. pyflakes runs inside the pytest suite (`test/test_lint.py`), and nothing skipped |
| No credentials, caches, build products or oversized media committed | ☑ spot-checked | The largest tracked files are the organisers' vendored meshes (up to 7.5 MB). Ours: `docs/assets/rafeeq_logo.jpeg` 0.42 MB, `logo.jpeg` 0.18 MB (a duplicate of it at the repository root, worth removing), the report PDF 0.17 MB |
| Every README link resolves | ☑ verified 12 Sep | Every relative link and image `src` in `README.md` resolves in the working tree; the two broken image links recorded earlier were repointed at `shelf_column_20260910_075409_862.png` and `target_book_20260910_075442_905.png`, which exist. Re-check on GitHub once the README carries the video link |

## Before publishing the release

1. ☐ Commit the six modified code files, then re-run both offline suites against the commit
2. ☐ Record or re-run a delivering trial **against that commit**, and keep its solution log, simulation log and the three evidence images. A *recorded* take stops at `/bin_contacts` and writes no trial record, so a machine-readable record has to come from a trial run outside the recorder
3. ☐ Copy that run's record and log into `results/`, its images into `erc_images/`, and delete or clearly supersede the stale 10 September set
4. ☐ Rebuild the report from the final records; check page count, size and that no link is dead
5. ☐ `git status` clean; review the full diff
6. ☐ **Fresh clone into an empty directory; `colcon build --symlink-install`; run the judges' two commands with no edits.** Attempted and abandoned: cloning this tree out of the Windows filesystem into WSL stalls on the ~5 GB of vendored packages. Verified instead, directly against the commit: all text blobs are LF, shell scripts are executable, all required files present, offline tests pass. **A real fresh-clone build is still outstanding** — run it on a Linux host where the repository lives on the native filesystem
7. ☐ Confirm every README link resolves on GitHub, the video link included
8. ☐ Confirm the repository loads while signed out
9. ☐ Draft release notes: baseline commit, what is verified, what is measured, what is still timing-critical, reproduction commands
10. ☐ **Get explicit approval before pushing or publishing** — nothing is pushed without it

## Human actions still required

| Action | Why |
|---|---|
| Confirm the Phase 1 deadline and timezone with the organisers | Two published dates conflict; one has passed |
| Upload the 4:34 take to YouTube as Public or Unlisted | `Rafeeq_Phase1_TeamKanban_delivered.mp4` on the Desktop (`/tmp/rafeeq_take44.mp4` in the container) — not in this repository |
| Put the video link in the submission form, the README and the report | Three places, one link |
| Submit the entry form: repository URL, release tag, video link, report | The submission itself is not something the repository can do |
| Approve the push and the `v1.0.0` release | Nothing is published without it |
| Decide how many further trials to record before submitting | The delivery is proven once; more takes would measure how often it holds |
