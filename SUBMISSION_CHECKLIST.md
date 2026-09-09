# Submission checklist — ERC 2026 Phase 1

**Team Kanban · Al Ain University · Mentor: Yazan Alahmed**

Items are marked complete only when they have been verified. Anything that
depends on a human action says so explicitly.

---

## ⚠ Blocking issues — read first

| # | Issue | Consequence |
|---|---|---|
| 1 | **Grasp retention does not work.** See [`docs/grasp_blocker.md`](docs/grasp_blocker.md). | 13 of 19 points are unearned: grasp (3), shelf navigation (3, awarded on grasp), return (3, awarded on delivery), placement (2–4). |
| 2 | **No demonstration video exists**, and one cannot honestly be made, because the video must show a *successful* run. | The video is 10% of the grade. |
| 3 | **The deadline is unconfirmed.** The RIT page states 1 August 2026; the v1.0 PDF states 15 September 2026. Today is 9 September 2026. | If 1 August is correct the window has already closed. **Confirm with the organisers, including their timezone.** |
| 4 | **The tucked left arm brushes the table** while the base turns to face the shelf. It happened on all five trials, always the same contact. | −0.5 every run, 2.5 points across five. Cheap to fix: re-select the tuck pose with the table in the clearance model. |
| 5 | **Logo file not in the repository.** `docs/assets/rafeeq_logo.png` is referenced by the README and the report title page but the file has not been saved. | Broken image in both. Drop the PNG at that path. |

---

## Form requirements

| Requirement | Status | Where / note |
|---|---|---|
| Public GitHub repository | ☐ **to verify** | `https://github.com/4waiz/Rafeeq-library` — confirm it is public and loads while signed out |
| Release titled `v1.0.0` | ☐ **not created** | Must be cut from the validated commit. **Note: the organiser repository has inherited tags `v1.0.0`, `v1.0.1`, `v1.0.3` — an inherited `v1.0.0` is NOT our release.** See below. |
| YouTube video, Public or Unlisted, ≤ 5 minutes | ☐ **not recorded** | Blocked by issue 1 |
| Final PDF report, ≤ 5 body pages, < 10 MB | ☑ **built** | `report/Rafeeq_ERC2026_Phase1_Report.pdf` — 5 body pages + title, 0.09 MB. Rebuild with `python3 report/build_report.py`. Every result is read from `results/` at build time. |
| Registered team name used consistently | ☑ | "Team Kanban" in README, report, package metadata, evidence images |
| Mentor and university named | ☑ | Yazan Alahmed, Al Ain University |

## Tag safety

The local repository has **no tags**; the organiser clone under
`/home/awaiz/erc_sim_2026` has `v1.0.0`, `v1.0.1`, `v1.0.3` pointing at
organiser commits (`ae4d1d9`, `51486d7`, `0a09806`).

- ☑ Confirmed `git tag -l` is empty in the submission repository
- ☑ Confirmed the only remote is `origin → github.com/4waiz/Rafeeq-library`
  (the organiser repository is **not** a remote here)
- ☐ Before tagging, re-run `git tag -l` and `git remote -v`
- ☐ Tag the exact validated commit, never a branch head assumed to match
- ☐ Never move or force-push a tag that already exists on the remote

## Code and evidence

| Item | Status | Note |
|---|---|---|
| Judges' exact launch command works | ☑ verified | Both commands run every trial |
| Two required inputs validated (1–5, four colours) | ☑ verified | Rejected before any motion; 20-combination test |
| No organiser file modified | ☑ verified | `git diff --name-only 2aa5a4a HEAD` touches only our package, `tools/`, docs and new directories |
| One arm used for retrieval | ☑ verified | Other arm tucked |
| `/erc/shelf_column_identification` published after identification | ☑ verified | Trial logs |
| `/erc/shelf_row_identification` published after identification | ☑ verified | Trial logs |
| Column evidence image boxes the **column** | ☑ verified | `erc_images/shelf_column_*.png` |
| Book evidence image shows the book and its row | ☑ verified | `erc_images/target_book_*.png` |
| Evidence images committed (not gitignored) | ☑ fixed | The previous `.gitignore` excluded exactly what the rules require |
| Machine-readable trial records | ☑ **5 of 5** | `results/*.json`, five layout seeds, five columns, all four colours. Column and row identified, published and photographed on every one. |
| Offline tests pass | ☑ verified | 109 pytest + 23 vision, pyflakes clean |
| No credentials, caches, build products or oversized media committed | ☐ to verify | Check before tagging |

## Before publishing the release

1. ☐ `git status` clean; review the full diff
2. ☐ `python3 -m pytest src/rafeeq_solution/test -q` — all 109 pass
3. ☐ `python3 src/rafeeq_solution/scripts/test_vision_offline.py` — all pass
4. ☐ **Fresh clone into an empty directory; `colcon build --symlink-install`; run the judges' two commands with no edits.** Attempted and abandoned: cloning this tree out of the Windows filesystem into WSL stalls on the ~5 GB of vendored packages. Verified instead, directly against the commit: all text blobs are LF, shell scripts are executable, all required files present, offline tests pass. **A real fresh-clone build is still outstanding** — run it on a Linux host where the repo lives on the native filesystem.
5. ☐ Rebuild the report from the final trial records; check page count, size and that no link is dead
6. ☐ Confirm every README link resolves on GitHub
7. ☐ Confirm the repository loads while signed out
8. ☐ Draft release notes: baseline commit, what is verified, what is blocked, measured results, reproduction commands
9. ☐ **Get explicit approval before pushing or publishing** — nothing is pushed without it

## Human actions still required

| Action | Why |
|---|---|
| Confirm the Phase 1 deadline and timezone with the organisers | Two published dates conflict; one has passed |
| Save the Rafeeq logo to `docs/assets/rafeeq_logo.png` | Referenced by README and report |
| Decide whether to submit with the grasp unresolved | It is a 13-point gap; the alternative is to raise the blocker with the organisers first |
| Record the demonstration video | Only possible once a run succeeds |
| Approve the push and the `v1.0.0` release | Nothing is published without it |
