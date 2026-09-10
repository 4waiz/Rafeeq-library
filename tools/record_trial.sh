#!/bin/bash
# Record a competition trial exactly as the Phase 1 document asks for it.
#
#   * a title card with the team name, shown BEFORE the trial begins
#   * simulation.launch.py launched on camera, then solution.launch.py
#   * a timer on screen from the moment the trial starts until it finishes
#   * one continuous take - no cuts, no edits, no speed-up
#
# The recording is of the real desktop, so whatever the judges see here is
# what actually happened.  Nothing is composited afterwards.
#
# Usage:  tools/record_trial.sh <column 1-5> <colour> [seed]
set -u

COLUMN="${1:-2}"
COLOUR="${2:-red}"
SEED="${3:-}"

TEAM="Team Kanban  -  Rafeeq Library Robot"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${OUT_DIR:-$HOME/rafeeq_video}/trial_${STAMP}_col${COLUMN}_${COLOUR}.mp4"
mkdir -p "$(dirname "$OUT")"

FFMPEG="${FFMPEG:-ffmpeg.exe}"
FPS="${FPS:-15}"

echo "recording to $OUT"

# ---------------------------------------------------------------- title card
# Shown full screen before anything is launched, so the team name is on camera
# before the trial begins, as the rules require.
title_card () {
    clear
    printf '\n\n\n'
    printf '        ============================================================\n'
    printf '\n'
    printf '                     %s\n' "$TEAM"
    printf '\n'
    printf '              Emirates Robotics Competition 2026 - Phase 1\n'
    printf '                        Al Ain University\n'
    printf '                     Mentor: Yazan Alahmed\n'
    printf '\n'
    printf '              Trial: shelf column %s, book colour %s\n' "$COLUMN" "$COLOUR"
    printf '              Recorded %s\n' "$(date '+%Y-%m-%d %H:%M:%S %Z')"
    printf '\n'
    printf '        ============================================================\n'
}

# -------------------------------------------------------------------- timer
# Runs in its own window from the instant solution.launch.py is launched.
timer_loop () {
    local start
    start=$(date +%s)
    while :; do
        local now=$(( $(date +%s) - start ))
        printf '\r  TRIAL TIME  %02d:%02d  ' $(( now / 60 )) $(( now % 60 ))
        sleep 1
    done
}

case "${1:-}" in
    --title) title_card; sleep "${2:-8}"; exit 0 ;;
    --timer) timer_loop; exit 0 ;;
esac

cat <<EOF
This script is the recording harness.  It expects to be driven by
tools/record_trial_run.sh, which arranges the windows, starts $FFMPEG,
shows the title card, and then launches the two files in view.

  column : $COLUMN
  colour : $COLOUR
  seed   : ${SEED:-<random>}
  output : $OUT
EOF
