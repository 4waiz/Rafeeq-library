#!/bin/bash
# Record one competition trial exactly as the Phase 1 document asks for it.
#
#   * a title card with the team name, shown BEFORE the trial begins
#   * simulation.launch.py launched on camera, then solution.launch.py
#   * a timer on screen from the moment the trial starts until it finishes
#   * one continuous take - no cuts, no edits, no speed-up
#
# Runs inside the simulation container, on a virtual X display of its own, so
# the recording holds the simulator and the two launch terminals and nothing
# else from the machine it runs on.  ffmpeg grabs that display for the whole
# take; nothing is composited or trimmed afterwards (the final step only
# re-wraps the stream into an .mp4 container).
#
# Usage (inside the container):
#   tools/record_trial.sh <column 1-5> <colour> [seed]
#
# Environment:
#   OUT          output file (default /tmp/rafeeq_trial.mp4)
#   FPS          capture frame rate (default 15)
#   TRIAL_LIMIT  seconds before the take is abandoned (default 900)
#
# No 'set -u': ROS's own setup.bash reads unset variables, and under nounset
# sourcing it kills this script - and the trap below then tears down the take
# a minute in.  Every variable here has a default instead.

COLUMN="${1:-2}"
COLOUR="${2:-red}"
SEED="${3:-}"
OUT="${OUT:-/tmp/rafeeq_trial.mp4}"
FPS="${FPS:-15}"
TRIAL_LIMIT="${TRIAL_LIMIT:-900}"

W=1920
H=1080
export DISPLAY=:99
WORK=/tmp/rafeeq_rec
rm -rf "$WORK"
mkdir -p "$WORK"
SOLUTION_LOG="$WORK/solution.log"

# The ROS / Gazebo environment every terminal needs.  Written to a file so the
# terminals on camera show the launch commands and not a page of exports.
cat > "$WORK/env.sh" <<EOF
source /opt/ros/humble/setup.bash
source /opt/erc_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/opt/erc_ws/install/erc_description/share:/opt/erc_ws/install/omni_base_description/share:/opt/erc_ws/install/tiago_pro_description/share:/opt/erc_ws/install/pal_sea_arm_description/share:/opt/erc_ws/install/tiago_pro_head_description/share:/opt/erc_ws/install/pal_pro_gripper_description/share:/opt/erc_ws/install/pal_gripper_description/share:/opt/erc_ws/install/pal_urdf_utils/share:/opt/ros/humble/share
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/erc_ws/install/gz_ros2_control/lib
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-23}
export ERC_IMAGES_DIR=/opt/erc_ws/src/erc_images
# Render on the GPU through Mesa's d3d12 driver (WSL).  In software the
# simulator ran at a real-time factor of 0.36; on the GPU it runs at 0.65-0.70,
# which is what fits an unedited, real-time run inside the 5-minute limit.
# Only the rendering changes - the physics and the solution are identical.
if [ -e /usr/lib/wsl/lib/libd3d12.so ]; then
    export GALLIUM_DRIVER=d3d12
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    export LD_LIBRARY_PATH=/usr/lib/wsl/lib:\$LD_LIBRARY_PATH
fi
cd /opt/erc_ws
EOF

# --------------------------------------------------------------- helpers
# Print a command at a prompt, a character at a time, then run it.  What is
# shown is exactly what is executed.
cat > "$WORK/typed.sh" <<'EOF'
#!/bin/bash
source /tmp/rafeeq_rec/env.sh
prompt='\033[1;32mkanban@rafeeq\033[0m:\033[1;34m/opt/erc_ws\033[0m$ '
for line in "$@"; do
    printf "$prompt"
    for ((i = 0; i < ${#line}; i++)); do
        printf '%s' "${line:$i:1}"
        sleep 0.03
    done
    printf '\n'
    sleep 0.4
    eval "$line"
done
exec bash
EOF
chmod +x "$WORK/typed.sh"

# Title card: team name on screen before anything is launched.
cat > "$WORK/title.sh" <<EOF
#!/bin/bash
tput civis
clear
printf '\n\n\n\n\n'
printf ' \033[1m====================================\033[0m\n\n'
printf ' \033[1;33m           TEAM KANBAN\033[0m\n\n'
printf ' \033[1m     Rafeeq  -  Library Robot\033[0m\n\n'
printf '  Emirates Robotics Competition 2026\n'
printf '         Phase 1 submission\n\n'
printf '  Al Ain University\n'
printf '  Mentor: Yazan Alahmed\n\n'
printf ' \033[1m====================================\033[0m\n\n'
printf '  Trial: shelf column ${COLUMN}, ${COLOUR} book\n'
printf '  Unedited run, real time, no speed-up\n'
printf '  Recorded %s\n' "\$(TZ=Asia/Dubai date '+%Y-%m-%d %H:%M UAE')"
sleep 100000
EOF
chmod +x "$WORK/title.sh"
LOGO="${LOGO:-/tmp/rafeeq_logo.png}"

# The timer.  Starts when solution.launch.py is launched and freezes when the
# mission reports its result (or the take is abandoned).
cat > "$WORK/timer.sh" <<EOF
#!/bin/bash
tput civis
start=\$(date +%s.%N)
result=''
while :; do
    # awk, not bc: bc is not in the simulation image, and without it the
    # timer sat at 00:00 for a whole take.
    t=\$(awk -v s="\$start" -v n="\$(date +%s.%N)" 'BEGIN { printf "%.1f", n - s }')
    s=\${t%.*}
    tenth=\${t#*.}
    state=\$(grep -aoE 'STATE\] [A-Z_]+' "$SOLUTION_LOG" 2>/dev/null | tail -1 | cut -c8-)
    if [ -z "\$result" ]; then
        if grep -aq 'OFFICIAL COMPLETION EVENT' "$SOLUTION_LOG" 2>/dev/null
        then
            # The bin's own contact sensor reporting the book is the task
            # being finished; what the arm does afterwards is tidying up.
            result='BOOK DELIVERED'
        elif grep -aq 'RUN .* COMPLETE' "$SOLUTION_LOG" 2>/dev/null; then
            result='TRIAL COMPLETE'
        elif grep -aq 'RUN .* FAILED' "$SOLUTION_LOG" 2>/dev/null; then
            result='TRIAL FAILED'
        fi
    fi
    printf '\033[H\033[2J'
    printf '\n  \033[1mTRIAL TIME\033[0m   \033[1;33m%02d:%02d.%s\033[0m\n' \$((s / 60)) \$((s % 60)) "\$tenth"
    printf '  state: %-22s\n' "\${state:-starting}"
    if [ -n "\$result" ]; then
        printf '\n  \033[1;32m%s\033[0m\n' "\$result"
        echo "\$t" > $WORK/final_time
        sleep 100000
    fi
    sleep 0.1
done
EOF
chmod +x "$WORK/timer.sh"

term () {   # term <title> <geometry> <font size> <command...>
    local title="$1" geom="$2" size="$3"
    shift 3
    xterm -T "$title" -geometry "$geom" -fa 'DejaVu Sans Mono' -fs "$size" \
        -bg '#101418' -fg '#e6e6e6' -bd '#101418' -b 0 -e "$@" &
}

cleanup () {
    [ -n "${FF:-}" ] && kill -INT "$FF" 2>/dev/null && wait "$FF" 2>/dev/null
    pkill -f 'ros2 launch' 2>/dev/null
    pkill -f 'gz sim' 2>/dev/null
    pkill xterm 2>/dev/null
    pkill Xvfb 2>/dev/null
}
trap cleanup EXIT

# --------------------------------------------------------------- display
pkill Xvfb 2>/dev/null
Xvfb "$DISPLAY" -screen 0 "${W}x${H}x24" -nolisten tcp +extension GLX &
sleep 2
# A plain backdrop in the terminals' colour, mapped first so everything else
# sits on top of it: no bare root window shows between the windows on camera.
xterm -geometry 400x200+0+0 -bg '#101418' -fg '#101418' -b 0 \
    -e sleep 100000 &
sleep 1

# ---------------------------------------------------------- start the take
ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate "$FPS" \
    -video_size "${W}x${H}" -i "$DISPLAY" \
    -c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p \
    "$WORK/take.mkv" &
FF=$!
sleep 1

# 1. Title card, before the trial begins: the team logo beside the team name.
LOGO_PID=''
if [ -f "$LOGO" ] && command -v feh >/dev/null; then
    feh -x -Z -B '#101418' -g 640x692+120+194 "$LOGO" &
    LOGO_PID=$!
fi
term 'Team Kanban' '46x24+800+0' 28 "$WORK/title.sh"
TITLE=$!
sleep 6
kill "$TITLE" $LOGO_PID 2>/dev/null

# Layout for the run: simulator on the left, terminals down the right.
#   timer              1280,0     640 x 170
#   simulation.launch  1280,170   640 x 430
#   solution.launch    1280,600   640 x 480
# xterm geometry is in characters; at -fs 11 a cell is ~8 x 17 px.

# 2. simulation.launch.py, typed and launched on camera.
SIM_CMD="ros2 launch erc_bringup simulation.launch.py"
if [ -n "$SEED" ]; then
    SIM_CMD="export ERC_SEED=$SEED   # fixed arena layout, as in the test log
$SIM_CMD"
fi
SIM_LOG="$WORK/simulation.log"
term 'simulation.launch.py' '80x25+1280+170' 11 "$WORK/typed.sh" \
    "${SIM_CMD%%$'\n'*}" \
    ${SEED:+"ros2 launch erc_bringup simulation.launch.py 2>&1 | tee $SIM_LOG"}

# Wait for the simulator window and put it where the camera can see it.  The
# window is still being laid out when it first maps and a single resize is
# lost, so it is placed again until it sticks.
place_gazebo () {
    local w
    for w in $(xdotool search --name 'Gazebo' 2>/dev/null); do
        xdotool windowmove --sync "$w" 0 0 2>/dev/null
        xdotool windowsize --sync "$w" 1280 1080 2>/dev/null
    done
}
for _ in $(seq 1 120); do
    GZW=$(xdotool search --name 'Gazebo' 2>/dev/null | head -1)
    [ -n "$GZW" ] && break
    sleep 1
done
for _ in 1 2 3; do sleep 2; place_gazebo; done

# Wait until the arena is fully spawned - all 20 books present - before the
# trial starts, then give the controllers a moment to come up.
source "$WORK/env.sh"
for _ in $(seq 1 36); do
    n=$(timeout 20 gz model --list 2>/dev/null | grep -c 'book_col_')
    [ "${n:-0}" -ge 20 ] && break
    sleep 5
done
# No arena, no trial.  Launching the solution into a simulator that never came
# up wastes a whole take: with no /clock the mission cannot even time itself
# out, so it sits at WAIT_READY until this script's own limit expires - 27
# minutes, once.
if [ "${n:-0}" -lt 20 ]; then
    echo "the arena never spawned (${n:-0} of 20 books); abandoning this take"
    echo "--- last lines of the simulator's own output"
    tail -25 "$SIM_LOG" 2>/dev/null || echo "(no simulator log)"
    exit 1
fi

# The arena is not the robot.  The organisers' launch races its own
# controller_manager, and measured on this machine every spawner can die with
# "Failed loading controller" while the manager is still loading them - the
# world comes up, the books spawn, and the robot never moves.  The solution
# then waits out its whole 180 s readiness timeout for something that cannot
# arrive.  /joint_states only flows once the joint_state_broadcaster is really
# running, so that is what the take waits for.
READY=no
for _ in $(seq 1 40); do
    if timeout 10 ros2 topic echo --once /joint_states >/dev/null 2>&1; then
        READY=yes
        break
    fi
    sleep 3
done
if [ "$READY" != yes ]; then
    echo "the robot's controllers never came up; abandoning this take"
    echo "--- controller spawner lines from the simulator"
    grep -aiE 'spawner|Failed loading controller' "$SIM_LOG" 2>/dev/null \
        | tail -12
    exit 1
fi
place_gazebo
# Point the simulator's camera at a side-on overview that keeps the shelf, the
# robot and the bin in one shot for the whole trial.  This only moves the
# viewer's camera; nothing in the simulation is touched.
timeout 10 gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
    --reptype gz.msgs.Boolean --timeout 3000 \
    --req 'pose: {position: {x: 0.8, y: -4.0, z: 3.2}, orientation: {w: 0.6815, x: -0.1886, y: 0.1886, z: 0.6815}}' \
    >/dev/null 2>&1
# The solution waits for the controllers itself (WAIT_READY observes them),
# so there is nothing to gain from sitting here.
sleep 5

# 3. solution.launch.py - the trial starts here, and so does the timer.
term 'TRIAL TIMER' '38x5+1280+0' 20 "$WORK/timer.sh"
term 'solution.launch.py' '80x28+1280+600' 11 "$WORK/typed.sh" \
    "ros2 launch rafeeq_solution solution.launch.py shelf_column_number:=$COLUMN book_colour:=$COLOUR 2>&1 | tee $SOLUTION_LOG"

# 4. Run until the mission reports its result.
T0=$(date +%s)
while :; do
    # The delivery event ends the take as surely as the run's own banner: the
    # book is in the bin, which is the task.
    if grep -aqE 'RUN .* (COMPLETE|FAILED)|OFFICIAL COMPLETION EVENT' \
            "$SOLUTION_LOG" 2>/dev/null; then
        break
    fi
    if [ $(( $(date +%s) - T0 )) -gt "$TRIAL_LIMIT" ]; then
        echo "trial did not finish within ${TRIAL_LIMIT}s"
        break
    fi
    sleep 1
done
# Hold on the final frame so the result and the frozen timer are readable.
sleep 5

kill -INT "$FF" 2>/dev/null
wait "$FF" 2>/dev/null
FF=''
# Compress the capture for upload.  Every frame is kept, at the same rate and
# timing - the capture is encoded 'ultrafast' so it cannot steal CPU from the
# simulator while it runs, which makes it several hundred MB for five minutes.
ffmpeg -loglevel error -y -i "$WORK/take.mkv" -c:v libx264 -preset medium \
    -crf 23 -pix_fmt yuv420p -movflags +faststart "$OUT"

# Keep this take's logs beside its video.  The work directory is wiped at the
# start of the next take, and running takes back to back cost one attempt's
# cause of death before it could be read.
cp -f "$SOLUTION_LOG" "${OUT%.mp4}_solution.log" 2>/dev/null
cp -f "$SIM_LOG" "${OUT%.mp4}_simulation.log" 2>/dev/null

echo "result: $(grep -aoE 'OFFICIAL COMPLETION EVENT.*|RUN .* (COMPLETE|FAILED).*' \
    "$SOLUTION_LOG" | tail -1)"
echo "timer:  $(cat "$WORK/final_time" 2>/dev/null) s"
echo "video:  $OUT"
