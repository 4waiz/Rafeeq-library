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
# The title card's logo, found by name rather than by extension: the asset was
# a .png, became a .jpeg, and everything that named the old extension lost its
# logo without saying so.  A staged copy is preferred (the repository is not
# always mounted in the container), then the repository's own asset.
LOGO="${LOGO:-}"
if [ -z "$LOGO" ]; then
    for candidate in /tmp/rafeeq_logo.png /tmp/rafeeq_logo.jpeg \
                     /tmp/rafeeq_logo.jpg \
                     /opt/erc_ws/src/*/docs/assets/rafeeq_logo.* \
                     /opt/erc_ws/docs/assets/rafeeq_logo.*; do
        [ -f "$candidate" ] && { LOGO="$candidate"; break; }
    done
fi

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
#
# The world is started without its viewer and the viewer is attached once the
# robot is in it.  Started together, the viewer builds its scene from whatever
# the server has at that instant, and the robot is spawned into the world a
# moment later: when the viewer loses that race the recording shows the arena
# with no robot in it - the shelf, the table and the bin, and nothing between
# them - for the whole run.  One finished take was lost to it.  Attaching the
# viewer afterwards asks the server for a scene that already has the robot, so
# there is no race left to lose.
SIM_LOG="$WORK/simulation.log"
LAUNCH_SIM="ros2 launch erc_bringup simulation.launch.py headless:=true 2>&1 | tee $SIM_LOG"
if [ -n "$SEED" ]; then
    term 'simulation.launch.py' '80x25+1280+170' 11 "$WORK/typed.sh" \
        "export ERC_SEED=$SEED   # fixed arena layout, as in the test log" \
        "$LAUNCH_SIM"
else
    term 'simulation.launch.py' '80x25+1280+170' 11 "$WORK/typed.sh" "$LAUNCH_SIM"
fi

# The viewer's window, placed where the camera can see it.  The window is
# still being laid out when it first maps and a single resize is lost, so it
# is placed again until it sticks.  It is made wider than the 1280 px the
# recording gives it: Gazebo's world and entity-tree panels take the overflow
# and sit under the terminal column, which leaves the whole 1280 px to the
# view of the arena - half again as many pixels on the robot as before.
place_gazebo () {
    local w
    for w in $(xdotool search --name 'Gazebo' 2>/dev/null); do
        xdotool windowmove --sync "$w" 0 0 2>/dev/null
        xdotool windowsize --sync "$w" 1700 1080 2>/dev/null
    done
    for w in $(xdotool search --name 'launch.py' 2>/dev/null) \
             $(xdotool search --name 'TRIAL TIMER' 2>/dev/null); do
        xdotool windowraise "$w" 2>/dev/null
    done
}

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

# The robot is in the world, so the viewer can be started: whatever scene it
# builds now has the robot in it.  Started here rather than after the
# controllers, its own startup runs while the controllers come up and costs
# the recording nothing - and a recording has five minutes to spend.
for _ in $(seq 1 24); do
    timeout 20 gz model --list 2>/dev/null | grep -q 'tiago_pro' && break
    sleep 5
done
gz sim -g > "$WORK/gui.log" 2>&1 &

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
# The robot is in the world; now attach the viewer to it.
gaze () {   # point the camera at a side-on overview that holds the shelf, the
            # robot and the bin in one shot for the whole trial.  This moves
            # the viewer's camera only; nothing in the simulation is touched.
    timeout 10 gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
        --reptype gz.msgs.Boolean --timeout 3000 \
        --req 'pose: {position: {x: 0.95, y: -3.9, z: 3.05}, orientation: {w: 0.6815, x: -0.1886, y: 0.1886, z: 0.6815}}' \
        >/dev/null 2>&1
}

# Is the robot in the picture?  Asking the viewer to follow it answers that and
# nothing else does: the follow target comes back named when the viewer has the
# robot in its scene, and empty when it has not.  The camera is put back
# afterwards, which is why this runs before the trial starts.
robot_on_camera () {
    local tracked
    timeout 10 gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
        --reptype gz.msgs.Boolean --timeout 3000 --req 'data: "tiago_pro"' \
        >/dev/null 2>&1
    sleep 3
    tracked=$(timeout 8 gz topic -e -t /gui/currently_tracked -n 1 2>/dev/null)
    timeout 10 gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
        --reptype gz.msgs.Boolean --timeout 3000 --req 'data: ""' \
        >/dev/null 2>&1
    sleep 2
    gaze
    echo "$tracked" | grep -q 'name: "tiago_pro"'
}

settle_gui () {
    for _ in $(seq 1 90); do
        [ -n "$(xdotool search --name 'Gazebo' 2>/dev/null | head -1)" ] && break
        sleep 1
    done
    for _ in 1 2 3; do sleep 2; place_gazebo; done
    gaze
    sleep 2
}

settle_gui
if ! robot_on_camera; then
    # A viewer started again on the same world builds its scene from scratch,
    # so that is the one thing worth trying.  The world itself is untouched:
    # the server is running headless and has no idea the viewer went away.
    echo "the viewer came up without the robot in it; restarting the viewer"
    pkill -f 'gz sim gui' 2>/dev/null; pkill -f 'gz sim -g' 2>/dev/null
    sleep 3
    gz sim -g > "$WORK/gui.log" 2>&1 &
    settle_gui
    if ! robot_on_camera; then
        echo "the viewer still has no robot in its scene; abandoning this take"
        echo "--- the viewer's own output"
        tail -20 "$WORK/gui.log" 2>/dev/null
        exit 1
    fi
fi
place_gazebo
# The solution waits for the controllers itself (WAIT_READY observes them),
# so there is nothing to gain from sitting here.  The camera does need it: the
# viewer glides back from the robot it was asked to follow rather than cutting,
# and a frame taken before it lands shows a view the trial never had.
sleep 5
# Keep the frame the trial starts from, so what the recording shows can be
# checked without watching the recording.
ffmpeg -loglevel error -y -f x11grab -video_size "${W}x${H}" -i "$DISPLAY" \
    -frames:v 1 "$WORK/first_frame.png" 2>/dev/null

# 3. solution.launch.py - the trial starts here, and so does the timer.
# The marker dates the trial, so the images the solution writes during it can be
# told from every earlier run's.  The launch is typed with the judges' two
# arguments and nothing else, so the evidence lands wherever the package's own
# default puts it; it is collected afterwards rather than redirected here.
touch "$WORK/trial_start"
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
# The frame the run ends on.  The take that had to be thrown away looked right
# at the start and had no robot in it at the end, so proving the framing at
# t = 0 proves only t = 0: keep the other end too.
ffmpeg -loglevel error -y -f x11grab -video_size "${W}x${H}" -i "$DISPLAY" \
    -frames:v 1 "$WORK/result_frame.png" 2>/dev/null
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
cp -f "$WORK/first_frame.png" "${OUT%.mp4}_first_frame.png" 2>/dev/null
cp -f "$WORK/result_frame.png" "${OUT%.mp4}_result_frame.png" 2>/dev/null

# The images the solution wrote during this trial, which it puts wherever the
# package's default sends them - not necessarily anywhere the host can see.
# They are the identification evidence: which plate was read above which
# physical slot, and which book was chosen.  Collected by date so an earlier
# run's images cannot be mistaken for this one's.
EVIDENCE="${OUT%.mp4}_evidence"
rm -rf "$EVIDENCE"
mkdir -p "$EVIDENCE"
for dir in /opt/erc_ws/erc_images /opt/erc_ws/src/erc_images "$ERC_IMAGES_DIR"; do
    [ -d "$dir" ] || continue
    find "$dir" -maxdepth 1 -name '*.png' -newer "$WORK/trial_start" \
        -exec cp -f {} "$EVIDENCE/" \; 2>/dev/null
done
rmdir "$EVIDENCE" 2>/dev/null   # leave nothing behind if the run wrote nothing

# Anchor the take.  Uploading re-encodes it and the container's /tmp does not
# survive, so without this nobody can show afterwards that a published video is
# this run.
( cd "$(dirname "$OUT")" && sha256sum "$(basename "$OUT")" \
    "$(basename "${OUT%.mp4}")_solution.log" \
    "$(basename "${OUT%.mp4}")_simulation.log" 2>/dev/null ) \
    > "${OUT%.mp4}.sha256"

echo "result: $(grep -aoE 'OFFICIAL COMPLETION EVENT.*|RUN .* (COMPLETE|FAILED).*' \
    "$SOLUTION_LOG" | tail -1)"
echo "timer:  $(cat "$WORK/final_time" 2>/dev/null) s"
echo "video:  $OUT"
