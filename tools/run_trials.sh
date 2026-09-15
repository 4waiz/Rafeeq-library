#!/bin/bash
# Run the validation trial matrix against the current solution.
#
# One trial is: a fresh simulator at a fixed seed, the solution launched into
# it, and the run left alone until the mission reaches a terminal state and
# writes its own record.  Nothing here inspects or steers the run.
#
# Runs INSIDE the simulation container:
#
#   docker cp tools/run_trials.sh erc_sim:/opt/erc_ws/tools/run_trials.sh
#   docker compose exec -T erc /entrypoint.sh bash /opt/erc_ws/tools/run_trials.sh V01
#
# Usage:
#   run_trials.sh                 every trial in the matrix, in order
#   run_trials.sh V01 V02         only those trials
#   run_trials.sh FIG [seed col colour]
#                                 one run with RViz attached, for the report's
#                                 planned-versus-executed navigation figure
#
# Environment:
#   TRIAL_LIMIT   wall seconds before a trial is abandoned (default 600)
#   SETTLE        seconds between the controllers coming up and the solution
#                 being launched (default 25)
#   RVIZ          V-id to run RViz alongside, for the navigation figure,
#                 or 'all' for every trial (default V01; empty disables it)
#   SHOTS         where the RViz screenshots go (default /opt/erc_ws/results)
#
# No 'set -u': sourcing ROS's setup.bash under nounset kills the script.
set -o pipefail

RESULTS=/opt/erc_ws/src/results
IMAGES=/opt/erc_ws/src/erc_images
TRIAL_LIMIT="${TRIAL_LIMIT:-600}"
RVIZ="${RVIZ-V01}"
SHOTS="${SHOTS:-/opt/erc_ws/results}"
RVIZ_CFG="${RVIZ_CFG:-/opt/erc_ws/tools/rafeeq_paths.rviz}"

# The matrix.  Same seeds, columns and colours as the development trials in
# results/, so the two sets are comparable run for run.
MATRIX=(
    "V01 101 2 red"
    "V02 202 4 blue"
    "V03 303 1 yellow"
    "V04 404 5 green"
    "V05 505 3 red"
)

source /opt/ros/humble/setup.bash
source /opt/erc_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/opt/erc_ws/install/erc_description/share:/opt/erc_ws/install/omni_base_description/share:/opt/erc_ws/install/tiago_pro_description/share:/opt/erc_ws/install/pal_sea_arm_description/share:/opt/erc_ws/install/tiago_pro_head_description/share:/opt/erc_ws/install/pal_pro_gripper_description/share:/opt/erc_ws/install/pal_gripper_description/share:/opt/erc_ws/install/pal_urdf_utils/share:/opt/ros/humble/share
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/erc_ws/install/gz_ros2_control/lib
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
export ERC_IMAGES_DIR=/opt/erc_ws/src/erc_images
# Render on the GPU through Mesa's d3d12 driver (WSL), as the recorded take
# did.  Only the rendering changes; the physics and the solution are identical.
if [ -e /usr/lib/wsl/lib/libd3d12.so ]; then
    export GALLIUM_DRIVER=d3d12
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH
fi
cd /opt/erc_ws
mkdir -p "$SHOTS"

teardown () {
    pkill -f 'ros2 launch' 2>/dev/null
    pkill -f rviz2 2>/dev/null
    sleep 3
    # Killing the launch process orphans its nodes rather than stopping them,
    # and an orphaned perception node from the last trial will happily answer
    # the next one's mission.  Take the nodes out by their own install paths -
    # patterns that cannot match this script's own command line.
    pkill -9 -f 'rafeeq_solution/lib' 2>/dev/null
    pkill -9 -f 'erc_bringup/lib' 2>/dev/null
    pkill -9 -f 'sensors/lib' 2>/dev/null
    pkill -9 -f parameter_bridge 2>/dev/null
    pkill -9 -f robot_state_publisher 2>/dev/null
    pkill -9 -f 'gz sim' 2>/dev/null
    sleep 4
}
trap 'teardown; pkill Xvfb 2>/dev/null' EXIT

say () { echo "[$(date -u +%H:%M:%S)] $*"; }

run_trial () {
    local id="$1" seed="$2" column="$3" colour="$4"
    local run_id="${id}_seed${seed}_col${column}_${colour}"
    local record="$RESULTS/trial_${run_id}.json"
    local log="$RESULTS/${run_id}.log"
    local simlog="/tmp/sim_${id}.log"

    say "=== $run_id ==============================================="
    rm -f "$record"
    teardown

    say "simulator, seed $seed"
    ERC_SEED="$seed" nohup ros2 launch erc_bringup simulation.launch.py \
        headless:=true > "$simlog" 2>&1 &

    # The arena is not the robot, and neither is the clock.  Wait for all
    # three: 20 books spawned, the robot in the world, and /joint_states
    # flowing - the organisers' launch races its own controller_manager, and a
    # solution launched into a world whose controllers died just waits out its
    # readiness timeout.
    local n=0 i
    for i in $(seq 1 36); do
        n=$(timeout 20 gz model --list 2>/dev/null | grep -c 'book_col_')
        [ "${n:-0}" -ge 20 ] && break
        sleep 5
    done
    if [ "${n:-0}" -lt 20 ]; then
        say "ABANDONED: the arena never spawned (${n:-0} of 20 books)"
        tail -20 "$simlog"
        return 1
    fi
    say "arena up ($n books)"

    local ready=no
    for i in $(seq 1 40); do
        if timeout 10 ros2 topic echo --once /joint_states >/dev/null 2>&1; then
            ready=yes
            break
        fi
        sleep 3
    done
    if [ "$ready" != yes ]; then
        say "ABANDONED: the robot's controllers never came up"
        grep -aiE 'spawner|Failed loading controller' "$simlog" | tail -10
        return 1
    fi
    say "controllers up"

    # Settle before launching the solution.  The controllers answering is not
    # the same as the arm being done with its own start-up: launched into that
    # window, the mission's first home command races the manipulation node's
    # and is rejected as "busy with home", which fails the run two seconds in.
    # The recorded take never saw it because attaching the viewer took half a
    # minute.  Waiting here makes every trial in the matrix start from the same
    # place; the race itself is a defect in the solution, reported as one.
    sleep "${SETTLE:-25}"

    # RViz, for the planned-versus-executed navigation figure.  It only
    # subscribes; nothing the solution does depends on it being here.  It runs
    # on a virtual display of its own with software GL: the d3d12 path is for
    # the simulator's sensors, and RViz drawing two polylines does not need it.
    local rviz_on=no
    if [ -f "$RVIZ_CFG" ] \
       && { [ "$RVIZ" = all ] || [ "$id" = "$RVIZ" ]; }; then
        pkill -f rviz2 2>/dev/null
        pkill Xvfb 2>/dev/null
        sleep 1
        rm -f /tmp/.X99-lock
        Xvfb :99 -screen 0 2040x760x24 -nolisten tcp -ac +extension GLX \
            >/tmp/xvfb.log 2>&1 &
        local up=no i
        for i in $(seq 1 15); do
            if DISPLAY=:99 xdpyinfo >/dev/null 2>&1; then up=yes; break; fi
            sleep 1
        done
        if [ "$up" != yes ]; then
            say "WARNING: no virtual display; skipping the RViz figure"
            cat /tmp/xvfb.log
        else
            DISPLAY=:99 nohup env -u GALLIUM_DRIVER \
                -u MESA_D3D12_DEFAULT_ADAPTER_NAME \
                LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d "$RVIZ_CFG" \
                > /tmp/rviz.log 2>&1 &
            for i in $(seq 1 20); do
                sleep 2
                DISPLAY=:99 xdotool search --name 'RViz' >/dev/null 2>&1 \
                    && { rviz_on=yes; break; }
            done
            if [ "$rviz_on" = yes ]; then
                sleep 5
                say "rviz2 up on :99"
            else
                say "WARNING: rviz2 did not come up"
                tail -5 /tmp/rviz.log
            fi
        fi
    fi

    say "solution: column $column, $colour"
    # image_dir is passed explicitly for the same reason results_dir is: the
    # launch file walks up from install/ looking for an erc_images directory,
    # and a stale one at the workspace root shadows the one on the bind mount.
    # Evidence written there is invisible outside the container.
    nohup ros2 launch rafeeq_solution solution.launch.py \
        shelf_column_number:="$column" book_colour:="$colour" \
        run_id:="$run_id" results_dir:="$RESULTS" \
        image_dir:="$IMAGES" > "$log" 2>&1 &

    local waited=0 state=''
    while [ "$waited" -lt "$TRIAL_LIMIT" ]; do
        [ -f "$record" ] && break
        sleep 5
        waited=$((waited + 5))
        if [ $((waited % 60)) -eq 0 ]; then
            state=$(grep -aoE 'STATE\] [A-Z_]+' "$log" 2>/dev/null | tail -1 \
                    | cut -c8-)
            say "  ${waited}s  ${state:-starting}"
        fi
    done

    # The screenshot is taken with the run finished and both paths on screen:
    # the planned path is latched, the executed one has stopped growing.
    if [ "$rviz_on" = yes ]; then
        sleep 3
        local w
        w=$(DISPLAY=:99 xdotool search --name 'RViz' 2>/dev/null | tail -1)
        if [ -n "$w" ]; then
            DISPLAY=:99 import -window "$w" "$SHOTS/nav_paths_${id}.png" \
                2>/dev/null
        fi
        [ -s "$SHOTS/nav_paths_${id}.png" ] || \
            DISPLAY=:99 import -window root "$SHOTS/nav_paths_${id}.png" \
                2>/dev/null
        say "rviz shot -> $SHOTS/nav_paths_${id}.png"
        pkill -f rviz2 2>/dev/null
        pkill Xvfb 2>/dev/null
    fi

    if [ -f "$record" ]; then
        say "$(grep -aoE 'RUN .* (COMPLETE|FAILED)[^\]*' "$log" | tail -1)"
        say "record -> $record"
    else
        say "ABANDONED: no terminal state within ${TRIAL_LIMIT}s"
        state=$(grep -aoE 'STATE\] [A-Z_]+' "$log" | tail -1 | cut -c8-)
        say "last state: ${state:-none}"
    fi
    teardown
}

# The navigation figure is captured on its own run, not on a trial: RViz on a
# virtual display costs the machine something, and every trial in the matrix
# has to be run the same way as every other.  Its record is written under the
# id FIG so it cannot be mistaken for one of the five.
if [ "${1:-}" = FIG ]; then
    RVIZ=FIG
    run_trial FIG "${2:-101}" "${3:-2}" "${4:-red}"
    say "figure run done"
    exit 0
fi

WANT=("$@")
for entry in "${MATRIX[@]}"; do
    set -- $entry
    if [ "${#WANT[@]}" -gt 0 ]; then
        case " ${WANT[*]} " in *" $1 "*) ;; *) continue ;; esac
    fi
    run_trial "$1" "$2" "$3" "$4"
done
say "matrix done"
