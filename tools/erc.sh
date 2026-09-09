#!/bin/bash
# ERC 2026 helper - run from inside WSL (Ubuntu-22.04).
#
#   ./tools/erc.sh up                     start docker + container
#   ./tools/erc.sh build                  colcon build the whole workspace
#   ./tools/erc.sh sim [gui|headless]     launch simulation.launch.py
#
#   ERC_SEED=<n> ./tools/erc.sh sim ...   reproducible arena layout.
#     simulation.launch.py seeds its randomisation from ERC_SEED, so the
#     same value gives the same digit order, the same colour-to-row
#     assignment and the same book jitter every time. Fix a seed while
#     debugging, then sweep seeds to prove generality.
#   ./tools/erc.sh solution 2 red         launch solution.launch.py
#   ./tools/erc.sh shell                  open a shell inside the container
#   ./tools/erc.sh status                 what is running, is the clock moving
#   ./tools/erc.sh logs sim|sol           tail a launch log
#   ./tools/erc.sh stop                   stop the sim (container stays up)
#
# Two things this wrapper exists to get right:
#   1. BOTH compose files are always passed. The GPU overlay is what injects
#      the NVIDIA runtime; omitting it silently recreates the container
#      without GPU access.
#   2. It never runs `docker compose down`. install/ and build/ live INSIDE
#      the container - only src/ is bind-mounted - so down() destroys the
#      entire workspace build.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

CF="-f docker/docker-compose.yml -f docker/docker-compose.gpu.yml"
DC="docker compose $CF"
SRC='source /opt/erc_ws/install/setup.bash'

_daemon() {
    if ! docker info >/dev/null 2>&1; then
        echo "[erc] starting the docker daemon (WSL has no systemd)"
        sudo service docker start >/dev/null 2>&1 || true
        sleep 3
    fi
}

_keepalive() {
    # Without a resident process WSL tears the VM down when no shell is
    # attached, killing dockerd and any running simulation.
    pgrep -f erc-keepalive >/dev/null 2>&1 || \
        nohup bash -c 'exec -a erc-keepalive sleep infinity' >/dev/null 2>&1 &
}

_up() {
    _daemon
    _keepalive
    xhost +local:docker >/dev/null 2>&1 || true
    $DC up -d >/dev/null
    sleep 2
}

case "${1:-status}" in

up)
    _up
    docker ps --format '  {{.Names}}  {{.Status}}'
    ;;

build)
    _up
    $DC exec -T erc /entrypoint.sh bash -c \
        'cd /opt/erc_ws && colcon build --symlink-install' 2>&1 | tail -8
    ;;

sim)
    _up
    MODE="${2:-gui}"
    [ "$MODE" = "headless" ] && HL=true || HL=false
    echo "[erc] stopping any previous sim"
    $DC exec -T erc bash -c 'pkill -f "gz sim" || true; pkill -f "ros2 launch" || true; pkill -f parameter_bridge || true; sleep 3' || true
    if [ -n "${ERC_SEED:-}" ]; then
        echo "[erc] ERC_SEED=$ERC_SEED (reproducible layout)"
    fi
    echo "[erc] launching simulation.launch.py headless:=$HL"
    $DC exec -d erc /entrypoint.sh bash -c \
        "$SRC && ${ERC_SEED:+export ERC_SEED=$ERC_SEED;} exec ros2 launch erc_bringup simulation.launch.py headless:=$HL > /tmp/sim.log 2>&1"
    echo "[erc] give it ~45 s, then: ./tools/erc.sh status"
    ;;

solution)
    _up
    COL="${2:-2}"; COLOUR="${3:-red}"
    echo "[erc] launching solution.launch.py shelf_column_number:=$COL book_colour:=$COLOUR"
    $DC exec -d erc /entrypoint.sh bash -c \
        "$SRC && exec ros2 launch rafeeq_solution solution.launch.py shelf_column_number:=$COL book_colour:=$COLOUR > /tmp/sol.log 2>&1"
    echo "[erc] follow it with: ./tools/erc.sh logs sol"
    ;;

shell)
    # /entrypoint.sh sources /opt/ros/humble but NOT the workspace overlay, so
    # a bare shell cannot see erc_bringup or rafeeq_solution ("Package not
    # found"). Persist the overlay into .bashrc so every future shell has it.
    _up
    exec $DC exec erc /entrypoint.sh bash -c '
        grep -q erc_ws/install /root/.bashrc 2>/dev/null || cat >> /root/.bashrc <<'"'"'RC'"'"'
source /opt/erc_ws/install/setup.bash 2>/dev/null
cd /opt/erc_ws
RC
        exec bash'
    ;;

status)
    _daemon
    echo "=== container ==="
    docker ps --format '  {{.Names}}  {{.Status}}' 2>/dev/null || echo "  not running"
    echo "=== gazebo ==="
    $DC exec -T erc bash -c 'ps aux | grep "[g]z sim" | awk "{print \"  \", \$12, \$13}"' 2>/dev/null || echo "  none"
    echo "=== sim clock ==="
    A=$($DC exec -T erc /entrypoint.sh bash -c "$SRC && timeout 6 ros2 topic echo /clock --once 2>/dev/null | grep -m1 sec: | tr -dc 0-9" 2>/dev/null || true)
    sleep 8
    B=$($DC exec -T erc /entrypoint.sh bash -c "$SRC && timeout 6 ros2 topic echo /clock --once 2>/dev/null | grep -m1 sec: | tr -dc 0-9" 2>/dev/null || true)
    if [ -n "${A:-}" ] && [ -n "${B:-}" ]; then
        echo "  advanced $((B-A)) s in 8 s wall (RTF ~ $(awk "BEGIN{printf \"%.2f\", ($B-$A)/8}"))"
    else
        echo "  /clock not publishing - the sim is not up"
    fi
    echo "=== mission state ==="
    $DC exec -T erc bash -c 'grep -oE "\[STATE\] [A-Z_]+" /tmp/sol.log 2>/dev/null | tail -1' || echo "  no solution log"
    echo "=== saved images ==="
    $DC exec -T erc bash -c 'ls /opt/erc_ws/src/erc_images/*.png 2>/dev/null | tail -5' || echo "  none yet"
    ;;

logs)
    _daemon
    case "${2:-sim}" in
        sol|solution) $DC exec -T erc bash -c 'tail -f /tmp/sol.log' ;;
        *)            $DC exec -T erc bash -c 'tail -f /tmp/sim.log' ;;
    esac
    ;;

stop)
    _daemon
    $DC exec -T erc bash -c '
        pkill -f "ros2 launch" || true
        pkill -f "gz sim"      || true
        pkill -f parameter_bridge || true
        pkill -f robot_state_publisher || true
        sleep 2
        pkill -9 -f "gz sim" || true
        pkill -9 -f parameter_bridge || true' || true
    echo "[erc] sim stopped (container left running so install/ survives)"
    ;;

window)
    # Gazebo's GUI opens wherever Weston last put it, which under WSLg is
    # regularly off the bottom-right of a 1920x1080 screen - the window is
    # mapped and rendering, you just cannot see it. This drags it back.
    #
    # wmctrl does not work here: WSLg's Weston compositor does not implement
    # enough EWMH for it to enumerate windows. xdotool talks raw X and does.
    _up
    $DC exec -T erc bash -c '
        command -v xdotool >/dev/null 2>&1 || {
            apt-get update -qq >/dev/null 2>&1
            apt-get install -y -qq xdotool x11-utils >/dev/null 2>&1; }
        ID=$(xdotool search --name "Gazebo Sim" 2>/dev/null | tail -1)
        if [ -z "$ID" ]; then
            echo "[erc] no Gazebo window found - is the sim running with a GUI?"
            exit 1
        fi
        xdotool windowmove "$ID" 30 30
        xdotool windowsize "$ID" 1280 720
        xdotool windowraise "$ID"
        xdotool windowactivate "$ID" 2>/dev/null || true
        xwininfo -root -tree 2>/dev/null | grep -i "gazebo sim"'
    ;;

shot)
    # Screenshot the Gazebo window to a PNG on the Windows desktop. Useful
    # for the competition video and for checking the sim without alt-tabbing.
    _up
    $DC exec -T erc bash -c '
        command -v import >/dev/null 2>&1 || {
            apt-get update -qq >/dev/null 2>&1
            apt-get install -y -qq imagemagick xdotool >/dev/null 2>&1; }
        ID=$(xdotool search --name "Gazebo Sim" 2>/dev/null | tail -1)
        [ -n "$ID" ] && xdotool windowraise "$ID" && sleep 1
        import -window "${ID:-root}" /opt/erc_ws/src/gzshot.png' || true
    OUT="${2:-/mnt/c/Users/$USER/Desktop/gzshot.png}"
    if cp "$REPO/src/gzshot.png" "$OUT" 2>/dev/null; then
        echo "[erc] saved to $OUT"
    else
        echo "[erc] saved to $REPO/src/gzshot.png"
    fi
    ;;

recreate)
    # Use this when the Gazebo GUI runs (gz sim gui is alive and burning CPU)
    # but no window appears. WSLg recreates /tmp/.X11-unix/X0 every time WSL
    # restarts, and a container's bind mount still points at the OLD socket -
    # so Gazebo renders happily into a dead X server. Compare the timestamps:
    #   ls -la /tmp/.X11-unix                          (WSL host)
    #   docker compose exec erc ls -la /tmp/.X11-unix   (container)
    # If they differ, the mount is stale and only recreating fixes it.
    #
    # This DOES destroy install/ and build/ (they live in the container), so
    # it rebuilds afterwards - about 20 s for all 33 packages.
    _daemon
    _keepalive
    echo "[erc] recreating the container to pick up the current X socket"
    $DC down >/dev/null 2>&1 || true
    $DC up -d >/dev/null
    sleep 4
    echo "[erc] rebuilding the workspace"
    $DC exec -T erc /entrypoint.sh bash -c \
        'cd /opt/erc_ws && colcon build --symlink-install 2>&1 | tail -3'
    echo "[erc] done. now: ./tools/erc.sh sim"
    ;;

reset)
    # Hard reset. `restart` kills every process inside the container but
    # keeps the container itself, so install/ and build/ survive. Never use
    # `down` here - that deletes them.
    _daemon
    echo "[erc] restarting the container (keeps install/, kills all processes)"
    $DC restart erc >/dev/null
    sleep 5
    echo "[erc] clean. now: ./tools/erc.sh sim [gui|headless]"
    ;;

*)
    sed -n '2,20p' "${BASH_SOURCE[0]}"
    ;;
esac
