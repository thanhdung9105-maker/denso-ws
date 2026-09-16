#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Ensure docker group permissions without requiring session re-login
if ! docker info >/dev/null 2>&1; then
    if getent group docker | grep -q "\b$(whoami)\b"; then
        exec sg docker "$0 $@"
    fi
fi

# Allow local GUI connections if xhost is available
xhost +local:root 2>/dev/null || xhost + 2>/dev/null || true

# Build image if not present
if ! docker image inspect denso_ros2_jazzy:latest >/dev/null 2>&1; then
    echo "[INFO] Dang build Docker image denso_ros2_jazzy:latest..."
    docker compose build
fi

echo "=========================================================="
echo "[INFO] KHOI CHAY ROBOT DENSO VS-6556 TRONG DOCKER (ROS 2)"
echo "=========================================================="
echo "-> Image: denso_ros2_jazzy:latest"
echo "-> GUI: WSLg X11 / Wayland Forwarding"
echo "-> Workspace: /home/dung/denso_ws"
echo "----------------------------------------------------------"

# Allocate pseudo-TTY only if attached to a terminal
DOCKER_FLAGS="-i"
if [ -t 0 ] && [ -t 1 ]; then
    DOCKER_FLAGS="-it"
fi

docker run --rm $DOCKER_FLAGS \
    --name denso_ros2_app \
    --network host \
    --ipc host \
    --privileged \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
    -e XDG_RUNTIME_DIR="/mnt/wslg/runtime-dir" \
    -e PULSE_SERVER="/mnt/wslg/PulseServer" \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e GALLIUM_DRIVER=llvmpipe \
    -e LP_NUM_THREADS=2 \
    -e MESA_GL_VERSION_OVERRIDE=4.5 \
    -e OGRE_RTT_MODE=FBO \
    -e QT_QPA_PLATFORM=xcb \
    -e QT_AUTO_SCREEN_SCALE_FACTOR=0 \
    -e QT_SCALE_FACTOR=1 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /mnt/wslg:/mnt/wslg:ro \
    -v /home/dung/denso_ws:/home/dung/denso_ws:rw \
    -w /home/dung/denso_ws \
    --user 1000:1000 \
    denso_ros2_jazzy:latest \
    bash -c "source /opt/ros/jazzy/setup.bash && source /home/dung/denso_ws/install/setup.bash && ros2 launch denso_vs6556 display.launch.py"
