#!/bin/bash
# Hospital Robot — Docker on WSL2 (uses WSLg GPU passthrough)
set -e
cd "$(dirname "$0")"

# Build context: use the WORKING WSL workspace
WSL_WS=/home/fransi/hospital_ws

if [ ! -d "$WSL_WS" ]; then
    echo "ERROR: Workspace not found at $WSL_WS"
    echo "Run install.sh first in WSL: cd ~/hospital_ws && ./install.sh"
    exit 1
fi

echo "=== Building Docker image from $WSL_WS ==="
docker build \
    -f Dockerfile \
    -t hospital-robot \
    "$WSL_WS"

echo "=== Launching with WSLg GPU ==="
docker run -it --rm \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e LIBGL_ALWAYS_SOFTWARE=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /mnt/wslg:/mnt/wslg:ro \
    -v /usr/lib/wsl:/usr/lib/wsl:ro \
    --device=/dev/dri \
    hospital-robot \
    ros2 launch hospital_robot hospital_slam.launch.py
