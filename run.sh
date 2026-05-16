#!/usr/bin/env bash
# run.sh — interactive launcher for the rosnav project.
#   Asks two questions, then runs the appropriate ros2 launch.
#   In manual mode, also opens a separate Windows Terminal tab for teleop.

set -e

# ---- source ROS + workspace ----
if [[ -z "${ROS_DISTRO:-}" ]]; then
  source /opt/ros/jazzy/setup.bash
fi
if [[ -f "$HOME/rosnav/install/setup.bash" ]]; then
  source "$HOME/rosnav/install/setup.bash"
else
  echo "ERROR: ~/rosnav/install/setup.bash not found." >&2
  echo "Build the workspace first:  cd ~/rosnav && colcon build --symlink-install" >&2
  exit 1
fi

# ---- menu helper ----
pick() {
  local prompt="$1"; shift
  local options=("$@")
  local n=${#options[@]}
  local reply i
  while true; do
    echo "$prompt" >&2
    for ((i=0; i<n; i++)); do
      printf "  [%d] %s\n" "$((i+1))" "${options[i]}" >&2
    done
    read -rp "Choice [1-$n]: " reply
    if [[ "$reply" =~ ^[0-9]+$ ]] && (( reply >= 1 && reply <= n )); then
      echo "$reply"
      return
    fi
    echo "  invalid — type a number between 1 and $n" >&2
  done
}

# ---- locate this script's directory (for teleop.sh path) ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_SCRIPT="$SCRIPT_DIR/teleop.sh"

echo
echo "=== rosnav launcher ==="
echo "World: maze (the only world)"
echo

# ---- step 1: fleet size ----
fleet=$(pick "Fleet size?" "Single robot" "Multi robot")
echo

# ---- step 2: control mode ----
mode=$(pick "Control mode?" "Automatic (frontier exploration)" "Manual (drive with keyboard)")
case "$mode" in 1) explore_arg="true" ;; 2) explore_arg="false" ;; esac
echo

# ---- build the launch command ----
case "$fleet" in
  1) launch_cmd=(ros2 launch diff_drive_robot slam_nav.launch.py
                 world_name:=maze "explore:=$explore_arg" rviz:=True) ;;
  2) launch_cmd=(ros2 launch diff_drive_robot multi_robot.launch.py
                 world:=maze "explore:=$explore_arg" rviz:=True) ;;
esac

# ---- manual mode: spawn teleop in a new Windows Terminal tab ----
if [[ "$mode" == "2" ]]; then
  # Always print the manual fallback so the user has a working command if the
  # auto-open misses (e.g. wt.exe not in PATH, or not running inside Windows
  # Terminal so -w 0 can't find a current window).
  echo "Manual mode."
  echo "If a teleop tab does NOT open automatically in a few seconds, open"
  echo "a new Ubuntu tab in Windows Terminal manually and run:"
  echo "    bash '$TELEOP_SCRIPT'"
  echo

  if command -v wt.exe >/dev/null 2>&1 && [[ -x "$TELEOP_SCRIPT" ]]; then
    # Use -w 0 (current window) when we're inside Windows Terminal, otherwise
    # spawn a fresh window. Detect by checking WT_SESSION (set by Windows Terminal).
    if [[ -n "${WT_SESSION:-}" ]]; then
      WT_TARGET=(-w 0 nt)
    else
      WT_TARGET=(nt)
    fi
    # Route through wsl.exe so the command runs in WSL, not as a Windows process.
    # The Ubuntu profile only controls tab styling; the actual commandline is the
    # explicit `wsl.exe -d Ubuntu -- bash <script>`.
    (
      wt.exe "${WT_TARGET[@]}" -p Ubuntu wsl.exe -d Ubuntu -- bash "$TELEOP_SCRIPT" \
        >/dev/null 2>&1 < /dev/null &
    )
    echo "Teleop tab opening — wait ~60s for it to print 'Teleop ready'."
    echo
  fi
fi

echo "Launching: ${launch_cmd[*]}"
echo "(Ctrl-C in this tab to stop the simulation.)"
echo
exec "${launch_cmd[@]}"
