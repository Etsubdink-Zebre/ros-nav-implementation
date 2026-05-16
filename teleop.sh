#!/usr/bin/env bash
# teleop.sh — wait for the rosnav stack to be ready, then start teleop_twist_keyboard.
# Invoked from a separate Windows Terminal tab by run.sh in manual mode.

# Source ROS
if [[ -z "${ROS_DISTRO:-}" ]]; then
  source /opt/ros/jazzy/setup.bash
fi

echo "=== rosnav teleop ==="
echo "Waiting for the simulation stack to come up..."
echo "(Polls /cmd_vel or /robotN/cmd_vel; up to 5 minutes.)"
echo

# Poll for the cmd_vel topic. Multi-robot uses /robotN/cmd_vel; single uses /cmd_vel.
# 150 iterations × 2s = 5 minutes max wait (multi-robot startup can take 3-5 min).
TARGET=""
for ((i=0; i<150; i++)); do
  # Find the first cmd_vel-style topic we can use. Prefer /robot1/cmd_vel
  # in multi-robot mode; fall back to /cmd_vel for single-robot.
  TOPICS=$(ros2 topic list 2>/dev/null)
  if echo "$TOPICS" | grep -q "^/robot1/cmd_vel$"; then
    TARGET="/robot1/cmd_vel"
    break
  fi
  if echo "$TOPICS" | grep -q "^/cmd_vel$"; then
    TARGET="/cmd_vel"
    break
  fi
  sleep 2
  if (( i % 15 == 0 && i > 0 )); then
    echo "  still waiting... ($((i*2)) s elapsed)"
  fi
done

if [[ -z "$TARGET" ]]; then
  echo
  echo "Timed out after 5 minutes."
  echo "Topics seen in this tab:"
  ros2 topic list 2>&1 | sed 's/^/  /'
  echo
  echo "Possible reasons:"
  echo "  - Launch not running (check the launch tab for errors)"
  echo "  - DDS discovery issue between tabs (try: pkill -f ros2 in launch tab, relaunch)"
  echo "  - Different ROS_DOMAIN_ID in this shell (check: echo \$ROS_DOMAIN_ID)"
  echo
  echo "Press Enter to close this tab."
  read
  exit 1
fi

echo
echo "============================================"
echo "Teleop ready — driving $TARGET."
if [[ "$TARGET" == "/robot1/cmd_vel" ]]; then
  echo "(Multi-robot: this drives robot1. To switch to robot2/3, kill with Ctrl-C,"
  echo " then run:  ros2 run teleop_twist_keyboard teleop_twist_keyboard \\"
  echo "             --ros-args -r cmd_vel:=/robot2/cmd_vel)"
fi
echo
echo "Click into this tab, then:"
echo "  i / , = forward / backward"
echo "  j / l = rotate left / right"
echo "  k     = stop"
echo "  q / z = increase / decrease speed"
echo "  Ctrl-C to exit teleop"
echo "============================================"
echo

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:="$TARGET"

echo
echo "Teleop closed. Press Enter to close this tab."
read
