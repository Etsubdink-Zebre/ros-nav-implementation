# Autonomous Hospital Delivery Robot
### TurtleBot3 Waffle · ROS 2 Jazzy · Gazebo Harmonic · Nav2 · SLAM Toolbox

Fully autonomous hospital delivery robot. Delivers medicines, supplies, and emergency
kits between reception, pharmacy, supply room, and patient rooms inside the
AWS RoboMaker hospital world.

**Target system:** WSL2 Ubuntu 24.04 with WSLg (built-in GUI).

---

## What is inside

```
hospital_ws/
  install.sh                         ← one-shot installer
  src/
    hospital_robot/                  ← main Python package
      hospital_robot/
        mission_manager.py           ← priority delivery queue + Nav2 client
        fleet_coordinator.py         ← Hungarian algorithm fleet dispatch
        obstacle_tracker.py          ← DBSCAN + Kalman filter tracker
        frontier_explorer.py         ← frontier-based autonomous exploration
      launch/
        hospital_slam.launch.py      ← single robot, SLAM mode
        hospital_nav.launch.py       ← single robot, pre-built map
        hospital_multi.launch.py     ← 3 robots + fleet coordinator
      config/
        nav2_params.yaml             ← Hybrid-A* + MPPI + AMCL
        slam_toolbox_params.yaml     ← online async SLAM
        hospital.rviz                ← RViz2 layout
      behavior_trees/
        hospital_bt.xml              ← Nav2 behavior tree with recovery

    hospital_world_bridge/           ← ament wrapper for AWS hospital world
      worlds/                        ← populated by install.sh
      models/                        ← populated by install.sh
```

---

## Step 1: Choose your setup

### Option A: Docker with GPU (recommended — stable clock)
```bash
chmod +x run.sh
./run.sh
```
Builds and launches the full system in Docker with GPU passthrough.
Gazebo GUI opens on your host. No clock instability — Nav2 works fully.

### Option B: WSL2 (development only)
```bash
cd ~/hospital_ws
chmod +x install.sh
./install.sh
source ~/.bashrc
```
WSL clock instability limits autonomous Nav2 to short sessions (< 2 min).

---

## Step 2: Verify WSLg

```bash
echo $DISPLAY          # must print :0
rviz2                  # must open a window
gz sim --help          # must print Gazebo help
```

If `$DISPLAY` is empty:
```bash
export DISPLAY=:0
echo "export DISPLAY=:0" >> ~/.bashrc
```

---

## Step 3: Launch

### Single robot — SLAM (recommended first run)
```bash
# WSL: use headless flags to avoid GUI crash + disable explorer
ros2 launch hospital_robot hospital_slam.launch.py use_rviz:=false use_explore:=false
```

**Wait 100 seconds** for Gazebo + Nav2 lifecycle + WSL activation timer to complete.

Then **in a second terminal**, activate SLAM and send deliveries:
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# Activate SLAM Toolbox (must be done manually)
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate

# Send delivery tasks
python3 send_deliveries.py
```

If `bt_navigator` is stuck at `inactive [2]`, activate it manually:
```bash
ros2 service call /bt_navigator/change_state lifecycle_msgs/srv/ChangeState \
  "{transition: {id: 3, label: 'activate'}}"
```

### Single robot — pre-built map
```bash
# Save map first (from a SLAM session)
ros2 run nav2_map_server map_saver_cli -f ~/hospital_map

# Then launch with the saved map
ros2 launch hospital_robot hospital_nav.launch.py map:=$HOME/hospital_map.yaml
```

### 3-robot fleet (Hungarian coordination)
```bash
ros2 launch hospital_robot hospital_multi.launch.py
```

**Important WSL notes:**
- Gazebo GUI (`use_rviz:=true`) crashes within minutes on llvmpipe — use `use_rviz:=false`
- SLAM Toolbox (`async_slam_toolbox_node`) does NOT auto-activate — must be done manually
- WSL clock instability causes TF buffer clears after ~2 minutes; restart for long sessions
- Hospital models use `emissive_map` materials for llvmpipe software renderer visibility

---

## Step 4: Send delivery tasks

### Batch delivery (all 8 tasks at once)
```bash
python3 send_deliveries.py
```

This sends: pharmacy→patient_room1 (STAT morphine), pharmacy→patient_room2 (URGENT antibiotics), supply_room→nurse_station (ROUTINE bandages), pharmacy→patient_room3 (URGENT IV_drip), lab→nurse_station (ROUTINE blood_sample), reception→patient_room4 (ROUTINE visitor_pass), supply_room→patient_room5 (URGENT linens), pharmacy→reception (ROUTINE prescription)

### Single delivery
```bash
# STAT delivery: pharmacy → patient_room1
ros2 topic pub --once /delivery_request std_msgs/msg/String \
  '{data: "{\"origin\": \"pharmacy\", \"destination\": \"patient_room1\", \"payload\": \"morphine\", \"priority\": \"STAT\"}"}'

# URGENT delivery: supply_room → nurse_station
ros2 topic pub --once /delivery_request std_msgs/msg/String \
  '{data: "{\"origin\": \"supply_room\", \"destination\": \"nurse_station\", \"payload\": \"bandages\", \"priority\": \"URGENT\"}"}'

# ROUTINE delivery: lab → nurse_station
ros2 topic pub --once /delivery_request std_msgs/msg/String \
  '{data: "{\"origin\": \"lab\", \"destination\": \"nurse_station\", \"payload\": \"blood_sample\", \"priority\": \"ROUTINE\"}"}'

# Multi-robot global task (fleet coordinator)
ros2 topic pub --once /hospital/request std_msgs/msg/String \
  '{data: "{\"origin\": \"pharmacy\", \"destination\": \"patient_room3\", \"payload\": \"IV_drip\", \"priority\": \"URGENT\"}"}'
```

### Available destinations & world coordinates

| Location | Coordinates (x, y, yaw) | World Position |
|---|---|---|
| reception | (0.0, -5.5, 0.0) | Waiting area, near entrance |
| pharmacy | (9.0, 10.0, 0.0) | Right north wing, storage area |
| supply_room | (-10.0, 10.0, 3.14) | Left north wing, cabinets |
| patient_room1 | (11.0, -2.0, 1.57) | Right wing top |
| patient_room2 | (11.0, -7.0, 1.57) | Right wing mid |
| patient_room3 | (11.0, -18.0, 1.57) | Right wing bottom |
| patient_room4 | (-11.0, 0.0, -1.57) | Left wing top |
| patient_room5 | (-11.0, -12.0, -1.57) | Left wing bottom |
| nurse_station | (0.0, 1.5, 0.0) | Center, confirmed by model |
| lab | (-1.0, -21.0, 0.0) | South, near XRay/Anesthesia |
| home | (-3.5, 1.0, 0.0) | Robot docking/spawn |

---

## Step 5: Teleoperation

```bash
export TURTLEBOT3_MODEL=waffle
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# w/x = forward/back, a/d = turn, s = stop
```

---

## Monitoring

```bash
# Mission status
ros2 topic echo /mission_status

# Fleet status (3-robot mode)
ros2 topic echo /hospital/fleet_status

# Task assignments (3-robot mode)
ros2 topic echo /hospital/assignments

# Tracked obstacles
ros2 topic echo /tracked_obstacles

# Exploration status
ros2 topic echo /explore/status

# TF tree
ros2 run tf2_tools view_frames

# Nav2 node states
ros2 lifecycle list

# Active topics
ros2 topic list
```

---

## Save and reload a map

```bash
# Save during SLAM session
ros2 run nav2_map_server map_saver_cli -f ~/hospital_map

# Reload for navigation-only launch
ros2 launch hospital_robot hospital_nav.launch.py map:=$HOME/hospital_map.yaml
```

---

## Pause/resume frontier exploration

```bash
# Pause (delivery task takes priority)
ros2 topic pub --once /explore/pause std_msgs/msg/Bool '{data: true}'

# Resume
ros2 topic pub --once /explore/pause std_msgs/msg/Bool '{data: false}'
```

---

## Algorithm selection

Switch global planner at runtime:

```bash
# Use Hybrid-A* (default, best for corridors)
ros2 param set /planner_server GridBased.use_astar true

# Send a goal via RViz2 "2D Goal Pose" button, or via CLI:
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"},
    pose: {position: {x: 5.0, y: 8.0, z: 0.0},
           orientation: {w: 1.0}}}'
```

---

## Troubleshooting

### WSL: Hospital world is dark/black (llvmpipe renderer)
```bash
# Check renderer
grep GL_RENDERER ~/.gz/rendering/ogre2.log
# If it says "llvmpipe", you're on software CPU rendering without GPU
```
All models use `emissive_map` materials (self-illuminating) + ambient=1.0. If still dark, verify the world loaded:
```bash
gz model --list 2>&1 | head -5
```

### WSL: Gazebo GUI crashes after a few minutes
```bash
# Launch headless — GUI is unstable on llvmpipe
ros2 launch hospital_robot hospital_slam.launch.py use_rviz:=false use_explore:=false
```

### Robot not moving after Nav2 starts
```bash
# Check Nav2 lifecycle — bt_navigator is often stuck
ros2 lifecycle get /bt_navigator

# If "inactive [2]", activate it
ros2 service call /bt_navigator/change_state lifecycle_msgs/srv/ChangeState \
  "{transition: {id: 3, label: 'activate'}}"

# Also check SLAM is active
ros2 lifecycle get /slam_toolbox
# If "unconfigured [1]", activate:
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate

# Check TF tree is connected
ros2 run tf2_tools view_frames
# Must show: map -> odom -> base_footprint -> base_link
```

### SLAM not publishing /map
```bash
# Activate SLAM Toolbox (does NOT auto-activate)
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate

# Verify scan data
ros2 topic hz /scan                # expect ~10 Hz
ros2 topic echo /scan --once       # verify frame_id: base_scan
```

### cmd_vel not reaching robot
```bash
# Test directly via Gazebo
gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.3}"
# Robot should move — if not, DiffDrive plugin isn't loaded

# Test via ROS bridge
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
# Check odom changes: gz topic -e -n 1 -t /odom
```

### "No module named hospital_robot"
```bash
source ~/hospital_ws/install/setup.bash
echo "source ~/hospital_ws/install/setup.bash" >> ~/.bashrc
```

### hospital_world_bridge models missing
The install.sh script clones the AWS repo and copies its assets.
If you skipped it, run manually:
```bash
HOSPITAL_SRC=~/aws-robomaker-hospital-world
git clone https://github.com/aws-robotics/aws-robomaker-hospital-world.git $HOSPITAL_SRC
cp -r $HOSPITAL_SRC/worlds ~/hospital_ws/src/hospital_world_bridge/
cp -r $HOSPITAL_SRC/models ~/hospital_ws/src/hospital_world_bridge/
cd ~/hospital_ws && colcon build --symlink-install
source install/setup.bash
```

### Obstacle tracker producing no output
```bash
ros2 param set /obstacle_tracker max_range 5.0
ros2 topic hz /scan                # LiDAR must be publishing
```

### Multi-robot: robots not receiving tasks
```bash
ros2 topic echo /hospital/fleet_status    # check 'online' field
ros2 topic echo /robot_1/odom --once      # confirm odom publishing
ros2 topic echo /hospital/assignments     # confirm dispatcher running
```

---

## System topology

```
Gazebo Harmonic
     │
  gz_bridge
     │
  ┌──┴────────────────────────────────┐
  │ scan  odom  cmd_vel  tf  imu      │
  └──┬────────────────────────────────┘
     │
  ┌──▼──────────────┐    ┌─────────────────┐
  │  SLAM Toolbox   │    │ obstacle_tracker │
  │  (online async) │    │ DBSCAN + Kalman  │
  └──┬──────────────┘    └────────┬────────┘
     │ /map                       │ /tracked_obstacles
  ┌──▼──────────────────────────┐ │
  │     Nav2 Stack              │◄┘
  │  Hybrid-A* + MPPI + AMCL   │
  │  Behavior Tree + Recovery   │
  └──┬──────────────────────────┘
     │ NavigateToPose action
  ┌──▼──────────────┐    ┌─────────────────┐
  │ mission_manager │    │frontier_explorer│
  │ Priority queue  │    │ OccupancyGrid   │
  │ STAT/URG/ROUTINE│    │ frontier detect │
  └─────────────────┘    └─────────────────┘
          ▲
  ┌───────┴────────────┐
  │ fleet_coordinator  │  (3-robot mode only)
  │ Hungarian assign   │
  └────────────────────┘
```

---

## Quick reference

| Command | What it does |
|---|---|
| `./install.sh` | Install everything and build |
| `ros2 launch hospital_robot hospital_slam.launch.py use_rviz:=false` | Single robot SLAM (WSL safe) |
| `ros2 lifecycle set /slam_toolbox activate` | Activate SLAM (REQUIRED) |
| `python3 send_deliveries.py` | Send all 8 delivery tasks |
| `ros2 launch hospital_robot hospital_nav.launch.py map:=...` | Single robot nav |
| `ros2 launch hospital_robot hospital_multi.launch.py` | 3 robots |
| `ros2 run nav2_map_server map_saver_cli -f ~/hospital_map` | Save map |
| `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | Teleop |
| `ros2 topic echo /mission_status` | Monitor deliveries |
| `ros2 topic echo /hospital/fleet_status` | Monitor fleet |
| `gz topic -e -t /odom` | Watch robot position |
| `gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.3}"` | Move robot directly |
