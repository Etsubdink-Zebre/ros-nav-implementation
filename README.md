# Robotics Project — Hospital Delivery Robot (TurtleBot3 Waffle)

Autonomous TurtleBot3 Waffle robot for hospital medicine & supply delivery.
SLAM-based navigation with Nav2, frontier exploration, multi-robot coordination,
and Gazebo Harmonic simulation.

**ROS 2 Jazzy | Gazebo Harmonic | Ubuntu 24.04 WSL | TurtleBot3 Waffle**

## Quick Start

```bash
# 1. Source ROS and workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 2. Launch simulation (TurtleBot3 in hospital world)
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=hospital rviz:=false headless:=false

# 3. In a NEW terminal — start the mission server daemon
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run diff_drive_robot mission_server.py --daemon

# 4. In a NEW terminal — deliver medicine from reception to patient room
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run diff_drive_robot mission_server.py patrol diff_drive reception patient_room1

# 5. Monitor mission progress
ros2 run diff_drive_robot mission_server.py status

# 6. (Optional) Drive with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Medicine Delivery Task

The robot autonomously delivers medicines and supplies from the **Reception Center**
(nurse station) to **Patient Room 1**. The workflow uses:

| Step | What happens |
|------|-------------|
| 1 | Robot receives patrol mission: `reception → patient_room1` |
| 2 | SLAM Toolbox builds/updates the hospital map in real-time |
| 3 | Smac Hybrid-A* planner computes an obstacle-free global path |
| 4 | MPPI controller follows the path with smooth, jerk-limited velocities |
| 5 | Robot arrives at patient room and reports mission complete |

**One-liner to run delivery:**
```bash
ros2 run diff_drive_robot mission_server.py patrol diff_drive reception patient_room1
```

Or use the helper script:
```bash
bash ros-nav-implementation/src/diff_drive_robot-main/scripts/deliver_medicine.sh
```

## Workspace Layout

```
Robotics_Project/                   # colcon workspace root
├── build/                          # build artifacts
├── install/                        # compiled install (source install/setup.bash)
├── log/                            # build logs
└── ros-nav-implementation/         # source repository
    └── src/
        ├── diff_drive_robot-main/  # ROS 2 package (ament_cmake)
        │   └── urdf/turtlebot3_waffle_gz.urdf.xacro  # TurtleBot3 for Gazebo Harmonic
        ├── turtlebot3/             # Official ROBOTIS TurtleBot3 (jazzy branch)
        │   └── turtlebot3_description/  # Meshes & official model files
        └── Intelligent Autonomous Hospital Delivery-world/  # Hospital SDF world/models
```

## Build

```bash
cd /mnt/c/Users/Hello/OneDrive/Documents/Projects/Robotics_Project
colcon build --symlink-install
source install/setup.bash
```

## Robot: TurtleBot3 Waffle

The official ROBOTIS TurtleBot3 Waffle model (jazzy branch) adapted for Gazebo Harmonic
with `gz::sim` plugins for differential drive, GPU lidar, IMU, and RGB camera.

## Package: diff_drive_robot

| Feature | Description |
|---|---|
| SLAM | SLAM Toolbox — pose-graph with Ceres solver |
| Planner | Smac Hybrid-A* with Reeds-Shepp motion model |
| Controller | MPPI (Model Predictive Path Integral) |
| Exploration | Frontier-based — single and coordinated multi-robot |
| Multi-robot | N robots, shared map, namespaced TF |
| Fleet management | Mission server, Hungarian task allocator, health monitor |
| Worlds | maze.world (self-contained SDF), hospital world |
| Velocity smoother | Jerk-limited cmd_vel pipeline |

## Named Locations

| Name | Coordinates (x, y, yaw°) | Description |
|------|--------------------------|-------------|
| reception | -8.0, 8.0, 90° | Reception center / Nurse station |
| nurse_station | -8.0, 8.0, 90° | Alias for reception |
| patient_room1 | 8.0, 6.0, 180° | Patient Room 101 |
| patient_room2 | 8.0, -6.0, 180° | Patient Room 102 |
| pharmacy | 2.0, -8.0, 0° | Pharmacy pickup |
| supply_room | -8.0, 0.0, 0° | Supply storage area |

See `ros-nav-implementation/README.md` for the full documentation.

## Worlds

- **maze** — Self-contained SDF for testing (no asset downloads)
- **Intelligent Autonomous Hospital Delivery-world** — Hospital environment (COLCON_IGNORE, not a ROS package)

## Install Dependencies

```bash
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-nav2-smac-planner
```

## WSL Note

Running Gazebo GUI on WSL requires an X server (e.g. VcXsrv, WSLg).
Use `headless:=true` to run without GUI for CI or remote testing.
