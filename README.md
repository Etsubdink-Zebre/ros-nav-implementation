# Robotics Project — Hospital Delivery Robot (TurtleBot3 Waffle)

Autonomous TurtleBot3 Waffle robot for hospital medicine & supply delivery.
SLAM-based navigation with Nav2, frontier exploration, multi-robot coordination,
and Gazebo Harmonic simulation.

**ROS 2 Jazzy | Gazebo Harmonic | Ubuntu 24.04 WSL | TurtleBot3 Waffle**

## Prerequisites

```bash
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-nav2-smac-planner
```

## Build

```bash
cd /mnt/c/Users/Hello/OneDrive/Documents/Projects/Robotics_Project
source /opt/ros/jazzy/setup.bash
colcon build --packages-select aws_robomaker_hospital_world --symlink-install
source install/setup.bash
```

## Quick Start — Run Everything

### Terminal 1: Launch the simulation

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aws_robomaker_hospital_world slam_nav.launch.py world_name:=hospital
```

This single command launches:
- Gazebo Harmonic (hospital world)
- TurtleBot3 Waffle robot spawn
- ROS-Gazebo bridge (odom, cmd_vel, tf, scan, imu)
- SLAM Toolbox (live mapping)
- Nav2 (MPPI controller + Hybrid-A* planner)
- Mission Server + Hermes Agent
- RViz2 visualization

> **Note:** Wait ~65 seconds for all nodes to initialize. You will see `Hermes Agent ready` in the terminal when the system is fully operational.

### Terminal 2: Send a delivery mission

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run aws_robomaker_hospital_world hermes_agent.py deliver --robot diff_drive --from reception --to patient_room1
```

### Alternative: Direct mission server command

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run aws_robomaker_hospital_world mission_server.py patrol diff_drive reception patient_room1
```

### Optional: Manual keyboard control

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Medicine Delivery Workflow

| Step | What happens |
|------|-------------|
| 1 | Robot receives patrol mission: `reception → patient_room1` |
| 2 | SLAM Toolbox builds/updates the hospital map in real-time |
| 3 | Smac Hybrid-A* planner computes an obstacle-free global path |
| 4 | MPPI controller follows the path with smooth, jerk-limited velocities |
| 5 | Robot arrives at patient room and reports mission complete |

## Named Locations

| Name | Coordinates (x, y, yaw°) | Description |
|------|--------------------------|-------------|
| reception | -8.0, 8.0, 90° | Reception center / Nurse station |
| nurse_station | -8.0, 8.0, 90° | Alias for reception |
| patient_room1 | 8.0, 6.0, 180° | Patient Room 101 |
| patient_room2 | 8.0, -6.0, 180° | Patient Room 102 |
| pharmacy | 2.0, -8.0, 0° | Pharmacy pickup |
| supply_room | -8.0, 0.0, 0° | Supply storage area |

## Package Features

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

## Workspace Layout

```
Robotics_Project/                   # colcon workspace root
├── build/                          # build artifacts
├── install/                        # compiled install (source install/setup.bash)
├── log/                            # build logs
└── ros-nav-implementation/         # source repository
    └── src/
        └── Intelligent Autonomous Hospital Delivery-world/
            ├── config/             # Nav2, SLAM, bridge YAML configs
            ├── launch/             # slam_nav.launch.py (main entry point)
            ├── models/             # TurtleBot3 URDF + hospital furniture models
            ├── scripts/            # Python nodes (hermes_agent, mission_server, etc.)
            └── worlds/             # hospital.world SDF
```

## WSL Notes

- GPU lidar and camera sensors are disabled (ogre2 segfaults on WSL without GPU passthrough).
- A `fake_laser.py` node provides synthetic `/scan` data for SLAM and Nav2 costmaps.
- Use `headless:=true` to run without the Gazebo GUI.
- RViz requires WSLg or an X server (e.g. VcXsrv).
