# Intelligent Autonomous Hospital Delivery System
**Using TurtleBot3 Waffle with Continuous SLAM and Nav2**

*   **Platform:** TurtleBot3 Waffle
*   **Middleware:** ROS 2 Jazzy
*   **Simulator:** Gazebo Harmonic
*   **Navigation:** Nav2 + MPPI Controller + Smac Hybrid-A*
*   **Mapping:** SLAM Toolbox (Online Async)
*   **Host OS:** Ubuntu 24.04 on WSL2
*   **Robot Fleet:** Single Robot
*   **Year:** 2026

## Abstract

This report presents the design and implementation of an Intelligent Autonomous Hospital Delivery System built on the TurtleBot3 Waffle mobile robot platform. The system uses continuous SLAM for real-time mapping of the AWS RoboMaker Hospital environment and Nav2 with MPPI and Hybrid-A* for robust navigation. A custom "Hermes" mission agent manages the state machine for autonomous point-to-point delivery. The system was developed and validated in Gazebo Harmonic simulation on Ubuntu 24.04 via WSL2, demonstrating solutions to common rendering and TF synchronization challenges in headless environments.

---

## 1. Introduction

Hospital logistics require robust autonomous mobile robots capable of safe indoor navigation. Manual delivery workflows contribute to clinical staff fatigue and take time away from patient care. 

This project implements an autonomous delivery system using the TurtleBot3 Waffle within the Gazebo Harmonic `aws_robomaker_hospital_world`. The robot navigates utilizing the Nav2 stack, specifically leveraging the MPPI local controller and Smac Hybrid-A* global planner to ensure kinematically smooth movements.

### 1.1 Objectives

*   Integrate the TurtleBot3 Waffle URDF into Gazebo Harmonic.
*   Deploy continuous SLAM using SLAM Toolbox for real-time mapping.
*   Configure Nav2 with the MPPI controller and Smac Hybrid-A* planner.
*   Implement a Python-based Hermes Mission Agent for delivery dispatch.
*   Overcome WSL-specific GPU rendering limitations using a custom `fake_laser.py` node and absolute topic bridging.

---

## 2. Hospital Use Case Problem

Hospital environments present unique navigation challenges. While this simulation uses a static environment to establish baseline autonomy, the real-world implications require careful parameter tuning.

| Challenge | Simulated Approach | Impact |
| :--- | :--- | :--- |
| **Narrow Corridors** | Tight doorframes in AWS Hospital World | Requires precise costmap inflation and Hybrid-A* turning radius. |
| **Delivery Workflow** | Point-to-point (Reception to Patient Room 1) | Requires high-level mission state machine (Hermes Agent). |
| **Hardware Constraints** | Running Gazebo on WSL2 without native GPU | Requires headless rendering and disabling ogre2 GPU sensors to prevent segfaults. |

**Table 1:** Implemented navigation challenges

---

## 3. System Architecture

The system uses a 3-layer hierarchical architecture separating concerns across mission management, navigation, and perception.

| Layer | Components | Responsibility |
| :--- | :--- | :--- |
| **Mission Layer** | Hermes Agent (`mission_server.py`) | State machine (Patrol, Wait, Deliver). |
| **Navigation Layer** | Nav2, BT Navigator, MPPI, Smac Hybrid-A* | Path planning, velocity control, recovery behaviors. |
| **Perception Layer** | SLAM Toolbox, `fake_laser.py` | Mapping, localization, costmap population. |

**Table 2:** Three-layer system architecture

### 3.1 ROS 2 Topic Architecture

Due to Gazebo Harmonic's behavior on WSL, specific architectural decisions were made to bridge topics effectively using absolute paths.

| Topic | Type | Publisher | Subscriber |
| :--- | :--- | :--- | :--- |
| `/scan` | `LaserScan` | `fake_laser.py` (Custom) | SLAM, Nav2 Costmaps |
| `/odom` | `Odometry` | Gazebo (DiffDrive Plugin) via Bridge | Nav2, SLAM Toolbox |
| `/map` | `OccupancyGrid` | SLAM Toolbox | Nav2 CostMap |
| `/cmd_vel` | `Twist` | Nav2 Controller | Gazebo (DiffDrive Plugin) |
| `/tf` | `TFMessage` | RSP + Gazebo Bridge | Nav2, SLAM Toolbox |
| `/joint_states` | `JointState`| Gazebo Bridge | Robot State Publisher |

**Table 3:** Key ROS 2 topic architecture

---

## 4. Robot Design - TurtleBot3 Waffle

The robot is based on the standard TurtleBot3 Waffle URDF, modified to function correctly within Gazebo Harmonic.

| Parameter | Value |
| :--- | :--- |
| Kinematics | Differential Drive |
| Base Dimension | 281 mm x 306 mm x 141 mm |
| Mass | ~1.8 kg |
| Max Linear Velocity | 0.26 m/s |
| Max Angular Velocity | 1.82 rad/s |
| LiDAR | Simulated 360-degree (LDS-01 equivalent) |

**Table 4:** TurtleBot3 Waffle hardware specifications

### 4.1 TF Tree

The transform tree is strictly maintained to ensure accurate spatial reasoning:
`map -> odom -> base_footprint -> base_link -> base_scan`

### 4.2 Gazebo Plugins

To prevent WSL segmentation faults, GPU-based visual plugins (like `gpu_lidar` and `camera`) were disabled. The active plugins include:
*   `gz-sim-diff-drive-system`: Kinematic motion and `/odom` publication.
*   `gz-sim-joint-state-publisher-system`: Wheel joint state publication.

---

## 5. Algorithms and Methodology

### 5.1 Global Planner - Smac Hybrid-A*

The `SmacPlannerHybrid` is used for global path planning. Unlike a standard A* planner, Hybrid-A* uses the Reeds-Shepp motion model. This ensures the generated path respects the minimum turning radius of the TurtleBot3, resulting in smooth, sweeping turns through hospital doorways rather than sharp 90-degree pivots.

### 5.2 Local Controller - MPPI

The Model Predictive Path Integral (MPPI) controller is the primary local controller. MPPI samples a distribution of future trajectories, evaluating them against the local costmap.

| Parameter | Value | Effect |
| :--- | :--- | :--- |
| `time_steps` | 56 | Planning horizon length |
| `batch_size` | 2000 | Number of trajectory samples |
| `vx_max` | 0.26 m/s | Maximum forward speed (Waffle limit) |
| `wz_max` | 1.0 rad/s | Maximum turning rate |

**Table 5:** Implemented MPPI controller parameters

### 5.3 Mapping - SLAM Toolbox

SLAM Toolbox operates in `online_async` mode. It consumes the `/scan` data provided by the custom `fake_laser.py` node and odometry from the Gazebo bridge to dynamically construct the 2D occupancy grid of the AWS hospital world.

---

## 6. Medicine Delivery Workflow

### 1. Initialization
The simulation launches `slam_nav.launch.py`, initializing Gazebo, spawning the TurtleBot3, and spinning up Nav2 and SLAM Toolbox. 

### 2. Mission Dispatch
A command is sent to the Hermes Agent: `ros2 run aws_robomaker_hospital_world hermes_agent.py deliver --robot diff_drive --from reception --to patient_room1`.

### 3. Navigation to Reception
The Hermes Agent sets the robot's goal to the Reception coordinates. The Smac planner calculates a path from the spawn point to the desk. The MPPI controller drives the robot, maintaining clearance from the walls.

### 4. Delivery to Patient Room
Upon arriving at Reception, the state transitions, and a new goal is sent for Patient Room 1. The robot navigates down the hospital corridor, executing smooth turns through the doorway and coming to a stop inside the patient room, successfully completing the autonomous delivery.

---

## 7. Results and Technical Resolutions

### WSL Compatibility Achievements
A major success of this project was achieving full autonomy on a WSL2 host without native GPU passthrough. This was accomplished by:
1. Disabling the `ogre2` rendering engine plugins in the URDF (`gpu_lidar`, `camera`) to prevent Gazebo segmentation faults.
2. Creating a custom `fake_laser.py` node that publishes synthetic `/scan` data relative to the `base_scan` frame, allowing Nav2 and SLAM to function perfectly without physical laser rendering.
3. Modifying the Gazebo URDF plugins to publish to absolute topic paths (`/odom`, `/joint_states`, `/tf`) to ensure the `ros_gz_bridge` correctly routed transforms to ROS 2.

### Navigation Performance
The combination of Smac Hybrid-A* and MPPI resulted in extremely smooth navigation. The robot successfully traversed from Reception to Patient Room 1 without colliding with the tight doorframes, demonstrating the kinematic superiority of Hybrid-A* over default grid-based planners.

---

## 8. Conclusion and Future Work

The implemented system successfully demonstrates end-to-end autonomous hospital delivery using the TurtleBot3 Waffle in ROS 2 Jazzy. The custom mission architecture, combined with state-of-the-art navigation algorithms (MPPI), provides a robust foundation for indoor robotics.

### 8.1 Future Work

*   **Dynamic Obstacles:** Introduce moving Gazebo actors (e.g., walking humans) to test MPPI's dynamic avoidance capabilities.
*   **Multi-Robot Fleet:** Scale the system to include multiple TurtleBots and implement a centralized fleet manager to prevent deadlocks in hallways.
*   **Physical Deployment:** Transfer the validated ROS 2 configuration directly onto a physical TurtleBot3 Waffle hardware platform.

---

## Software Stack Summary

| Component | Package / Version |
| :--- | :--- |
| **OS** | Ubuntu 24.04 (WSL2 Windows) |
| **Middleware** | ROS 2 Jazzy Jalisco |
| **Simulator** | Gazebo Harmonic (GZ Sim 8) |
| **Navigation** | Nav2 (Jazzy) |
| **Mapping** | SLAM Toolbox (`slam_toolbox`) |
| **Robot Bridge** | `ros_gz_sim` + `ros_gz_bridge` |
| **Custom Nodes** | `fake_laser.py`, `mission_server.py`, `hermes_agent.py` |
| **Languages** | Python 3.12, XML (URDF), YAML |

**Table 6:** Implemented software stack
