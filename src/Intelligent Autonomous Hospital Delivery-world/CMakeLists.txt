cmake_minimum_required(VERSION 3.8)
project(aws_robomaker_hospital_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/hermes_agent.py
    scripts/mission_server.py
    scripts/task_allocator.py
    scripts/fleet_manager.py
    scripts/fleet_health.py
    scripts/frontier_explorer.py
    scripts/frontier_coordinator.py
    scripts/navigation.py
    scripts/path_planning.py
    scripts/waypoint_nav.py
    scripts/check_odometry.py
    scripts/reset_pose.py
    scripts/multi_teleop.py
    scripts/fleet_gui.py
    scripts/collision_monitor.py
    scripts/coverage_planner.py
    scripts/obstacle_tracker.py
    scripts/priority_collision_avoidance.py
    scripts/deadlock_recovery.py
    scripts/fake_laser.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  models
  fuel_models
  worlds
  maps
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()