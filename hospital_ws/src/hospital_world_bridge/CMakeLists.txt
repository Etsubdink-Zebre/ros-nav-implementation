cmake_minimum_required(VERSION 3.8)
project(hospital_world_bridge)

find_package(ament_cmake REQUIRED)

# Install everything that install.sh copies here
foreach(dir worlds models launch)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

ament_package()
