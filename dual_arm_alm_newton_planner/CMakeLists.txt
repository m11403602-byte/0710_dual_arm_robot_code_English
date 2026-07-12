cmake_minimum_required(VERSION 3.16)
project(dual_arm_alm_newton_planner)

# ===== Release + maximum optimization =====
#   Single .so architecture: all .cpp (including planner_manager) are compiled into one .so.
#   ⚠ Do not use -march=native: it makes Eigen enable AVX (32-byte alignment),
#      which is inconsistent with MoveIt's standard build (16-byte alignment) when passing Eigen objects,
#      causing a crash in __libc_free after the collision path regenerates the trajectory.
#   ✅ Verified: -O3 -DNDEBUG (without march=native) works correctly and is fast enough.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3 -DNDEBUG)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== Dependencies =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

# =====================================================================
# Single .so: core algorithm + MoveIt plugin all bundled together
#   newton_solver / avoidance_system / data_io = core (pure Eigen)
#   planner_manager = MoveIt plugin interface
#   all compiled into one .so, so Eigen objects never cross a .so boundary
# =====================================================================
add_library(${PROJECT_NAME} SHARED
  src/newton_solver.cpp
  src/avoidance_system.cpp
  src/data_io.cpp
  src/planner_manager.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp pluginlib moveit_core moveit_msgs Eigen3
)

# Register the plugin description file with moveit_core
pluginlib_export_plugin_description_file(moveit_core dual_arm_alm_newton_planner.xml)

# =====================================================================
# Standalone executable + test program (linked against the same .so, invoking AvoidanceSystem as usual)
# =====================================================================
# ===== Install =====
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(moveit_core pluginlib rclcpp moveit_msgs Eigen3)
ament_package()
