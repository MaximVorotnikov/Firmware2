cmake_minimum_required(VERSION 3.8)
project(gz_timer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)

set(COLCON_VARS ament_cmake rclcpp rosgraph_msgs)

include_directories(include)

# add_executable(gz_timer_sim src/drone_reg_node.cpp src/drone_reg_node.h)
add_executable(gz_timer_sim src/gz_timer_sim.cpp) # include/Quaternion_Solution.h include/wheeltec_robot.h
ament_target_dependencies(gz_timer_sim  ${COLCON_VARS})

install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION include
)

install(
    TARGETS gz_timer_sim
    EXPORT gz_timer_sim
    LIBRARY DESTINATION ${COLCON_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${COLCON_GLOBAL_LIB_DESTINATION}
    RUNTIME DESTINATION  ${COLCON_GLOBAL_LIB_DESTINATION}
    INCLUDES DESTINATION include
)

install(TARGETS gz_timer_sim
DESTINATION lib/${PROJECT_NAME}/)

ament_export_targets(gz_timer_sim HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
