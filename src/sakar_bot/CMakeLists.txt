cmake_minimum_required(VERSION 3.8)
project(sakar_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED) 
find_package(std_msgs REQUIRED)

# ------------------------------------------
# EXECUTABLE
# ------------------------------------------
add_executable(mission_planner missions/mission_planner.cpp)

ament_target_dependencies(mission_planner
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  gazebo_ros
  gazebo_msgs
  std_msgs
)

# ------------------------------------------
# INSTALL EXECUTABLE  (THIS WAS MISSING)
# ------------------------------------------
install(TARGETS mission_planner
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------
# INSTALL SHARE FILES
# ------------------------------------------
install(
  DIRECTORY urdf launch rviz world config maps models missions
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  # scripts here if needed
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
