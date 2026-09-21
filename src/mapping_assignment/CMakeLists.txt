cmake_minimum_required(VERSION 3.5)

# cmake_policy(SET CMP0148 NEW)  
project(mapping_assignment)

# Set C++17
set(CMAKE_CXX_STANDARD 17)

# ROS 2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

#Install rviz configuration
install(
  FILES "rviz/config.rviz"
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(PROGRAMS
  scripts/main.py
  DESTINATION lib/${PROJECT_NAME} )

# main.py imports these as sibling modules, so they must be installed
# alongside it rather than left in the source scripts/ directory.
install(FILES
  scripts/engine.py
  scripts/engine_ros2.py
  scripts/grid_map.py
  scripts/helper.py
  scripts/mapping.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/local
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()