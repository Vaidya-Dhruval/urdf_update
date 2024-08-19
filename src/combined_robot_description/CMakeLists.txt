cmake_minimum_required(VERSION 3.8)
project(combined_robot_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install directories
install(DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config file
install(FILES rviz/view_robot.rviz
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
