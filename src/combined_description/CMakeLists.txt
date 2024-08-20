cmake_minimum_required(VERSION 3.5)
project(combined_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
# find_package(joint_state_publisher_gui REQUIRED)
find_package(rviz2 REQUIRED)
find_package(kiwi_cell_description REQUIRED)
find_package(motoman_gp7_description REQUIRED)
find_package(motoman_resources REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF and configuration files
install(DIRECTORY urdf 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
