cmake_minimum_required(VERSION 3.0.2)
project(combined_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
  robot_state_publisher
  joint_state_publisher
  rviz
  kiwi_cell_description
  motoman_gp7_description
  motoman_resources
)

catkin_package()

install(DIRECTORY launch urdf config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
