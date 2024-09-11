cmake_minimum_required(VERSION 3.5)
project(robotiq_2f_85_gripper_visualization)
find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/launch)
ament_package()