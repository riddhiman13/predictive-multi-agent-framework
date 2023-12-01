cmake_minimum_required(VERSION 2.8.3)
project(franka_sim_description)

find_package(catkin REQUIRED xacro urdf_utils)
catkin_package(CATKIN_DEPENDS xacro urdf_utils)

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
