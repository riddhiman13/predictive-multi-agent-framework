cmake_minimum_required(VERSION 2.8.3)
project(blaise_description)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
   xacro
   franka_sim_description
)

catkin_package(
  DEPENDS
   xacro
   franka_sim_description
   urdf_utils
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
