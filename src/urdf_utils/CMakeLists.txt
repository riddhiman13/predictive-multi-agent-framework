cmake_minimum_required(VERSION 2.8.3)

project(urdf_utils)

find_package(catkin REQUIRED)
catkin_package(CATKIN_DEPENDS xacro)

install(DIRECTORY utils
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
