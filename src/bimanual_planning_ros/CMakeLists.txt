cmake_minimum_required(VERSION 3.0.2)
project(bimanual_planning_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  controller_interface
  franka_gripper
  franka_hw
  franka_msgs
  geometry_msgs
  hardware_interface
  message_generation
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  realtime_tools
  roscpp
  rospy
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  eigen_conversions
)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_message_files(
  FILES
  ControllerInfo.msg
  Position.msg
  CollisionObjects.msg
  ContactWrench.msg
  ControllerParams.msg
  Obstacles.msg
)

add_service_files(
  FILES
  JointMotion.srv
)

generate_messages(DEPENDENCIES std_msgs moveit_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utilities dual_panda_costp_controller
  CATKIN_DEPENDS
    actionlib
    controller_interface
    franka_gripper
    franka_hw
    franka_msgs
    geometry_msgs
    hardware_interface
    message_runtime
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    pluginlib
    roscpp
    rospy
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
  DEPENDS
    Poco
    Franka
)

include_directories(
  include
)

add_library(utilities
  src/controller_type.cpp
  src/end_condition.cpp
  src/goal_type.cpp
  src/safe_and_fast_publisher.cpp
)
add_dependencies(utilities
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(utilities SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(utilities
  ${catkin_LIBRARIES}
)

add_library(dual_panda_costp_controller
  src/dual_panda_costp_controller.cpp
  src/costp_controller.cpp
  src/moveit_whole_body_avoidance.cpp
  src/panda_joint_impedance_controller.cpp
  src/franka_robot.cpp
  src/joint_motion_generator.cpp
  src/cf_agent.cpp
  src/obstacle.cpp
  src/helper_functions.cpp
  src/trajectory_buffer.cpp
)
add_dependencies(dual_panda_costp_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  utilities
)
target_link_libraries(dual_panda_costp_controller PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  dqrobotics
  dqrobotics-interface-vrep
  utilities
)
target_include_directories(dual_panda_costp_controller SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(dual_panda_costp_controller PUBLIC
  include
)

add_executable(panda_bimanual_control_node
  src/main.cpp
  src/obstacle.cpp
  src/panda_bimanual_control.cpp
  src/cf_agent.cpp
  src/cf_manager.cpp
  src/helper_functions.cpp
  src/dual_panda_interface.cpp
  src/parameter_manager.cpp
)
add_dependencies(panda_bimanual_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  utilities
)
target_include_directories(panda_bimanual_control_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(panda_bimanual_control_node
  ${catkin_LIBRARIES}
  dqrobotics
  dqrobotics-interface-vrep
  utilities
)

add_executable(vrep_interface
  src/vrep_controller.cpp
  src/vrep_interface.cpp
  src/costp_controller.cpp
  src/moveit_whole_body_avoidance.cpp
  src/franka_robot.cpp
  src/cf_agent.cpp
  src/obstacle.cpp
  src/helper_functions.cpp
  src/trajectory_buffer.cpp
)
add_dependencies(vrep_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  utilities
)
target_include_directories(vrep_interface SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(vrep_interface
  ${catkin_LIBRARIES}
  dqrobotics
  dqrobotics-interface-vrep
  utilities
)

add_executable(dynamic_obstacle_node
  src/dynamic_obstacle_node.cpp
  src/parameter_manager.cpp
  src/obstacle.cpp
)
add_dependencies(dynamic_obstacle_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(dynamic_obstacle_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(dynamic_obstacle_node
  ${catkin_LIBRARIES}
  dqrobotics
  dqrobotics-interface-vrep
  utilities
)

add_executable(vision_interface_node
  src/vision_interface_node.cpp
  src/parameter_manager.cpp
  src/obstacle.cpp
)
add_dependencies(vision_interface_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(vision_interface_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(vision_interface_node
  ${catkin_LIBRARIES}
  dqrobotics
  dqrobotics-interface-vrep
  utilities
)

install(TARGETS dual_panda_costp_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
