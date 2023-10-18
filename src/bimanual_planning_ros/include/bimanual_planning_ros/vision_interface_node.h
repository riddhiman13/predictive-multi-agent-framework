/*
   Class for obstacle information
*/
#pragma once

#include <bimanual_planning_ros/CollisionObjects.h>
#include <bimanual_planning_ros/Obstacles.h>
#include <bimanual_planning_ros/obstacle.h>
#include <bimanual_planning_ros/parameter_manager.h>
#include <bimanual_planning_ros/stl_reader.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Dense>

using Eigen::Vector3d;
using ghostplanner::cfplanner::Obstacle;

class VisionInterface {
 private:
  ros::NodeHandle n_;
  ros::Publisher obstacle_pub_, rviz_markers_pub_, moveit_obj_pub_,
      moveit_obj_pub_move_r_, moveit_obj_pub_move_l_, moveit_obj_pub_add_r_,
      moveit_obj_pub_add_l_;
  stl_reader::StlMesh<float, unsigned int> mesh_;
  visualization_msgs::Marker marker_;
  visualization_msgs::MarkerArray markers_;
  moveit_msgs::CollisionObject moveit_msg_;
  double frequency_;
  std::vector<Obstacle> obstacles_;
  bool initialized_obstacles_;
  geometry_msgs::TransformStamped O_T_Marker_stamped_;
  std::vector<std::string> frame_id_markers_;
  std::string frame_id_kobo_base_;
  int num_obstacles_;
  Matrix3d r_R_B_, l_R_B_;
  Vector3d r_r_B_, r_l_B_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  bimanual_planning_ros::CollisionObjects objects_l_, objects_r_;
  bimanual_planning_ros::Obstacles obst_msg_;
  moveit_msgs::CollisionObject getAddMessage(
      const Obstacle& obst, const int id, const std::string& base_frame,
      const stl_reader::StlMesh<float, unsigned int>& unit_sphere_mesh,
      bool left);
  moveit_msgs::CollisionObject getMoveMessage(const Vector3d& curr_pos,
      const int id, const std::string& base_frame, bool left);

 public:
  VisionInterface(double frequency);
  void loadParams();
  int getNumObstacles() { return num_obstacles_; };
  bool initializedObstacleMsgs() { return initialized_obstacles_; };
  void initObstacles();
  void updateAndPublishObstacles();
};
