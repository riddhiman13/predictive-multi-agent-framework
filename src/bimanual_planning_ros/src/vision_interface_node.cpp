#include <bimanual_planning_ros/vision_interface_node.h>

using Eigen::Vector3d;
using ghostplanner::cfplanner::Obstacle;

VisionInterface::VisionInterface(double frequency)
    : initialized_obstacles_(false),
      frequency_(frequency),
      num_obstacles_(0),
      tfListener_(tfBuffer_) {
  loadParams();
  obstacle_pub_ = n_.advertise<bimanual_planning_ros::Obstacles>(
      "obstacles", 1);
  rviz_markers_pub_ = n_.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker_array", 1);
  /*moveit_obj_pub_add_l_ = n_.advertise<bimanual_planning_ros::CollisionObjects>(
       "collision_objects_add_left", 1, true);
  moveit_obj_pub_add_r_ = n_.advertise<bimanual_planning_ros::CollisionObjects>(
       "collision_objects_add_right", 1, true);
  moveit_obj_pub_move_l_ = 
	  n_.advertise<bimanual_planning_ros::CollisionObjects>(
          "collision_objects_move_left", 1, true);
  moveit_obj_pub_move_r_ =
      n_.advertise<bimanual_planning_ros::CollisionObjects>(
          "collision_objects_move_right", 1, true);*/
  // Read sphere mesh
  const std::string mesh_file = ros::package::getPath("bimanual_planning_ros") +
                                "/config/unit_sphere.stl";
  try {
    mesh_ = stl_reader::StlMesh<>(mesh_file);
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("Failed to read .stl file, e.what(): " << e.what());
  }
  frame_id_kobo_base_ = "base_link";
  for (size_t i = 0; i < num_obstacles_; i++) {
    std::string frame_id_marker = "marker_" + std::to_string(i);
    frame_id_markers_.push_back(frame_id_marker);
  }
}

/**
 * @brief Get the MoveIt CollisionObjectMsg with operation = ADD.
 * @note Eventhough all obstacles are spheres we represent their shapes with
 * mesh files instead of using the primary shape "SPHERE", because FCL
 * (collision library) is buggy when using this primary shape.
 */
moveit_msgs::CollisionObject VisionInterface::getAddMessage(
    const Obstacle& obst, const int id, const std::string& base_frame,
    const stl_reader::StlMesh<float, unsigned int>& unit_sphere_mesh,
    bool left) {
  moveit_msgs::CollisionObject msg;
  // Object id
  msg.id = "CollObject_" + std::to_string(id);
  // 0 corresponds to ADD message
  msg.operation = 0;
  // the absolute pose is returned relative to this frame
  msg.header.frame_id = base_frame;
  msg.header.stamp = ros::Time::now();
  // fill in mesh information (vertices and triangles), scale sphere accordingly
  msg.meshes.push_back(shape_msgs::Mesh());
  for (size_t i = 0; i < unit_sphere_mesh.num_vrts(); ++i) {
    geometry_msgs::Point p;
    auto cord = unit_sphere_mesh.vrt_coords(i);
    p.x = cord[0] * obst.getRadius();
    p.y = cord[1] * obst.getRadius();
    p.z = cord[2] * obst.getRadius();
    msg.meshes[0].vertices.push_back(p);
  }
  for (size_t i = 0; i < unit_sphere_mesh.num_tris(); ++i) {
    shape_msgs::MeshTriangle t;
    auto inds = unit_sphere_mesh.tri_corner_inds(i);
    t.vertex_indices = {inds[0], inds[1], inds[2]};
    msg.meshes[0].triangles.push_back(t);
  }
  // default constructors init everything with zero
  geometry_msgs::Pose identity_pose;
  identity_pose.orientation.w = 1;
  msg.mesh_poses.push_back(identity_pose);

  Vector3d pos_transformed;
  if (left) {
    pos_transformed = l_R_B_*obst.getPosition()+r_l_B_;
  } else {
    pos_transformed = r_R_B_*obst.getPosition()+r_r_B_;
  }
  // current pose
  msg.pose.position.x = pos_transformed.x();
  msg.pose.position.y = pos_transformed.y();
  msg.pose.position.z = pos_transformed.z();

  msg.pose.orientation.w = 1;
  msg.pose.orientation.x = 0;
  msg.pose.orientation.y = 0;
  msg.pose.orientation.z = 0;
  return msg;
}

moveit_msgs::CollisionObject VisionInterface::getMoveMessage(
    const Vector3d& curr_pos, const int id, const std::string& base_frame,
    bool left) {
  moveit_msgs::CollisionObject msg;
  msg.id = "CollObject_" + std::to_string(id);
  msg.operation = 3;
  msg.header.frame_id = base_frame;
  msg.header.stamp = ros::Time::now();
  Vector3d pos_transformed;
  if (left) {
    pos_transformed = l_R_B_*curr_pos+r_l_B_;
  } else {
    pos_transformed = r_R_B_*curr_pos+r_r_B_;
  }
  msg.pose.position.x = pos_transformed.x();
  msg.pose.position.y = pos_transformed.y();
  msg.pose.position.z = pos_transformed.z();
  msg.pose.orientation.w = 1;
  msg.pose.orientation.x = 0;
  msg.pose.orientation.y = 0;
  msg.pose.orientation.z = 0;
  // default constructors init everything with zero
  geometry_msgs::Pose identity_pose;
  identity_pose.orientation.w = 1;
  msg.mesh_poses.assign(1, identity_pose);
}

void VisionInterface::initObstacles() {
  bool got_tf_transform = false;
  Eigen::Affine3d O_T_Marker;
  for (size_t i = 0; i < num_obstacles_; i++) {
    try {
      O_T_Marker_stamped_ = tfBuffer_.lookupTransform(
          frame_id_kobo_base_, frame_id_markers_.at(i), ros::Time::now(),
          ros::Duration(1.0));
      tf::transformMsgToEigen(O_T_Marker_stamped_.transform, O_T_Marker);
      obstacles_.at(i).setPosition(O_T_Marker.translation());
      got_tf_transform = true;
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      got_tf_transform = false;
      obstacles_.at(i).setPosition({0.0, 0.0, 0.0});
      ROS_WARN("Cannot see aruco marker. \n");
      break;
    }
  }
  if (got_tf_transform) {
    std::string error;
    if (!tfBuffer_.canTransform("base_link", "panda_right_link0",
                  ros::Time(0), ros::Duration(3), &error)) {
    ROS_ERROR("DynamicObstacleNode: Could transform from fram base_link "
          "to arm frame panda_right_link0. Error: %s", error.c_str());
    }
    if (!tfBuffer_.canTransform("base_link", "panda_left_link0",
                  ros::Time(0), ros::Duration(3), &error)) {
    ROS_ERROR("DynamicObstacleNode: Could transform from fram base_link "
          "to arm frame panda_left_link0. Error: %s", error.c_str());
    }
    geometry_msgs::TransformStamped transform;
    transform = tfBuffer_.lookupTransform("panda_right_link0", "base_link",
                      ros::Time(0), ros::Duration(3));
    r_r_B_ = Vector3d(transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);
    Quaterniond Q_r_R_B(transform.transform.rotation.w,
              transform.transform.rotation.x,
              transform.transform.rotation.y,
              transform.transform.rotation.z);
    r_R_B_ = Q_r_R_B.toRotationMatrix();
    transform = tfBuffer_.lookupTransform("panda_left_link0", "base_link",
                      ros::Time(0), ros::Duration(3));
    r_l_B_ = Vector3d(transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);
    Quaterniond Q_l_R_B(transform.transform.rotation.w,
              transform.transform.rotation.x,
              transform.transform.rotation.y,
              transform.transform.rotation.z);
    l_R_B_ = Q_l_R_B.toRotationMatrix();
    marker_.header.frame_id = frame_id_kobo_base_;
    marker_.header.stamp = ros::Time();
    marker_.ns = "bimanual_planner";
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::SPHERE;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.lifetime = ros::Duration(0);
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.color.a = 1.0;  // Don't forget to set the alpha!
    marker_.color.r = 0.0;
    marker_.color.g = 0.0;
    marker_.color.b = 1.0;
    bimanual_planning_ros::Position obst_pos;
    bimanual_planning_ros::Position obst_vel;
    for (int i = 0; i < obstacles_.size(); ++i) {
      obst_pos.data[0] = marker_.pose.position.x =
          obstacles_.at(i).getPosition().x();
      obst_pos.data[1] = marker_.pose.position.y =
          obstacles_.at(i).getPosition().y();
      obst_pos.data[2] = marker_.pose.position.z =
          obstacles_.at(i).getPosition().z();
      obst_vel.data[0] = obst_vel.data[1] = obst_vel.data[2] = 0.0;
      obst_msg_.pos.push_back(obst_pos);
      obst_msg_.vel.push_back(obst_vel);
      obst_msg_.radius.push_back(obstacles_.at(i).getRadius());
      marker_.scale.x = obstacles_.at(i).getRadius();
      marker_.scale.y = obstacles_.at(i).getRadius();
      marker_.scale.z = obstacles_.at(i).getRadius();
      marker_.id = i;
      markers_.markers.push_back(marker_);

      /*objects_l_.objects.push_back(getAddMessage(obstacles_.at(i), i,
          "/panda_left_link0", mesh_, true));
      objects_r_.objects.push_back(getAddMessage(obstacles_.at(i), i,
          "/panda_right_link0", mesh_, false));*/
    }
    initialized_obstacles_ = true;
    obstacle_pub_.publish(obst_msg_);
    rviz_markers_pub_.publish(markers_);
    /*moveit_obj_pub_add_l_.publish(objects_l_);
    moveit_obj_pub_add_r_.publish(objects_r_);*/
  }
}

void VisionInterface::updateAndPublishObstacles() {
  std::vector<Eigen::Vector3d> old_pos;
  Eigen::Affine3d O_T_Marker;
  for (size_t i = 0; i < num_obstacles_; i++) {
    old_pos.push_back(obstacles_.at(i).getPosition());
    try {
      O_T_Marker_stamped_ = tfBuffer_.lookupTransform(
          frame_id_kobo_base_, frame_id_markers_.at(i), ros::Time::now(),
          ros::Duration(1.0));
      tf::transformMsgToEigen(O_T_Marker_stamped_.transform, O_T_Marker);
      obstacles_.at(i).setPosition(O_T_Marker.translation());
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      ROS_WARN("Cannot see aruco marker. \n");
    }
    obstacles_.at(i).setVelocity({0.0, 0.0, 0.0});
    // obstacles_.at(i).setVelocity(
    //     (obstacles_.at(i).getPosition() - old_pos.at(i)) * frequency_);
  }
    marker_.action = visualization_msgs::Marker::MODIFY;
    bimanual_planning_ros::Position obst_vel;
    markers_.markers.clear();
    for (int i = 0; i < obstacles_.size(); ++i) {
      obst_msg_.pos[i].data[0] = marker_.pose.position.x =
          obstacles_.at(i).getPosition().x();
      obst_msg_.pos[i].data[1] = marker_.pose.position.y =
          obstacles_.at(i).getPosition().y();
      obst_msg_.pos[i].data[2] = marker_.pose.position.z =
          obstacles_.at(i).getPosition().z();
      obst_msg_.vel[i].data[0] = obstacles_.at(i).getVelocity().x();
      obst_msg_.vel[i].data[1] = obstacles_.at(i).getVelocity().y();
      obst_msg_.vel[i].data[2] = obstacles_.at(i).getVelocity().z();
      marker_.scale.x = obstacles_.at(i).getRadius();
      marker_.scale.y = obstacles_.at(i).getRadius();
      marker_.scale.z = obstacles_.at(i).getRadius();
      marker_.id = i;
      markers_.markers.push_back(marker_);

      /*objects_l_.objects.push_back(getMoveMessage(
          obstacles_.at(i).getPosition(), i, "/panda_left_link0", true));
      objects_r_.objects.push_back(getMoveMessage(
          obstacles_.at(i).getPosition(), i, "/panda_right_link0", false));*/
    }
    obstacle_pub_.publish(obst_msg_);
    rviz_markers_pub_.publish(markers_);
    /*moveit_obj_pub_move_l_.publish(objects_l_);
    moveit_obj_pub_move_r_.publish(objects_r_);*/
}

void VisionInterface::loadParams() {
  Vector3d param_pos;
  Vector3d param_vel;
  double param_radius;
  ParameterManager pm("params");
  ParameterManager obstacles("obstacles");
  obstacles.addHook(
      [&]() -> void {
        obstacles_.push_back(Obstacle{param_pos, param_vel, param_radius});
      },
      "fill obstacle vector");
  obstacles.addParameter(ScalarParameter<double>("radius", param_radius),
                         {"fill obstacle vector"});
  obstacles.addParameter(ArrayParameter<Vector3d>("pos", param_pos),
                         {"fill obstacle vector"});
  obstacles.addParameter(ArrayParameter<Vector3d>("vel", param_vel),
                         {"fill obstacle vector"});
  pm.addHook([&]() -> void { obstacles_.clear(); }, "clear obstacle vector");
  pm.addParameter(ParameterManager(obstacles, false), {},
                  {"clear obstacle vector"});
  try {
    XmlRpc::XmlRpcValue params, params_reduced;
    n_.getParam("/panda_dual/dual_panda_costp_controller/bimanual_planning",
                params);
    params_reduced["obstacles"] = params["obstacles"];
    if (!pm.checkParameters(params_reduced)) {
      ROS_ERROR("Found errors in parameters, aborting.");
    } else {
      pm.loadParameters(params_reduced);
    }
  } catch (...) {
    ROS_ERROR("Something went wrong");
  }
  num_obstacles_ = obstacles_.size();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_obstacle_node");
  const double frequency = 100;
  VisionInterface VisionInterfaceObj(frequency);
  ros::Rate loop_rate(frequency);

  while (!VisionInterfaceObj.initializedObstacleMsgs()) {
    VisionInterfaceObj.initObstacles();
    loop_rate.sleep();
    if (!ros::ok()) {
      return 0;
    }
  }

  while (ros::ok()) {
    VisionInterfaceObj.updateAndPublishObstacles();
    loop_rate.sleep();
  }
  return 0;
}
