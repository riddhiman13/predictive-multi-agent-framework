#include <bimanual_planning_ros/CollisionObjects.h>
#include <bimanual_planning_ros/Obstacles.h>
#include <bimanual_planning_ros/obstacle.h>
#include <bimanual_planning_ros/parameter_manager.h>
#include <bimanual_planning_ros/stl_reader.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Dense>

#include "std_msgs/Bool.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using ghostplanner::cfplanner::Obstacle;

Matrix3d r_R_B, l_R_B;
Vector3d r_r_B, r_l_B;
geometry_msgs::PoseStamped tray_pose;
bool start_obst_motion = false;

/**
 * @brief Get the MoveIt CollisionObjectMsg with operation = ADD.
 * @note Eventhough all obstacles are spheres we represent their shapes with
 * mesh files instead of using the primary shape "SPHERE", because FCL
 * (collision library) is buggy when using this primary shape.
 */
moveit_msgs::CollisionObject getAddMessage(const Obstacle& obst, const int id, const std::string& base_frame,
                                           const stl_reader::StlMesh<float, unsigned int>& unit_sphere_mesh,
                                           const Vector3d pos, bool left)
{
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
  for (size_t i = 0; i < unit_sphere_mesh.num_vrts(); ++i)
  {
    geometry_msgs::Point p;
    auto cord = unit_sphere_mesh.vrt_coords(i);
    p.x = cord[0] * obst.getRadius();
    p.y = cord[1] * obst.getRadius();
    p.z = cord[2] * obst.getRadius();
    msg.meshes[0].vertices.push_back(p);
  }

  for (size_t i = 0; i < unit_sphere_mesh.num_tris(); ++i)
  {
    shape_msgs::MeshTriangle t;
    auto inds = unit_sphere_mesh.tri_corner_inds(i);
    t.vertex_indices = { inds[0], inds[1], inds[2] };
    msg.meshes[0].triangles.push_back(t);
  }

  // default constructors init everything with zero
  geometry_msgs::Pose identity_pose;
  identity_pose.orientation.w = 1;
  msg.mesh_poses.push_back(identity_pose);

  Vector3d pos_transformed;
  if (left) {
    pos_transformed = l_R_B*pos+r_l_B;
  } else {
    pos_transformed = r_R_B*pos+r_r_B;
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

moveit_msgs::CollisionObject getAddMessage(const Obstacle& obst, const int id, const std::string& base_frame,
                                           const stl_reader::StlMesh<float, unsigned int>& unit_sphere_mesh, bool left)
{
  return getAddMessage(obst, id, base_frame, unit_sphere_mesh, obst.getPosition(), left);
}

moveit_msgs::CollisionObject getMoveMessage(const Vector3d& curr_pos, const int id, const std::string& base_frame, bool left)
{
  moveit_msgs::CollisionObject msg;
  msg.id = "CollObject_" + std::to_string(id);
  msg.operation = 3;
  msg.header.frame_id = base_frame;
  msg.header.stamp = ros::Time::now();

  Vector3d pos_transformed;
  if (left) {
    pos_transformed = l_R_B*curr_pos+r_l_B;
  } else {
    pos_transformed = r_R_B*curr_pos+r_r_B;
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
  return msg;
}

void startSignalCallback(const std_msgs::Bool& starting) {
  start_obst_motion = starting.data;
}

void poseCallback(const geometry_msgs::PoseStamped& p) { tray_pose = p; }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_obstacle_node");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<bimanual_planning_ros::Obstacles>(
      "obstacles", 1, true);
  ros::Publisher pub_markers = n.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker_array", 1, true);
  ros::Publisher moveit_obj_pub_add_l =
      n.advertise<bimanual_planning_ros::CollisionObjects>(
          "collision_objects_add_left", 1, true);
  ros::Publisher moveit_obj_pub_add_r =
      n.advertise<bimanual_planning_ros::CollisionObjects>(
          "collision_objects_add_right", 1, true);
  ros::Publisher moveit_obj_pub_move_l =
      n.advertise<bimanual_planning_ros::CollisionObjects>(
          "collision_objects_move_left", 1, true);
  ros::Publisher moveit_obj_pub_move_r =
      n.advertise<bimanual_planning_ros::CollisionObjects>(
          "collision_objects_move_right", 1, true);
  ros::Subscriber pose_sub =
      n.subscribe("/panda_dual/dual_panda_costp_controller/abs_pose", 1,
                  poseCallback, ros::TransportHints().reliable().tcpNoDelay());
  ros::Subscriber start_signal_sub = n.subscribe(
      "/panda_dual/dual_panda_costp_controller/start_obstacles", 2,
      startSignalCallback, ros::TransportHints().reliable().tcpNoDelay());

  std::string left_arm_id, right_arm_id;
  if (!n.getParam("left/arm_id", left_arm_id)) {
    ROS_ERROR_STREAM(
        "DynamicObstacleNode: Could not read parameter left_arm_id");
    return 1;
  }
  if (!n.getParam("right/arm_id", right_arm_id)) {
    ROS_ERROR_STREAM(
        "DynamicObstacleNode: Could not read parameter right_arm_id");
    return 2;
  }
  bimanual_planning_ros::CollisionObjects objects_l, objects_r;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  std::string error;
  if (!tfBuffer.canTransform("base_link", right_arm_id + "_link0",
                             ros::Time(0), ros::Duration(3), &error)) {
    ROS_ERROR("DynamicObstacleNode: Could transform from fram base_link "
              "to arm frame %s_link0. Error: %s", right_arm_id.c_str(),
              error.c_str());
  }
  if (!tfBuffer.canTransform("base_link", left_arm_id + "_link0",
                             ros::Time(0), ros::Duration(3), &error)) {
    ROS_ERROR("DynamicObstacleNode: Could transform from fram base_link "
              "to arm frame %s_link0. Error: %s", left_arm_id.c_str(),
              error.c_str());
  }
  geometry_msgs::TransformStamped transform;
  transform = tfBuffer.lookupTransform(right_arm_id + "_link0", "base_link",
                                       ros::Time(0), ros::Duration(3));
  r_r_B = Vector3d(transform.transform.translation.x,
                   transform.transform.translation.y,
                   transform.transform.translation.z);
  Quaterniond Q_r_R_B(transform.transform.rotation.w,
                      transform.transform.rotation.x,
                      transform.transform.rotation.y,
                      transform.transform.rotation.z);
  r_R_B = Q_r_R_B.toRotationMatrix();
  transform = tfBuffer.lookupTransform(left_arm_id + "_link0", "base_link",
                                       ros::Time(0), ros::Duration(3));
  r_l_B = Vector3d(transform.transform.translation.x,
                   transform.transform.translation.y,
                   transform.transform.translation.z);
  Quaterniond Q_l_R_B(transform.transform.rotation.w,
                      transform.transform.rotation.x,
                      transform.transform.rotation.y,
                      transform.transform.rotation.z);
  l_R_B = Q_l_R_B.toRotationMatrix();

  // Read sphere mesh
  const std::string mesh_file = ros::package::getPath("bimanual_planning_ros") + "/config/unit_sphere.stl";
  stl_reader::StlMesh<float, unsigned int> mesh;
  try
  {
    mesh = stl_reader::StlMesh<>(mesh_file);
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("Failed to read .stl file, e.what(): " << e.what());
    return 1;
  }

  const double frequency = 100;
  const double add_frequency = 0.5;
  ros::Rate loop_rate(frequency);

  std::vector<Obstacle> obstacles_;
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
    n.getParam("/panda_dual/dual_panda_costp_controller/bimanual_planning",
               params);
    params_reduced["obstacles"] = params["obstacles"];
    if (!pm.checkParameters(params_reduced)) {
      ROS_ERROR("Found errors in parameters, aborting.");
    } else {
      pm.loadParameters(params_reduced);
    }
  } catch (const std::exception& e) {
    ROS_ERROR(e.what());
    return 1;
  } catch (const std::string& s) {
    ROS_ERROR(s.c_str());
    return 2;
  } catch (const char* s) {
    ROS_ERROR(s);
    return 3;
  } catch (...) {
    ROS_ERROR("Something went wrong");
    return 4;
  }

  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "bimanual_planner";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  visualization_msgs::Marker tray_marker;
  tray_marker.header.frame_id = "/base_link";
  tray_marker.header.stamp = ros::Time();
  tray_marker.ns = "bimanual_planner";
  tray_marker.id = 0;
  tray_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  tray_marker.mesh_resource =
      "package://bimanual_planning_ros/vrep_scenes/tray.dae";
  tray_marker.action = visualization_msgs::Marker::ADD;
  tray_marker.lifetime = ros::Duration(0);
  tray_marker.mesh_use_embedded_materials = true;
  tray_marker.scale.x = tray_marker.scale.y = tray_marker.scale.z = 1.0;
  tray_marker.pose.orientation.x = 0.0;
  tray_marker.pose.orientation.y = 0.0;
  tray_marker.pose.orientation.z = 0.0;
  tray_marker.pose.orientation.w = 1.0;
  tray_marker.pose = tray_pose.pose;

  markers.markers.push_back(tray_marker);

  bimanual_planning_ros::Obstacles obst_msg;
  bimanual_planning_ros::Position obst_pos;
  bimanual_planning_ros::Position obst_vel;
  std::vector<Vector3d> cur_pos;
  std::vector<Vector3d> cur_vel;
  for (int i = 0; i < obstacles_.size() - 1; ++i) {
    cur_pos.push_back(obstacles_.at(i).getPosition());
    cur_vel.push_back(obstacles_.at(i).getVelocity());
    obst_pos.data[0] = marker.pose.position.x = cur_pos.at(i).x();
    obst_pos.data[1] = marker.pose.position.y = cur_pos.at(i).y();
    obst_pos.data[2] = marker.pose.position.z = cur_pos.at(i).z();
    obst_vel.data[0] = cur_vel.at(i).x();
    obst_vel.data[1] = cur_vel.at(i).y();
    obst_vel.data[2] = cur_vel.at(i).z();
    obst_msg.pos.push_back(obst_pos);
    obst_msg.vel.push_back(obst_vel);
    obst_msg.radius.push_back(obstacles_.at(i).getRadius());
    marker.scale.x = obstacles_.at(i).getRadius() * 2 - 0.05;
    marker.scale.y = obstacles_.at(i).getRadius() * 2 - 0.05;
    marker.scale.z = obstacles_.at(i).getRadius() * 2 - 0.05;
    marker.id = i + 1;
    markers.markers.push_back(marker);

    objects_l.objects.push_back(getAddMessage(obstacles_.at(i), i, "/" + left_arm_id + "_link0", mesh, true));
    objects_r.objects.push_back(getAddMessage(obstacles_.at(i), i, "/" + right_arm_id + "_link0", mesh, false));
  }

  pub.publish(obst_msg);
  pub_markers.publish(markers);
  moveit_obj_pub_add_l.publish(objects_l);
  moveit_obj_pub_add_r.publish(objects_r);

  marker.action = visualization_msgs::Marker::MODIFY;
  tray_marker.action = visualization_msgs::Marker::MODIFY;

  while (!start_obst_motion) {
    ros::spinOnce();
    usleep(1);
  }

  while (ros::ok()) {
    markers.markers.clear();
    objects_l.objects.clear();
    objects_r.objects.clear();
    for (int i = 0; i < obstacles_.size() - 1; ++i) {
      cur_pos.at(i) += cur_vel.at(i) / frequency;
      obst_msg.pos[i].data[0] = marker.pose.position.x = cur_pos.at(i).x();
      obst_msg.pos[i].data[1] = marker.pose.position.y = cur_pos.at(i).y();
      obst_msg.pos[i].data[2] = marker.pose.position.z = cur_pos.at(i).z();
      marker.scale.x = obstacles_.at(i).getRadius() * 2 - 0.05;
      marker.scale.y = obstacles_.at(i).getRadius() * 2 - 0.05;
      marker.scale.z = obstacles_.at(i).getRadius() * 2 - 0.05;
      marker.id = i + 1;
      markers.markers.push_back(marker);

      objects_l.objects.push_back(getMoveMessage(cur_pos.at(i), i, "/" + left_arm_id + "_link0", true));
      objects_r.objects.push_back(getMoveMessage(cur_pos.at(i), i, "/" + right_arm_id + "_link0", false));
    }

    tray_marker.pose = tray_pose.pose;
    tray_marker.scale.x = tray_marker.scale.y = tray_marker.scale.z = 1.0;
    tray_marker.id = 0;
    markers.markers.push_back(tray_marker);

    pub.publish(obst_msg);
    pub_markers.publish(markers);
    moveit_obj_pub_move_l.publish(objects_l);
    moveit_obj_pub_move_r.publish(objects_r);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
