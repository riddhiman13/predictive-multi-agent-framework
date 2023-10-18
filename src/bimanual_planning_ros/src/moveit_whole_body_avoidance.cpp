#include <bimanual_planning_ros/moveit_whole_body_avoidance.h>

#include <cmath>

#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <tf2_ros/transform_listener.h>

#include <bimanual_planning_ros/cf_agent.h>

using namespace bimanual_planning_ros;

using DQ_robotics::pinv;

namespace {

// overload for whole body avoidance
MatrixXd geomJ(const DQ_SerialManipulator &robot, const MatrixXd &poseJacobian,
                                               const VectorXd &q, const int n) {
  Matrix<double, 8, 1> v;
  Matrix<double, 3, 4> CJ4_2_J3;
  Matrix<double, 6, 7> J;
  v << -1, 1, 1, 1, -1, 1, 1, 1;
  MatrixXd C8 = v.array().matrix().asDiagonal();
  MatrixXd C4m = -C8.block(0, 0, 4, 4);
  CJ4_2_J3 << 0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
  DQ xm = robot.fkm(q,n-1);
  J.block(0, 0, 3, n) = CJ4_2_J3 * 2 * xm.P().conj().haminus4() * poseJacobian.topRows(4);
  J.block(3, 0, 3, n) = CJ4_2_J3 * 2 * (xm.D().hamiplus4() * C4m * poseJacobian.topRows(4)
                              + xm.P().conj().haminus4() * poseJacobian.middleRows(4, 4));
  return J.block(0, 0, 6, n);
}

// TODO MAKE THESE FUNCTIONS PRIVATE MEMBER FUNCTIONS
Vector3d getLinkPosition(const VectorXd &q, const int link,
                         const DQ_SerialManipulator &robot,
                         const Vector3d offset) {
  DQ link_fkm { robot.fkm(q, link-1) };
  DQ dq_offset { 0, offset(0), offset(1), offset(2) };
  DQ offset_world_frame { link_fkm.P() * dq_offset * link_fkm.P().conj() };
  return (link_fkm.translation() + offset_world_frame).q.segment(1, 3);
}

std::vector<Vector3d> getLinkForce(
    ghostplanner::cfplanner::CfAgent &agent,
    const std::vector<Eigen::Vector3d> &link_positions,
    const std::vector<ghostplanner::cfplanner::Obstacle> &obstacles,
    const double gain) {
  std::vector<Vector3d> forces;
  for (size_t i = 0; i < link_positions.size(); ++i) {
    agent.setPosition(link_positions[i]);
    Vector3d force = agent.bodyForce(obstacles, gain);
    forces.push_back(force);
  }
  return forces;
};

void collisionJ(const Vector3d &agent_pos, const Vector3d &force,
    const std::pair<int, Vector3d> link_offset,
    const DQ_SerialManipulator &robot, const VectorXd &q,
    MatrixXd& Jac, Vector7d& err) {
  Jac = MatrixXd::Zero(3, 7);
  err = MatrixXd::Zero(7, 1);
  if (force.norm() == 0) {
    return;
  }
  int link = link_offset.first;
  Vector3d offset = link_offset.second;                          // TODO: this is the offset that has to be added in the two lines below
  DQ p = DQ(0.0, agent_pos(0), agent_pos(1), agent_pos(2));
  DQ r = DQ(1);
  r = r * r.inv().norm();
  DQ fk_point = r + 0.5 * E_ * p * r;
  DQ link_fkm = robot.fkm(q, link);
  DQ link_fkm_joint = link_fkm.inv() * fk_point;
  DQ dq_offset { 0, offset(0), offset(1), offset(2) };
  MatrixXd link_joint_jac = robot.pose_jacobian(q, link - 1);       // TODO: offset from the joint frame needs to be added here
  MatrixXd link_jac = haminus8(link_fkm_joint) * link_joint_jac;
  MatrixXd link_geomJ = geomJ(robot, link_jac, q, link);            // TODO: and probably also here (offset adding)
  Jac = link_geomJ.middleRows(3, 3);
  MatrixXd Jac_inv { Jac.transpose() * pinv(Jac * Jac.transpose() +
                              0.001 * MatrixXd::Identity(3, 3)) };
  err = Jac_inv * force * 0.001;                                    // TODO magic number, this is the time step
}

}

MoveitWholeBodyAvoidance::~MoveitWholeBodyAvoidance() {
  if (calculation_thread_.joinable()) {
    calculation_thread_.join();
  }
}

void MoveitWholeBodyAvoidance::addObstaclesLeft(
    const std::vector<moveit_msgs::CollisionObject>& msg) {
  std::lock_guard<std::recursive_mutex> lock(obs_mutex_l_);
  obstacles_l_ = msg;
  update_obstacles_l_ = true;
}

void MoveitWholeBodyAvoidance::addObstaclesRight(
    const std::vector<moveit_msgs::CollisionObject>& msg) {
  std::lock_guard<std::recursive_mutex> lock(obs_mutex_r_);
  obstacles_r_ = msg;
  update_obstacles_r_ = true;
}

void MoveitWholeBodyAvoidance::init(const Eigen::Isometry3d& base_tf_l,
                                    const Eigen::Isometry3d& base_tf_r) {
  base_tf_l_ = base_tf_l;
  base_tf_r_ = base_tf_r;
  calculation_thread_ = std::thread(
      &MoveitWholeBodyAvoidance::calculationThreadFcn, this);
}

void MoveitWholeBodyAvoidance::moveObstaclesLeft(
    const std::vector<moveit_msgs::CollisionObject>& msg) {
  std::lock_guard<std::recursive_mutex> lock(obs_mutex_l_);
  if (has_obstacles_) {
    obstacles_l_ = msg;
    update_obstacles_l_ = true;
  }
}

void MoveitWholeBodyAvoidance::moveObstaclesRight(
    const std::vector<moveit_msgs::CollisionObject>& msg) {
  std::lock_guard<std::recursive_mutex> lock(obs_mutex_r_);
  if (has_obstacles_) {
    obstacles_r_ = msg;
    update_obstacles_r_ = true;
  }
}

void MoveitWholeBodyAvoidance::wholeBodyCollisionAvoidance(MatrixXd& Jac,
    Vector14d& err, const Vector14d& q) {
  {
    std::lock_guard<std::recursive_mutex> lock(q_mutex_);
    q_ = q;
  }
  std::lock_guard<std::recursive_mutex> lock(data_mutex_);
  Jac = Jac_;
  err = err_;
}

void MoveitWholeBodyAvoidance::calculationThreadFcn() {
  Vector14d err;
  Eigen::Matrix<double,6,14> Jac;
  std::vector<double> q_l, q_r;
  q_l.resize(14);
  q_r.resize(14);
  ros::NodeHandle nh;
  ros::Publisher pub_l, pub_r;
  pub_l = nh.advertise<moveit_msgs::PlanningScene>("scene_l",1);
  pub_r = nh.advertise<moveit_msgs::PlanningScene>("scene_r",1);
  moveit_msgs::PlanningScene scene_msg;
  robot_model_loader::RobotModelLoader robot_model_loader_l(
      "panda_left/robot_description");
  robot_model::RobotModelPtr robot_model_l = robot_model_loader_l.getModel();
  planning_scene::PlanningScene planning_scene_l(robot_model_l);
  robot_state::RobotState& robot_state_l =
      planning_scene_l.getCurrentStateNonConst();
  robot_model_loader::RobotModelLoader robot_model_loader_r(
      "panda_right/robot_description");
  robot_model::RobotModelPtr robot_model_r = robot_model_loader_r.getModel();
  planning_scene::PlanningScene planning_scene_r(robot_model_r);
  robot_state::RobotState& robot_state_r =
      planning_scene_r.getCurrentStateNonConst();
  // Interface for publishing markers/collision objects/... to rviz
  visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(
      "base_link", "collision_avoidance_forces", robot_model_l);
  {
    bool got_objects = false;
    while (!got_objects && ros::ok()) {
      {
        std::scoped_lock lock(obs_mutex_l_, obs_mutex_r_);
        got_objects = update_obstacles_l_ && update_obstacles_r_;
      }
      ros::spinOnce();
    }
    std::scoped_lock lock(obs_mutex_l_, obs_mutex_r_);
    for (const moveit_msgs::CollisionObject& msg : obstacles_l_) {
      planning_scene_l.processCollisionObjectMsg(msg);
    }
    for (const moveit_msgs::CollisionObject& msg : obstacles_r_) {
      planning_scene_r.processCollisionObjectMsg(msg);
    }
    has_obstacles_ = true;
  }
  while (ros::ok()) {
    Jac.setZero();
    err.setZero();
    visual_tools_->deleteAllMarkers();
    next_marker_id_ = 0;
    {
      std::lock_guard<std::recursive_mutex> lock(q_mutex_);
      for (int i=0;i<7;++i) {
        q_l[i] = q_[i];
        q_r[i] = q_[i+7];
      }
    }
    {
      std::lock_guard<std::recursive_mutex> lock(obs_mutex_l_);
      if (update_obstacles_l_) {
        update_obstacles_l_ = false;
        for (const moveit_msgs::CollisionObject& msg : obstacles_l_) {
          planning_scene_l.processCollisionObjectMsg(msg);
        }
      }
    }
    {
      std::lock_guard<std::recursive_mutex> lock(obs_mutex_r_);
      if (update_obstacles_r_) {
        update_obstacles_r_ = false;
        for (const moveit_msgs::CollisionObject& msg : obstacles_r_) {
          planning_scene_r.processCollisionObjectMsg(msg);
        }
      }
    }
    robot_state_l.setVariablePositions(q_l);
    robot_state_r.setVariablePositions(q_r);
    robot_state_l.updateCollisionBodyTransforms();
    robot_state_r.updateCollisionBodyTransforms();
    err.head(7) = wholeBodyCollisionAvoidanceTorques(planning_scene_l,
        "panda_left_arm", true, Jac.topLeftCorner<3, 7>());
    err.tail(7) = wholeBodyCollisionAvoidanceTorques(planning_scene_r,
        "panda_right_arm", false, Jac.bottomRightCorner<3, 7>());
    visual_tools_->trigger();
    planning_scene_l.getPlanningSceneMsg(scene_msg);
    pub_l.publish(scene_msg);
    planning_scene_r.getPlanningSceneMsg(scene_msg);
    pub_r.publish(scene_msg);
    std::lock_guard<std::recursive_mutex> lock(data_mutex_);
    Jac_ = Jac;
    err_ = err;
  }
}

Vector7d MoveitWholeBodyAvoidance::wholeBodyCollisionAvoidanceTorques(
    const planning_scene::PlanningScene& scene,
    const std::string &planning_group, bool left,
    Eigen::Ref<Eigen::Matrix<double, 3, 7>> Jac) {
  Eigen::Isometry3d base_tf;
  if (left) {
    base_tf = base_tf_l_;
  } else {
    base_tf = base_tf_r_;
  }
  moveit::core::RobotState robot_state = scene.getCurrentState();
  moveit::core::RobotModelConstPtr robot_model = scene.getRobotModel();
  collision_detection::DistanceRequest request;
  collision_detection::DistanceResult result;
  request.enable_nearest_points = true;
  request.group_name = planning_group;
  request.max_contacts_per_body = 1;
  // Only works if robot collision geometry does not include any spheres (FCL bug)
  request.type = collision_detection::DistanceRequestTypes::SINGLE;
  request.distance_threshold = this->wb_avoidance_max_distance_ + 0.2;
  request.enableGroup(robot_model);
  Vector7d tau_avoidance = Vector7d::Zero();
  scene.getCollisionEnv()->distanceRobot(request, result, robot_state);
  for (const auto& res : result.distances) {
    double response = avoidanceResponse(res.second[0].distance, this->distance_0_);
    if (response > 0){
      // second body should always be of type robot link
      if (res.second[0].body_types[1] != collision_detection::BodyTypes::ROBOT_LINK)
        ROS_WARN("Body type not robot link");
      bool has_link;
      auto link = robot_model->getLinkModel(res.second[0].link_names[1], &has_link);
      if (!has_link){
        ROS_WARN_STREAM("Link '" << res.second[0].link_names[1] << "' not found in robot model");
        continue;
      }
      // jacobian
      Eigen::MatrixXd ja;
      Eigen::Vector3d attack_point =
          robot_state.getGlobalLinkTransform(link).inverse() * res.second[0].nearest_points[1];
      robot_state.getJacobian(robot_state.getJointModelGroup(planning_group), link, attack_point, ja);
      ja = ja.topRows(3); // only need translational jacobian

      // return jacobian for (later) nullspace projection
      Jac += ja;

      // artificial force
      Eigen::Vector3d force = response * res.second[0].normal;

      // convert force to joint torques
      tau_avoidance += ja.transpose() * force;

      this->visualiseForce(res.second[0].nearest_points[0], force, base_tf);
    }
  }
  return tau_avoidance;
}

void MoveitWholeBodyAvoidance::setWholeBodyDistanceThresh(double distance_0) {
  this->distance_0_ = distance_0;
}

void MoveitWholeBodyAvoidance::visualiseForce(
    const Eigen::Vector3d& start_point, const Eigen::Vector3d& force,
    const Eigen::Isometry3d tf) {
  visual_tools_->publishArrow(
      visual_tools_->convertPoint(tf.inverse() * start_point),
      visual_tools_->convertPoint(tf.inverse() * (start_point + force)),
      rviz_visual_tools::BLUE, rviz_visual_tools::scales::XSMALL,
      next_marker_id_++);
}

/**
 * @brief Potential field gradient function.
 *
 * @param distance
 * @param distance_0 Maximum distance where response != 0
 * @return double
 */
double MoveitWholeBodyAvoidance::avoidanceResponse(const double& distance, const double& distance_0) {
  constexpr double n = 3 * 0.02;
  if (distance > distance_0)
    return 0;
  else
    return n * (1 / distance - 1 / distance_0) * pow(distance, -2);
}
