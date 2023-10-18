/**
Panda Bimanual Setup Example with Landing Controller

\author Riddhiman Laha, Juraj Vrabel, Jonathan Vorndamme, and Luis Figueredo
\since 02/2021

**/
#include <exception>
#include <fstream>
#include <stdlib.h>

#include <franka_msgs/ErrorRecoveryActionGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <bimanual_planning_ros/controller_type.h>
#include <bimanual_planning_ros/ControllerParams.h>
#include <bimanual_planning_ros/end_condition.h>
#include <bimanual_planning_ros/goal_type.h>
#include <bimanual_planning_ros/panda_bimanual_control.h>
#include <bimanual_planning_ros/JointMotion.h>

using Eigen::Vector3d;
using franka_msgs::ErrorRecoveryActionGoal;

const int MAX_PATH_LENGTH = 2000;

namespace bimanual_planning_ros {
/***************************************************************
 *     Constructor For PandaBimanual Control Class
 ***************************************************************/
PandaBimanualPlanning::PandaBimanualPlanning() : planning_active_(false),
    check_pm_("global context"), init_pm_("global context"), goal_pm_("goals"),
    reflex_tau_max_(7), reflex_F_max_(6) {
  th_min_ << -20, -20, -20, -6, -6, -6;
  th_max_ << 20, 20, 20, 6, 6, 6;
  initROSConfig();
  ParameterManager cl("controller_list");
  cl.addHook([&]()->void {
    gains_.push_back(gain_);
    controller_type_.push_back(type_);
  }, "fill gain and type vector");
  cl.addParameter(StringParameter<std::string>("type", type_),
                  {"fill gain and type vector"});
  cl.addParameter(ScalarParameter<double>("gain", gain_),
                  {"fill gain and type vector"});
  ParameterManager obstacles("obstacles");
  obstacles.addHook(
      [&]() -> void {
        obstacles_.push_back(Obstacle{obst_pos_, obst_vel_, radius_});
      },
      "fill obstacle vector");
  obstacles.addParameter(ScalarParameter<double>("radius", radius_),
                         {"fill obstacle vector"});
  obstacles.addParameter(ArrayParameter<Vector3d>("pos", obst_pos_),
                         {"fill obstacle vector"});
  obstacles.addParameter(ArrayParameter<Vector3d>("vel", obst_vel_),
                         {"fill obstacle vector"});
  goal_pm_.addHook([&]()->void {
    gains_.clear();
    controller_type_.clear();
  }, "clear gain and type vector");
  goal_pm_.addHook([&]()->void {
    ControllerParams p;
    p.velocity = velocity_;
    p.switching = switching_;
    p.low_level_gain = low_level_gain_;
    for (int i = 0; i < controller_type_.size(); ++i) {
      std_msgs::String s;
      s.data = controller_type_[i];
      p.controllers.push_back(s);
    }
    p.gains = gains_;
    controller_params_pub_.publish(p);
  }, "send controller params");
  goal_pm_.addHook([&]()->void {
    time_step_ = 1.0 / frequency_ros_;
  }, "set time step");
  goal_pm_.addHook([&]()->void {
    obstacles_.clear();
  }, "clear obstacle vector");
  goal_pm_.addHook([&]()->void {
    q_goal_ << q_goal_left_, q_goal_right_;
  }, "set q_goal");
  goal_pm_.addHook([&]()->void {
    ri_->setReflexThresholds(reflex_F_max_, reflex_tau_max_);
  }, "set reflex thresholds");
  goal_pm_.addHook([&]()->void {
    ROS_INFO("%s", message_.c_str());
  }, "print message");
  goal_pm_.addHook([&]()->void {
    std_msgs::String ei;
    ei.data = event_;
    event_pub_.publish(ei);
  }, "publish event");
  goal_pm_.addHook([&]()->void {
    ec_ = EndCondition(end_condition_);
  }, "assign end condition");
  goal_pm_.addHook([&]()->void {
    init_params_ = false;
    goal_finished_ = false;
    displayed_goal_ = false;
    if (end_condition_ == "") {
      ec_ = EndCondition::NONE;
    }
    if (message_ == "") {
      switch(GoalType(goal_type_)) {
        case GoalType::KEY: {
          ROS_INFO("Press any key to continue.");
          break;
        }
        case GoalType::GESTURE: {
          ROS_INFO("Push robot to continue.");
          break;
        }
        default:
          break;
      }
    }
    end_condition_ = "";
    message_ = "";
  }, "init goal");
  goal_pm_.addParameter(ParameterManager(cl, true), {"send controller params"},
                        {"clear gain and type vector"});
  goal_pm_.addParameter(ParameterManager(obstacles, true), {},
                        {"clear obstacle vector"});
  goal_pm_.addParameter(ScalarParameter<double>("velocity", velocity_, true),
                        {"send controller params"});
  goal_pm_.addParameter(ScalarParameter<double>("k_attr", k_attr_, true));
  goal_pm_.addParameter(ScalarParameter<double>("k_circ", k_circ_, true));
  goal_pm_.addParameter(ScalarParameter<double>("k_repel", k_repel_, true));
  goal_pm_.addParameter(ScalarParameter<double>("k_damp", k_damp_, true));
  goal_pm_.addParameter(ScalarParameter<double>("k_manip", k_manip_, true));
  goal_pm_.addParameter(
      ScalarParameter<double>("k_goal_dist", k_goal_dist_, true));
  goal_pm_.addParameter(
      ScalarParameter<double>("k_path_len", k_path_len_, true));
  goal_pm_.addParameter(
      ScalarParameter<double>("k_safe_dist", k_safe_dist_, true));
  goal_pm_.addParameter(
      ScalarParameter<double>("k_workspace", k_workspace_, true));
  goal_pm_.addParameter(ScalarParameter<size_t>("max_prediction_steps",
                                                max_prediction_steps_, true));
  goal_pm_.addParameter(
      ScalarParameter<double>("approach_dist", approach_dist_, true));
  goal_pm_.addParameter(
      ArrayParameter<Vector6d>("desired_ws_limits", des_ws_limits_, true));
  goal_pm_.addParameter(ScalarParameter<double>("k_repel_body", k_repel_body_,
                                                true));
  goal_pm_.addParameter(ScalarParameter<size_t>("num_agents_ee", num_agents_ee_,
                                                true));
  goal_pm_.addParameter(ScalarParameter<size_t>("num_agents_body",
                                                num_agents_body_, true));
  goal_pm_.addParameter(ScalarParameter<size_t>(
      "prediction_freq_multiple", prediction_freq_multiple_, true));
  goal_pm_.addParameter(
      ScalarParameter<double>("detect_shell_rad", detect_shell_rad_, true));
  goal_pm_.addParameter(ScalarParameter<double>("frequency_ros", frequency_ros_,
                                                true), {"set time step"});
  goal_pm_.addParameter(ScalarParameter<bool>("switching", switching_, true),
                                              {"send controller params"});
  goal_pm_.addParameter(ScalarParameter<double>("low_level_gain",
      low_level_gain_, true), {"send controller params"});
  goal_pm_.addParameter(ScalarParameter<bool>("open_loop", open_loop_, true));
  goal_pm_.addParameter(ScalarParameter<bool>("visualize_commanded_path",
                                              visualize_commanded_path_, true));
  goal_pm_.addParameter(ScalarParameter<bool>(
      "visualize_predicted_paths", visualize_predicted_paths_, true));
  goal_pm_.addParameter(ScalarParameter<double>("force_feedback",
                                                force_feedback_, true));
  goal_pm_.addParameter(ArrayParameter<Vector3d>("force_offset", force_offset_,
                                                 true));
  goal_pm_.addParameter(ScalarParameter<double>("force_dead_zone",
                                                force_dead_zone_, true));
  goal_pm_.addParameter(ArrayParameter<Vector7d>("q_goal_left", q_goal_left_,
                                                 true), {"set q_goal"});
  goal_pm_.addParameter(ArrayParameter<Vector7d>("q_goal_right", q_goal_right_,
                                                 true), {"set q_goal"});
  goal_pm_.addParameter(ArrayParameter<std::vector<double>>("reflex_F_max",
      reflex_F_max_, true), {"set reflex thresholds"});
  goal_pm_.addParameter(ArrayParameter<std::vector<double>>("reflex_tau_max",
      reflex_tau_max_, true), {"set reflex thresholds"});
  goal_pm_.addParameter(ArrayParameter<Vector6d>("detect_F_min", th_min_,
                                                 true));
  goal_pm_.addParameter(ArrayParameter<Vector6d>("detect_F_max", th_max_,
                                                 true));
  goal_pm_.addParameter(ScalarParameter<double>("gesture_F_th", gesture_th_,
                                                true));
  goal_pm_.addParameter(ScalarParameter<double>("gripper_width", gripper_width_,
                                                true));
  init_pm_ = ParameterManager(goal_pm_, false);
  goal_pm_.addParameter(StringParameter<std::string>("type", goal_type_, false,
      [&](const std::string& gt, const std::string& parent)->bool {
        if (GoalType(gt) == GoalType::INVALID) {
          ROS_ERROR("Unknown goal type %s of %s.", gt.c_str(), parent.c_str());
          return false;
        }
        return true;
      }), {"init goal"});
  goal_pm_.addParameter(ArrayParameter<Vector3d>("pos", last_goal_, true));
  goal_pm_.addParameter(StringParameter<std::string>("message", message_, true),
                        {"print message"});
  goal_pm_.addParameter(StringParameter<std::string>("event", event_, true),
                        {"publish event"});
  goal_pm_.addParameter(StringParameter<std::string>("end_condition",
      end_condition_, true,
      [](const std::string& ec, const std::string& parent)->bool {
        if (EndCondition(ec) ==
          EndCondition::INVALID) {
          ROS_ERROR("Invalid end_condition '%s' of %s.", ec.c_str(),
                    parent.c_str());
          return false;
        }
        return true;
      }), {"assign end condition"});
  init_pm_.insertHookFront([&]()->void {
    if (robot_type_ == "vrep") {
      ROS_INFO("Loading vrep robot");
      ri_ = std::make_unique<RobotInterface>();
    } else if (robot_type_ == "dual_panda") {
      ROS_INFO("Loading dual_panda robot");
      ri_ = std::make_unique<DualPandaInterface>();
    } else {
      ROS_WARN("Unknown robot %s. Loading no-op dummy interface.",
               robot_type_.c_str());
      ri_ = std::make_unique<RobotInterface>();
    }
  }, "init robot type");
  init_pm_.insertParameterFront(StringParameter<std::string>("robot_type",
      robot_type_), {"init robot type"});
  check_pm_ = ParameterManager(init_pm_);
  check_pm_.addParameter(ParameterManager(goal_pm_));
}
/***************************************************************
 *     Destructor For PandaBimanual Control Class
 ***************************************************************/
PandaBimanualPlanning::~PandaBimanualPlanning() {}
/***************************************************************
 *     Init Data for ROS (has the publishers, subscribers
 *     and action clients)
 ***************************************************************/
void PandaBimanualPlanning::initROSConfig() {
  std::cout << std::endl;
  ROS_INFO("[ OK ] Start node to plan and control FE Panda Bimanual Set-up");
  controller_params_pub_ =
      rosnode_.advertise<ControllerParams>("controller_params", 1, true);
  event_pub_ = rosnode_.advertise<std_msgs::String>("events", 1, true);
  goal_pub_ = rosnode_.advertise<Position>("goals", 1, true);
  dist_pub_ = rosnode_.advertise<std_msgs::Float64>("goal_distance", 1, true);
  obs_pub_ = rosnode_.advertise<bimanual_planning_ros::Obstacles>("obstacles", 1, true);
  commanded_path_pub_ =
      rosnode_.advertise<nav_msgs::Path>("commanded_path", 1000);
  predicted_paths_pub_ =
      rosnode_.advertise<visualization_msgs::Marker>("predicted_paths", 16);
  error_recovery_pub_ =
      rosnode_.advertise<ErrorRecoveryActionGoal>("error_recovery", 1, true);
  contact_wrench_sub_ = rosnode_.subscribe(
      "contact_wrench", 1, &PandaBimanualPlanning::contactWrenchCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  obstacle_sub_ = rosnode_.subscribe(
      "obstacles", 1, &PandaBimanualPlanning::obstacleCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  position_sub_ =
      rosnode_.subscribe("position", 1, &PandaBimanualPlanning::planCallback,
                         this, ros::TransportHints().reliable().tcpNoDelay());
  std::string exp_name;
  if (!rosnode_.getParam("experiment_name", exp_name)) {
    ROS_ERROR("Couldn't get experiment name from parameter server.");
    throw std::runtime_error("Couldn't get experiment name from parameter "
                             "server.");
  }
  int num_agents;
  if (!rosnode_.getParam("bimanual_planning/num_agents_ee", num_agents)) {
    ROS_ERROR("Couldn't get number of agents from parameter server.");
    throw std::runtime_error("Couldn't get number of agents from parameter "
                             "server.");
  }
}
/**************************************************************
 *     Method to move to joint goal (motion generator)
 **************************************************************/
void PandaBimanualPlanning::jointMotion(
    const Eigen::Matrix<double, 14, 1> &goal, const double &v) {
  JointMotion srv;
  for (int i = 0; i < 14; ++i) {
    srv.request.goal[i] = goal[i];
  }
  srv.request.v = v;
  ros::service::call("joint_motion", srv);
}
/***************************************************************
 *     Get Contact Wrench Information
 ***************************************************************/
void PandaBimanualPlanning::contactWrenchCallback(const ContactWrench &msg) {
  for (int i = 0; i < 6; ++i) {
    contact_wrench_[i] = msg.B_F_ext[i];
  }
}

void PandaBimanualPlanning::obstacleCallback(const Obstacles &msg) {
  for (int i = 0; i < msg.radius.size(); ++i) {
    Vector3d pos(msg.pos[i].data[0], msg.pos[i].data[1], msg.pos[i].data[2]);
    obstacles_.at(i).setPosition(pos);
    Vector3d vel(msg.vel[i].data[0], msg.vel[i].data[1], msg.vel[i].data[2]);
    obstacles_.at(i).setVelocity(vel);
  }
}

double dead_zone(const double &in, const double &dz) {
  return std::min(in + dz, std::max(in - dz, 0.0));
}

template <int i, int j>
Eigen::Matrix<double, i, j> dead_zone(const Eigen::Matrix<double, i, j> &in,
                                      const double &dz) {
  Eigen::Matrix<double, i, j> tmp;
  for (int k = 0; k < i; ++k) {
    for (int l = 0; l < j; ++l) {
      tmp(k, l) = dead_zone(in(k, l), dz);
    }
  }
  return std::move(tmp);
}
/***************************************************************
 *     Plan for the next time step
 ***************************************************************/
void PandaBimanualPlanning::planCallback(
    const bimanual_planning_ros::Position &p) {
  if (planning_active_) {
    double start_plan_timestamp = ros::Time::now().toSec();
    if (!open_loop_) {
      cf_manager_.setRealEEAgentPosition(Vector3d(p.data.data()));
    }
    cf_manager_.stopPrediction();
    int best_agent_id =
        cf_manager_.evaluateAgents(obstacles_, k_goal_dist_, k_path_len_,
                                   k_safe_dist_, k_workspace_, des_ws_limits_);
    if (visualize_predicted_paths_) {
      for (int i = 0; i < cf_manager_.getPredictedPaths().size(); i++) {
        if (cf_manager_.getPredictedPaths().at(i).size() > 2) {
          visualize_predicted_path(cf_manager_.getPredictedPaths().at(i), i,
                                   best_agent_id);
        }
      }
    }
    cf_manager_.moveRealEEAgent(obstacles_, time_step_, 1, best_agent_id);
    double end_plan_timestamp = ros::Time::now().toSec();
    cf_manager_.resetEEAgents(cf_manager_.getNextPosition(),
                              cf_manager_.getNextVelocity(), obstacles_);
    cf_manager_.startPrediction();
    bimanual_planning_ros::Position msg;
    msg.data = {cf_manager_.getNextPosition()[0],
                cf_manager_.getNextPosition()[1],
                cf_manager_.getNextPosition()[2]};
    goal_pub_.publish(msg);
    std_msgs::Float64 dist;
    dist.data = cf_manager_.getDistFromGoal();
    dist_pub_.publish(dist);
    if (visualize_commanded_path_) {
      visualize_commanded_path(cf_manager_.getPlannedTrajectory());
    }
  } else {
    ROS_INFO_STREAM("Planning not active. Setting initial position.");
    cf_manager_.setInitialPosition(Vector3d(p.data.data()));
    got_initial_pos_ = true;
  }
}
/***************************************************************
 *     Method for publishing commanded path for visualization
 ***************************************************************/
void PandaBimanualPlanning::visualize_commanded_path(
    const std::vector<Eigen::Vector3d> &poses) const {
  nav_msgs::Path commanded_path;
  commanded_path.header.frame_id = "base_link";
  commanded_path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped commanded_pose;
  for (const Eigen::Vector3d &pose : poses) {
    commanded_pose.pose.position.x = pose.x();
    commanded_pose.pose.position.y = pose.y();
    commanded_pose.pose.position.z = pose.z();
    commanded_path.poses.push_back(commanded_pose);
  }
  commanded_path_pub_.publish(commanded_path);
}
/***************************************************************
 *     Method for publishing predicted paths for visualization
 ***************************************************************/
void PandaBimanualPlanning::visualize_predicted_path(
    const std::vector<Eigen::Vector3d> &poses, int agent_num,
    int best_agent_num) const {
  visualization_msgs::Marker trace;
  trace.header.frame_id = "base_link";
  trace.header.stamp = ros::Time::now();
  trace.lifetime = ros::Duration(1.0);
  trace.scale.x = 0.01;
  trace.color.a = 1.0;
  if (agent_num == best_agent_num) {
    trace.color.r = 0.9;
    trace.color.g = 0.1;
    trace.color.b = 0.1;
  } else {
    trace.color.r = 0.0;
    trace.color.g = 0.0;
    trace.color.b = 0.9;
  }
  trace.pose.orientation.w = 1.0;
  trace.ns = "trace_agent" + std::to_string(agent_num);
  trace.id = agent_num;
  trace.action = visualization_msgs::Marker::ADD;
  trace.type = visualization_msgs::Marker::LINE_STRIP;
  int number_of_poses = poses.size();
  trace.points.reserve(number_of_poses);
  geometry_msgs::Point point_position;
  for (const auto &pose : poses) {
    point_position.x = pose.x();
    point_position.y = pose.y();
    point_position.z = pose.z();
    trace.points.push_back(point_position);
  }
  point_position.x = last_goal_.x();
  point_position.y = last_goal_.y();
  point_position.z = last_goal_.z();
  trace.points.push_back(point_position);
  predicted_paths_pub_.publish(trace);
}
/***************************************************************
 *     Method to setup task callback
 ***************************************************************/
void PandaBimanualPlanning::start() {
  try {
    XmlRpc::XmlRpcValue params;
    rosnode_.getParam("bimanual_planning", params);
    if (!check_pm_.checkParameters(params)) {
      ROS_ERROR("Found errors in parameters, aborting.");
      return;
    } else {
      init_pm_.loadParameters(params);
    }
    goals_ = params["goals"];
  } catch (const std::exception& e) {
    ROS_ERROR(e.what());
    return;
  } catch (const std::string& s) {
    ROS_ERROR(s.c_str());
    return;
  } catch (const char* s) {
    ROS_ERROR(s);
    return;
  } catch (...) {
    ROS_ERROR("Something went wrong");
    return;
  }
  ri_->init();
  std_msgs::String ei;
  ei.data = "starting";
  event_pub_.publish(ei);
  current_goal_ = 0;
  init_params_ = true;
  ros::spinOnce();
  ROS_INFO("Number of goals: %i", goals_.size());
  cf_manager_.init(last_goal_, time_step_, obstacles_,
                   std::vector<double>(num_agents_ee_, k_attr_),
                   std::vector<double>(num_agents_ee_, k_circ_),
                   std::vector<double>(num_agents_ee_, k_repel_),
                   std::vector<double>(num_agents_ee_, k_damp_),
                   std::vector<double>(num_agents_ee_, k_manip_),
                   std::vector<double>(num_agents_body_, k_repel_body_),
                   velocity_, approach_dist_, detect_shell_rad_,
                   max_prediction_steps_, prediction_freq_multiple_);
  task_timer_ = rosnode_.createTimer(ros::Duration(1.0/frequency_ros_),
      &PandaBimanualPlanning::taskCallback, this);
  task_timer_.start();
  ros::spin();
}
/***************************************************************
 *     Task callback executes all task in tasklist
 ***************************************************************/
void PandaBimanualPlanning::taskCallback(const ros::TimerEvent& e) {
  if (current_goal_ < goals_.size()) {
    if (init_params_) {
      goal_pm_.loadParameters(goals_[current_goal_]);
    }
    switch (GoalType(goal_type_)) {
      case GoalType::KEY: {
        std::cin.get();
        break;
      }
      case GoalType::GESTURE: {
        waitForGesture(gesture_th_);
        break;
      }
      case GoalType::PLAN: {
        if (!displayed_goal_) {
          displayed_goal_ = true;
          while (!got_initial_pos_) {
            ros::spinOnce();
          }
          Vector3d current_pos = cf_manager_.getNextPosition();
          cf_manager_.init(last_goal_, time_step_, obstacles_,
                           std::vector<double>(num_agents_ee_, k_attr_),
                           std::vector<double>(num_agents_ee_, k_circ_),
                           std::vector<double>(num_agents_ee_, k_repel_),
                           std::vector<double>(num_agents_ee_, k_damp_),
                           std::vector<double>(num_agents_ee_, k_manip_),
                           std::vector<double>(num_agents_body_, k_repel_body_),
                           velocity_, approach_dist_, detect_shell_rad_,
                           max_prediction_steps_, prediction_freq_multiple_);
          cf_manager_.setInitialPosition(current_pos);
          last_pos_ = Vector3d::Zero();
          ROS_INFO_STREAM("Planning towards goal:\n"
                          << cf_manager_.getGoalPosition().transpose());
          bimanual_planning_ros::Position msg;
          msg.data = {cf_manager_.getInitialPosition()[0],
                      cf_manager_.getInitialPosition()[1],
                      cf_manager_.getInitialPosition()[2] + 0.00001};
          goal_pub_.publish(msg);
        }
        planning_active_ = true;
        break;
      }
      case GoalType::GOTO: {
        got_initial_pos_ = false;
        jointMotion(q_goal_);
        break;
      }
      case GoalType::GRASP: {
        ri_->closeGrippers();
        break;
      }
      case GoalType::RELEASE: {
        ri_->openGrippers(gripper_width_);
        break;
      }
      case GoalType::ERROR_RECOVERY: {
        errorRecovery();
        break;
      }
      case GoalType::START_LOGGING: {
        startLogging();
        break;
      }
      case GoalType::STOP_LOGGING: {
        stopLogging();
        break;
      }
      default: {
        ROS_ERROR("Invalid type: %s for goal %i.", goal_type_.c_str(),
                  current_goal_);
        return;
      }
    }
    switch (ec_) {
      case EndCondition::NONE: {
        goal_finished_ = true;
        break;
      }
      case EndCondition::CONTACT: {
        if (hasContact()) {
          goal_finished_ = true;
        }
        break;
      }
      case EndCondition::REACHED: {
        if (cf_manager_.getDistFromGoal() < 0.01) {
          goal_finished_ = true;
        }
        break;
      }
      case EndCondition::ENDLESS: {
        goal_finished_ = false;
        break;
      }
      default: {
        ROS_ERROR("Invalid type: %s for end_condition %i.",
                  end_condition_.c_str(), current_goal_);
        return;
      }
    }
    if (goal_finished_) {
      init_params_ = true;
      ++current_goal_;
      planning_active_ = false;
    }
  } else {
    task_timer_.stop();
    ros::shutdown();
  }
}
/***************************************************************
 *     Method to check if the robot has detected a contact
 ***************************************************************/
bool PandaBimanualPlanning::hasContact() {
  if ((th_max_ - contact_wrench_).minCoeff() < 0 ||
      (contact_wrench_ - th_min_).minCoeff() < 0) {
    return true;
  }
  return false;
}
/***************************************************************
 *     Method set contact detection thresholds
 ***************************************************************/
void PandaBimanualPlanning::setThresholds(
    const Eigen::Matrix<double, 6, 1> &F_min,
    const Eigen::Matrix<double, 6, 1> &F_max) {
  th_min_ = F_min;
  th_max_ = F_max;
}
/***************************************************************
 *     Method to have the robot not holding anything to wait
 *     until it is pushed
 ***************************************************************/
void PandaBimanualPlanning::waitForGesture(double F_th) {
  while ((contact_wrench_).head(3).norm() < F_th) {
    ros::spinOnce();
    ros::Duration(0.001).sleep();
  }
}
/***************************************************************
 *     Method to estimate the contact point in a plane
 ***************************************************************/
Eigen::Vector3d PandaBimanualPlanning::getContactPointEstimation(
    Eigen::Vector3d n, Eigen::Vector3d p) {
  Eigen::Vector3d v = contact_wrench_.topLeftCorner(3, 1);
  if (std::abs(n.dot(v)) < 1e-6) {
    return std::move(Eigen::Vector3d(NAN, NAN, NAN));
  }
  Eigen::Matrix3d f_cross;
  f_cross.setZero();
  f_cross(0, 1) = -contact_wrench_[2];
  f_cross(0, 2) = contact_wrench_[1];
  f_cross(1, 0) = contact_wrench_[2];
  f_cross(1, 2) = -contact_wrench_[0];
  f_cross(2, 0) = -contact_wrench_[1];
  f_cross(2, 1) = contact_wrench_[0];
  Eigen::Vector3d q =
      (-f_cross).completeOrthogonalDecomposition().pseudoInverse() *
      contact_wrench_.bottomRightCorner(3, 1);
  double lambda = n.dot(p - q) / n.dot(v);
  return std::move(Eigen::Vector3d(lambda * v + q));
}

void PandaBimanualPlanning::errorRecovery() {
  error_recovery_pub_.publish(ErrorRecoveryActionGoal());
}

void PandaBimanualPlanning::startLogging() {
  ROS_INFO("No logging service defined.");
}

void PandaBimanualPlanning::stopLogging() {
  ROS_INFO("No logging service defined.");
}

}  // namespace bimanual_planning_ros
