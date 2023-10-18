#include <ros/ros.h>

#include <bimanual_planning_ros/joint_motion_generator.h>

void JointMotionGenerator::generateC1Trajectory(std::mutex& m, Vector14d& q_d,
    Vector14d& qD_d, const double& v, const Vector14d& q_start,
    const Vector14d& goal) {
  double v_ = 0;
  double a = v;
  double rate = 100;
  double dq_max;
  bool stopping = false;
  Vector14d dq, q = q_start;
  ros::Rate loop_rate(rate);
  {
    std::lock_guard<std::mutex> lock(m);
    q_d = q_start;
    qD_d.setZero();
  }
  while (ros::ok()) {
    v_ += a/rate;
    if (v_ > v) {
      v_ = v;
      a = 0;
    }
    dq = goal-q_d;
    dq_max = std::max(dq.maxCoeff(), std::abs(dq.minCoeff()));
    if (v_ < 0 || dq_max < 1e-6) {
      std::lock_guard<std::mutex> lock(m);
      q_d = goal;
      qD_d.setZero();
      return;
    }
    if (!stopping && dq_max < v*0.5) {
      a = -2*dq_max;
      v_ = 2*dq_max;
      stopping = true;
    }
    dq *= v_/dq_max;
    q += dq/rate;
    {
      std::lock_guard<std::mutex> lock(m);
      q_d = q;
      qD_d = dq;
    }
    loop_rate.sleep();
  }
}
