/*
   Class for obstacle information
*/
#pragma once

#include <dqrobotics/DQ.h>

#include <iostream>
#include <vector>

#include "dqrobotics/interfaces/vrep/DQ_VrepInterface.h"
#include "eigen3/Eigen/Dense"

namespace ghostplanner {
namespace cfplanner {
class Obstacle {
   private:
    std::string name_;
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    double rad_;

   public:
    Obstacle(const std::string name, const Eigen::Vector3d pos,
             const Eigen::Vector3d vel, const double rad)
        : name_{name}, pos_{pos}, vel_{vel}, rad_{rad} {};
    Obstacle(const Eigen::Vector3d pos, const double rad)
        : pos_{pos}, rad_{rad}, vel_{0, 0, 0}, name_{""} {};
    Obstacle(const Eigen::Vector3d pos, const Eigen::Vector3d vel,
             const double rad)
        : pos_{pos}, rad_{rad}, vel_{vel}, name_{""} {};
    Obstacle() : pos_{0, 0, 0}, rad_{0}, vel_{0, 0, 0}, name_{""} {};
    std::string getName() const { return name_; };
    Eigen::Vector3d getPosition() const { return pos_; };
    void setPosition(Eigen::Vector3d pos) { pos_ = pos; }
    void setVelocity(Eigen::Vector3d vel) { vel_ = vel; }
    Eigen::Vector3d getVelocity() const { return vel_; };
    double getRadius() const { return rad_; };
    void updateVrepObstacles(DQ_VrepInterface &vi, const double delta_t);
};
}  // namespace cfplanner
}  // namespace ghostplanner
