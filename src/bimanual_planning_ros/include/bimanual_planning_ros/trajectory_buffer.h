#pragma once

#include <array>
#include <vector>

#include <Eigen/Dense>

namespace bimanual_planning_ros {

class TrajectoryBuffer {
 public:
  TrajectoryBuffer() = delete;
  TrajectoryBuffer(int size);
  Eigen::Matrix<double, 3, 1>& operator[](int i);
  void clear();
  bool empty();
  const Eigen::Matrix<double, 3, 1>& end();
  bool full();
  Eigen::Matrix<double, 3, 1> get();
  int max_size();
  bool put(const Eigen::Matrix<double, 3, 1>& qD);
  int size();
 private:
  int head_ = 0, tail_ = 0;
  std::vector<Eigen::Matrix<double, 3, 1>> buf_;
  bool full_ = false;
  const int size_;
};

}
