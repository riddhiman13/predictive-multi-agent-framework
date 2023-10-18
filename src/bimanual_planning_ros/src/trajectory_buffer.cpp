#include <bimanual_planning_ros/trajectory_buffer.h>
#include <ros/console.h>

using Eigen::Vector3d;

namespace bimanual_planning_ros {

TrajectoryBuffer::TrajectoryBuffer(int size) :
    size_(size) {
  buf_.resize(size);
}

Vector3d& TrajectoryBuffer::operator[](int i) {
  return buf_[(tail_+i) % size_];
}

void TrajectoryBuffer::clear() {
  head_ = tail_;
  full_ = false;
}

bool TrajectoryBuffer::empty() {
  return !full_ && head_==tail_;
}

const Vector3d& TrajectoryBuffer::end() {
  return buf_[(head_ - 1) % size_];
}

bool TrajectoryBuffer::full() {
  return full_;
}

Vector3d TrajectoryBuffer::get() {
  int rv = tail_;
  full_ = false;
  tail_ = (tail_ + 1) % size_;
  return buf_[rv];
}

int TrajectoryBuffer::max_size() {
  return size_;
}

bool TrajectoryBuffer::put(const Vector3d& qD) {
  if (full_) {
    return false;
  }
  buf_[head_] = qD;
  head_ = (head_ + 1) % size_;
  full_ = head_==tail_;
  return true;
}

int TrajectoryBuffer::size() {
  if (full_) {
    return size_;
  }
  if (head_>=tail_) {
    return head_ - tail_;
  } else {
    return size_ + head_ - tail_;
  }
}

}
