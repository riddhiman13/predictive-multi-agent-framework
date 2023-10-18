#include <bimanual_planning_ros/safe_and_fast_publisher.h>

SafeAndFastPublisher::SafeAndFastPublisher() {};

SafeAndFastPublisher& SafeAndFastPublisher::operator=(const ros::Publisher& r) {
  pub_ = r;
  return *this;
}
