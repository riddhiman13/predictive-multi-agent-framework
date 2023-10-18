#include "bimanual_planning_ros/obstacle.h"

#include "bimanual_planning_ros/cf_agent.h"

namespace ghostplanner {
namespace cfplanner {
void Obstacle::updateVrepObstacles(DQ_VrepInterface &vi, const double delta_t) {
  pos_ += delta_t * vel_;
  DQ dq_pos = pos_(0) * i_ + pos_(1) * j_ + pos_(2) * k_;
  vi.set_object_translation(name_, dq_pos);
}
}  // namespace cfplanner
}  // namespace ghostplanner
