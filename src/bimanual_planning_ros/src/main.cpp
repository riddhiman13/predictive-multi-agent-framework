#include <bimanual_planning_ros/panda_bimanual_control.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "panda_bimanual_planning");
  bimanual_planning_ros::PandaBimanualPlanning panda_bimanual_;
  panda_bimanual_.start();
  // auto [panda_bimanual, panda_l, panda_r]{initVrep()};
  return 0;
}
