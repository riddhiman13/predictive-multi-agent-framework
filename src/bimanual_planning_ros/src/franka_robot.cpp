#include <bimanual_planning_ros/franka_robot.h>
#include <dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics {

DQ_SerialManipulator Panda::kinematics(double r_B_O[3], double B_Q_O[4]) {
  Matrix<double, 4, 7> franka_dh(4, 7);
  franka_dh << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // theta (q)
      0.333, 0.0, 0.316, 0.0, 0.384, 0.0,
      0.2104,  // d last = flange 0.107 + EE 0.1034
      0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088,             // a
      0.0, -pi / 2, pi / 2, pi / 2, -pi / 2, pi / 2, pi / 2;  // alpha
  DQ_SerialManipulator franka(franka_dh, "modified");
  DQ p = DQ(0.0, r_B_O[0], r_B_O[1], r_B_O[2]);
  DQ r = DQ(B_Q_O[3], B_Q_O[0], B_Q_O[1], B_Q_O[2]);
  r = r * r.inv().norm();
  std::cout << "Panda Left Reference Frame: " << r + 0.5 * E_ * p * r
            << std::endl;
  franka.set_base_frame(r + 0.5 * E_ * p * r);
  franka.set_reference_frame(r + 0.5 * E_ * p * r);
  return franka;
}

}  // namespace DQ_robotics
