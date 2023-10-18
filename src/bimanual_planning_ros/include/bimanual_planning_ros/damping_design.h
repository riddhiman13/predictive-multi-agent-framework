// Copyright (c) 2019 Franka Emika GmbH
#pragma once

#include <bimanual_planning_ros/eigen_decomposition_op.h>

#include <Eigen/Eigen>
#include <cmath>

namespace panda_controllers_impl {

/**
 * factorization_design returns the damping matrix given stiffness, desired damping ratio and
 * inertia matrix.
 *
 * Useful links:
 * Square root of a matrix: https://en.wikipedia.org/wiki/Square_root_of_a_matrix
 *
 */
template <size_t dim>
inline Eigen::Matrix<double, dim, dim> factorizationDesign(
    const Eigen::Matrix<double, dim, dim>& stiffness,
    const Eigen::Matrix<double, dim, dim>& damping_ratio,
    const Eigen::Matrix<double, dim, dim>& inertia) {
  // TODO(qu_zh): when interpolating between two positive-definite stiffness matrix, the
  // interpolated matrix can be non-positive-definite matrix, which have negative singular values.
  // Applying square root operation on this kind of matrix, will yield NaN values. Possible
  // solutions could be interpolating only in the frame where the stiffness matrix is diagonal, or
  // interpolating singular values and vectors of source and target matrix.

  // Saturating the min value of singular values to zero is just a workaround.
  Eigen::Matrix<double, dim, dim> inertia_sqrt = eigenDecompositionOp<dim>(
      inertia, [](double x) -> double { return std::sqrt(x); }, true, 0);
  Eigen::Matrix<double, dim, dim> stiffness_sqrt = eigenDecompositionOp<dim>(
      stiffness, [](double x) -> double { return std::sqrt(x); }, true, 0);

  return inertia_sqrt * damping_ratio * stiffness_sqrt +
         stiffness_sqrt * damping_ratio * inertia_sqrt;
};

/**
 * Reference:
 * Albu-Schaffer, Alin, et al. "Cartesian impedance control of redundant robots: Recent results with
 * the DLR-light-weight-arms." 2003 IEEE International Conference on Robotics and Automation (Cat.
 * No. 03CH37422). Vol. 3. IEEE, 2003.
 */
inline Eigen::Matrix<double, 6, 6> pandaCartesianDampingDesign(
    const Eigen::Matrix<double, 6, 6>& stiffness,
    const Eigen::Matrix<double, 6, 6>& damping_ratio,
    const Eigen::Matrix<double, 6, 7>& jacobian,
    const Eigen::Matrix<double, 7, 7>& inertia,
    const Eigen::Matrix<double, 7, 1>& motor_inertias = Eigen::Matrix<double, 7, 1>(
        std::vector<double>({0.6, 0.6, 0.5, 0.5, 0.2, 0.2, 0.2}).data()),
    const Eigen::Matrix<double, 7, 1>& motor_stiffnesses = Eigen::Matrix<double, 7, 1>(
        std::vector<double>({1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 2.5}).data())) {
  // Compute Cartesian Mass inverse, previously adding motor intertia B
  Eigen::Matrix<double, 7, 7> B, K_T, I_K_T_inv;  // NOLINT (readability-identifier-naming)
  B = motor_inertias.asDiagonal();
  K_T = motor_stiffnesses.asDiagonal();
  I_K_T_inv = Eigen::Matrix<double, 7, 7>::Identity() + K_T;
  I_K_T_inv = I_K_T_inv.inverse();

  // Joint mass matrix of manipulator with flexible joints
  Eigen::Matrix<double, 7, 7> inertia_hat = inertia + I_K_T_inv * B;
  Eigen::Matrix<double, 6, 6> cartesian_mass_hat_inverse(
      (jacobian * inertia_hat.inverse() * jacobian.transpose()));

  // Cartesian mass matrix of manipulator with flexible joints
  Eigen::Matrix<double, 6, 6> cartesian_mass_hat = eigenDecompositionOp<6>(
      cartesian_mass_hat_inverse, [](double x) -> double { return 1 / x; }, true);

  return factorizationDesign<6>(stiffness, damping_ratio, cartesian_mass_hat);
}

}  // namespace panda_controllers_impl
