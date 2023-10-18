// Copyright (c) 2019 Franka Emika GmbH
#pragma once

#include <Eigen/Eigen>
#include <functional>
#include <iostream>
#include <limits>

namespace panda_controllers_impl {

// TODO(qu_zh): test how frequently the eigenvalues are damped and for what operations

/**
 * Tranform a given matrix by manipulating its eigenvalues.
 * @param[in] matrix The input matrix.
 * @param[in] eigen_value_op The transform function for eigenvalues.
 * @param[in] damped Truncates minimum eigenvalues if true, false otherwise.
 * @param[in] threshold If damped is true, eigenvalues smaller than threshold will be set to
 * threshold. No effect otherwise.
 *
 * @return the transformed matrix.
 *
 * Useful link: https://en.wikipedia.org/wiki/Eigendecomposition_of_a_matrix
 *
 * Examples:
 * @verbatim
   Eigen::Matrix<double, 6, 6> mat;
   // mat <<  [initialize mat here]
   // Get inverse of the matrix (truncating small eigenvalues to 1e-3)
   auto res = eigenDecompositionOp(mat, [](double x) -> double {return 1 / x;}, true, 1e-3);
   // Get square root of the matrix (truncating small eigenvalues to 0)
   res = eigenDecompositionOp(mat, [](double x) -> double {return std::sqrt(x);}, true, 0);
   @endverbatim
 */
template <size_t dim>
inline Eigen::Matrix<double, dim, dim> eigenDecompositionOp(
    const Eigen::Matrix<double, dim, dim>& matrix,
    const std::function<double(double)>& eigen_value_op,
    bool damped = false,
    double threshold = 1e-2) {
  Eigen::EigenSolver<Eigen::Matrix<double, dim, dim>> es(matrix, true);
  Eigen::Matrix<double, dim, 1> eigenvals = es.eigenvalues().real();
  Eigen::Matrix<double, dim, 1> eigenvals_new;
  eigenvals_new.setZero();
  // Ensure positive definiteness
  for (size_t i = 0; i < dim; i++) {
    if (damped && eigenvals(i) < threshold) {
      eigenvals(i) = threshold;
    }
    eigenvals_new(i) = eigen_value_op(eigenvals(i));
  }
  // If the matrix M is symmetric, you might be tempted to use M=V*E*V' instead of M=V*E*inv(V)
  // since V' theoretically equals inv(V). DONT'T DO THAT. Due to numerical limitations sometimes
  // Eigen will treat a symmetric matrix nonsymmetric which results in an invalid eigen
  // decomposition.
  Eigen::Matrix<double, dim, dim> res =
      es.eigenvectors().real() * eigenvals_new.asDiagonal() * es.eigenvectors().real().inverse();
  if (res.array().isNaN().any()) {
    // For robot application this error is critical, and mostly cannot be handled by the program.
    // Therefore throw.

    std::cout.precision(std::numeric_limits<double>::max_digits10);
    std::cout << "matrix: " << matrix << std::endl;
    std::cout << "op: " << eigen_value_op(10) << std::endl;
    std::cout << "eigenvals: " << es.eigenvalues() << std::endl;
    std::cout << "eigenvals_new: " << eigenvals_new << std::endl;
    std::cout << "eigenvectors: " << es.eigenvectors() << std::endl;
    std::cout << "res: " << res << std::endl;
    throw std::runtime_error("The result of eigenDecompositionOp has NaN values.");
  }
  return res;
}
}  // namespace panda_controllers_impl
