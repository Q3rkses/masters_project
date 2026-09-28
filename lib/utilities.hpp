/**
 * @file utilities.hpp
 * @brief Contains small mathematical helpers that are shared between the
 * models, the filters and the smoothers, and that do not belong to any
 * single one of them
 */

#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include "nonlinear_models.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>

/**
 * @brief maps an angle to its smallest signed representation, i.e. wraps it
 * into [-pi, pi].
 * @param angle, an angle in radians, of any magnitude
 * @return the equivalent angle in [-pi, pi]
 */
inline double ssa(const double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

/**
 * @brief applies ssa elementwise, for when several components of a vector
 * are angles
 * @param angles, a vector of angles in radians
 * @return a vector where every element has been wrapped into [-pi, pi]
 */
inline Eigen::VectorXd ssa(const Eigen::VectorXd &angles) {
  Eigen::VectorXd result = Eigen::VectorXd::Zero(angles.size());

  for (int i = 0; i < angles.size(); i++) {
    result(i) = ssa(angles(i));
  }

  return result;
}

/**
 * @brief the 2D rotation matrix about the z-axis.
 * @param psi, the rotation angle in radians
 * @return R_z(psi)
 */
inline Eigen::Matrix2d rotation_matrix_z_2D(const double psi) {
  Eigen::Matrix2d rotation_matrix_z;
  rotation_matrix_z << std::cos(psi), -std::sin(psi), std::sin(psi),
      std::cos(psi);

  return rotation_matrix_z;
}

/**
 * @brief the derivative of the 2D rotation matrix about the z-axis.
 * @param psi, the rotation angle in radians
 * @return dR_z(psi)
 */
inline Eigen::Matrix2d derivative_rotation_matrix_z_2D(const double psi) {
  Eigen::Matrix2d derivative_rotation_matrix_z;
  derivative_rotation_matrix_z << -std::sin(psi), -std::cos(psi), std::cos(psi),
      -std::sin(psi);

  return derivative_rotation_matrix_z;
}

// TODO: If there are any functions that do not naturally fall into any existing
// category lump them in here and make this a more general utils.

#endif
