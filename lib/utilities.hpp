/**
 * @file utilities.hpp
 * @brief Contains small mathematical helpers that are shared between the
 * models, the filters and the smoothers, and that do not belong to any
 * single one of them
 */

#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <Eigen/Dense>
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

// TODO: Add rotation matrices for 2D and 3D rotations here which will later
// be utilized in the 2D and 3D strapdown INS model that i will develop

// TODO: If there are any functions that do not naturally fall into any existing
// category lump them in here and make this a more general utils.

#endif
