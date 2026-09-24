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
 * @brief applies the 2D rotation matrix about the z-axis
 * @param state, the state of the system
 * @return the rotated state vector
 */
inline State Rotate_Z_2D(const State &state) {

  Eigen::Vector2d initial_position{state.x_, state.y_};
  Eigen::Matrix2d rotation_matrix_z;
  rotation_matrix_z << std::cos(state.psi_), -std::sin(state.psi_),
      std::sin(state.psi_), std::cos(state.psi_);

  Eigen::Vector2d rotated_position = rotation_matrix_z * initial_position;

  State rotated_state = state;
  rotated_state.x_ = rotated_position(0);
  rotated_state.y_ = rotated_position(1);
  return rotated_state;
}

/**
 * @brief applies the 3D rotation matrix about the x-axis
 * @param state, the state of the system
 * @return the rotated state vector
 */
// inline State Rotate_X(const State &state) {
//
//   Eigen::Vector3d initial_position = Eigen::Vector3d{state.x, state.y,
//   state.z}; Eigen::Matrix3d rotation_matrix_x =
//       Eigen::Matrix3d{[1, 0, 0],
//                       [0, std::cos(state.psi), -std::sin(state.psi)],
//                       [0, std::sin(state.psi), std::cos(state.psi)]};
//
//   return initial_state * rotation_matrix_x;
// }

/**
 * @brief applies the 3D rotation matrix about the x-axis
 * @param state, the state of the system
 * @return the rotated state vector
 */
// inline State Rotate_Y(const State &state) {
//
//   Eigen::Vector3d initial_position = Eigen::Vector3d{state.x, state.y,
//   state.z}; Eigen::Matrix2d rotation_matrix_y =
//       Eigen::Matrix2d{[std::cos(state.psi), 0, std::sin(state.psi)],
//                       [0, 1, 0],
//                       [-std::sin(state.psi), 0, std::cos(state.psi)]};
//
//   return initial_position * rotation_matrix_y;
// }

/**
 * @brief applies the 3D rotation matrix about the x-axis
 * @param state, the state of the system
 * @return the rotated state vector
 */
// inline State Rotate_Z(const State &state) {
//
//   Eigen::Vector3d initial_position = Eigen::Vector3d{state.x, state.y,
//   state.z}; Eigen::Matrix2d rotation_matrix_z =
//       Eigen::Matrix2d{[std::cos(state.psi), std::sin(state.psi), 0],
//                       [-std::sin(state.psi), std::cos(state.psi), 0],
//                       [0, 0, 1]};
//
//   return initial_position * rotation_matrix_z;
// }

// TODO: If there are any functions that do not naturally fall into any existing
// category lump them in here and make this a more general utils.

#endif
