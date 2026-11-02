/**
 * @file math_utilities.hpp
 * @brief Contains small mathematical helpers that are shared between the
 * models, the filters and the smoothers, and that do not belong to any
 * single one of them, including the attitude helpers of the 3D models:
 * unit quaternions (Hamilton, [w x y z]) and rotation vectors, following Sola,
 * "Quaternion kinematics for the error-state Kalman filter"
 */

#ifndef MATH_UTILITIES_HPP
#define MATH_UTILITIES_HPP

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

/**
 * @brief the cross product matrix, skew(a) * b = a x b
 * @param a, a 3D vector
 * @return [a]_x
 */
inline Eigen::Matrix3d skew(const Eigen::Vector3d &a) {
  Eigen::Matrix3d skew_matrix;
  skew_matrix << 0.0, -a(2), a(1), a(2), 0.0, -a(0), -a(1), a(0), 0.0;

  return skew_matrix;
}

/**
 * @brief unpacks a quaternion stored as [w, x, y, z] in a state vector
 * @param q, the four elements of a quaternion, real part first
 * @return the quaternion
 */
inline Eigen::Quaterniond quaternion_from_vector(const Eigen::Vector4d &q) {
  return Eigen::Quaterniond(q(0), q(1), q(2), q(3));
}

/**
 * @brief packs a quaternion as [w, x, y, z] for a state vector
 * @param q, a quaternion
 * @return the four elements of the quaternion, real part first
 */
inline Eigen::Vector4d quaternion_to_vector(const Eigen::Quaterniond &q) {
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

/**
 * @brief the exponential map from a rotation vector to a unit quaternion,
 * Exp(phi) = [cos(|phi| / 2), sin(|phi| / 2) phi / |phi|]
 * @param phi, a rotation vector, angle times axis
 * @return the unit quaternion of that rotation
 */
inline Eigen::Quaterniond quaternion_exp(const Eigen::Vector3d &phi) {
  const double angle = phi.norm();
  if (angle < 1e-9) {
    return Eigen::Quaterniond(1.0, 0.5 * phi(0), 0.5 * phi(1), 0.5 * phi(2))
        .normalized();
  }
  const Eigen::Vector3d imaginary = std::sin(0.5 * angle) / angle * phi;
  return Eigen::Quaterniond(std::cos(0.5 * angle), imaginary(0), imaginary(1),
                            imaginary(2));
}

/**
 * @brief the logarithm map from a unit quaternion to a rotation vector, the
 * inverse of quaternion_exp. q and -q are the same rotation, the result is
 * always the shorter one, with an angle in [0, pi].
 * @param q, a unit quaternion
 * @return the rotation vector, angle times axis
 */
inline Eigen::Vector3d quaternion_log(const Eigen::Quaterniond &q) {
  double w = q.w();
  Eigen::Vector3d imaginary = q.vec();
  if (w < 0.0) {
    w = -w;
    imaginary = -imaginary;
  }

  const double sin_half_angle = imaginary.norm();
  if (sin_half_angle < 1e-9) {
    return 2.0 * imaginary / w;
  }
  return 2.0 * std::atan2(sin_half_angle, w) / sin_half_angle * imaginary;
}

/**
 * @brief the right Jacobian of SO(3), which relates a small change of a
 * rotation vector to the change of the rotation in the local frame,
 * Exp(phi + d) = Exp(phi) Exp(Jr(phi) d)
 * @param phi, a rotation vector, angle times axis
 * @return Jr(phi)
 */
inline Eigen::Matrix3d right_jacobian_so3(const Eigen::Vector3d &phi) {
  const double angle = phi.norm();
  const Eigen::Matrix3d phi_skew = skew(phi);

  // (1 - cos) / angle^2 and (angle - sin) / angle^3, as series for tiny angles
  double a = 0.5 - angle * angle / 24.0;
  double b = 1.0 / 6.0 - angle * angle / 120.0;
  if (angle > 1e-4) {
    a = (1.0 - std::cos(angle)) / (angle * angle);
    b = (angle - std::sin(angle)) / (angle * angle * angle);
  }

  return Eigen::Matrix3d::Identity() - a * phi_skew + b * phi_skew * phi_skew;
}

/**
 * @brief the attitude of the zyx Euler angles, R = R_z(psi) R_y(theta) R_x(phi)
 * @param phi, roll in radians
 * @param theta, pitch in radians
 * @param psi, yaw in radians
 * @return the unit quaternion, body to navigation frame
 */
inline Eigen::Quaterniond quaternion_from_euler(const double phi,
                                                const double theta,
                                                const double psi) {
  return Eigen::Quaterniond(
      Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()));
}

#endif
