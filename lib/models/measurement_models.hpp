/**
 * @file measurement_models.hpp
 * @brief Aiding sensor measurement models beyond GNSS: a magnetometer that
 * reports heading, and a DVL that reports velocity and position.
 */

#ifndef MEASUREMENT_MODELS_HPP
#define MEASUREMENT_MODELS_HPP

#include "models/linear_models.hpp"
#include "models/models.hpp"
#include <Eigen/Dense>

struct MagnetometerConfig {
  double heading_noise_variance; // rad^2
};

/**
 * @brief A magnetometer that reports heading directly. h and H are
 * inherited unchanged from LinearMeasurementModel; only the composition
 * operators are overridden, to wrap at +-pi.
 */
class MagnetometerMeasurementModel final : public LinearMeasurementModel {
public:
  explicit MagnetometerMeasurementModel(const MagnetometerConfig &config);

  Eigen::VectorXd composition_plus(const Eigen::VectorXd &measurement,
                                   const Eigen::VectorXd &delta) const override;

  Eigen::VectorXd
  composition_minus(const Eigen::VectorXd &measurement_a,
                    const Eigen::VectorXd &measurement_b) const override;
};

struct DVLConfig {
  Eigen::MatrixXd position_velocity_noise; // R, 4x4
};

/**
 * @brief A DVL that reports position and body-frame velocity. Nonlinear,
 * since converting velocity to body frame needs the rotation matrix.
 */
class DVLMeasurementModel final : public MeasurementModel {
public:
  explicit DVLMeasurementModel(const DVLConfig &config);

  /**
   * @brief [x, y, u_body, v_body]
   */
  Eigen::VectorXd h(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Jacobian of h
   */
  Eigen::MatrixXd H(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief fixed for now
   * TODO: scale with speed instead
   */
  Eigen::MatrixXd R(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief flat vector space, plain +/-
   */
  Eigen::VectorXd composition_plus(const Eigen::VectorXd &measurement,
                                   const Eigen::VectorXd &delta) const override;

  Eigen::VectorXd
  composition_minus(const Eigen::VectorXd &measurement_a,
                    const Eigen::VectorXd &measurement_b) const override;

  bool is_linear() const override;

private:
  Eigen::MatrixXd position_velocity_noise_;
};

#endif
