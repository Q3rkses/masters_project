/**
 * @file measurement_models.hpp
 * @brief GNSS, magnetometer and DVL measurement models. Each knows its own
 * measurement dimension, so each writes its own fix CSV (one row per fix:
 * timestep, raw measurement, innovation, innovation covariance S).
 */

#ifndef MEASUREMENT_MODELS_HPP
#define MEASUREMENT_MODELS_HPP

#include "models/linear_models.hpp"
#include "models/models.hpp"
#include "utilities.hpp"
#include <Eigen/Dense>
#include <filesystem>
#include <vector>

struct GNSSConfig {
  Eigen::MatrixXd position_noise; // R, 2x2
};

/**
 * @brief GNSS, reporting position directly. h and H are inherited
 * unchanged from LinearMeasurementModel.
 */
class GNSSMeasurementModel final : public LinearMeasurementModel {
public:
  explicit GNSSMeasurementModel(const GNSSConfig &config);

  void write_fix_csv(const std::filesystem::path &path,
                     const std::vector<int> &fix_steps,
                     const std::vector<Eigen::VectorXd> &measurements,
                     const std::vector<Eigen::VectorXd> &innovations,
                     const std::vector<Eigen::MatrixXd> &S) const;
};

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

  void write_fix_csv(const std::filesystem::path &path,
                     const std::vector<int> &fix_steps,
                     const std::vector<Eigen::VectorXd> &measurements,
                     const std::vector<Eigen::VectorXd> &innovations,
                     const std::vector<Eigen::MatrixXd> &S) const;
};

struct DVLConfig {
  Eigen::MatrixXd velocity_noise; // R, 2x2
};

/**
 * @brief A DVL that reports body-frame velocity. Nonlinear, since
 * converting velocity to body frame needs the rotation matrix.
 */
class DVLMeasurementModel final : public MeasurementModel {
public:
  explicit DVLMeasurementModel(const DVLConfig &config);

  /**
   * @brief [u_body, v_body]
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

  void write_fix_csv(const std::filesystem::path &path,
                     const std::vector<int> &fix_steps,
                     const std::vector<Eigen::VectorXd> &measurements,
                     const std::vector<Eigen::VectorXd> &innovations,
                     const std::vector<Eigen::MatrixXd> &S) const;

private:
  Eigen::MatrixXd velocity_noise_;
};

#endif
