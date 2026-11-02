/**
 * @file measurement_models.hpp
 * @brief GNSS, magnetometer and DVL measurement models of the 2D strapdown
 * INS, and the GNSS, magnetometer, DVL and altimeter models of the 3D one
 */

#ifndef MEASUREMENT_MODELS_HPP
#define MEASUREMENT_MODELS_HPP

#include "models/linear_models.hpp"
#include "models/models.hpp"
#include <Eigen/Dense>

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

private:
  Eigen::MatrixXd velocity_noise_;
};


// The 3D state is 16D, so the H of the models below is 15 columns wide, the
// Jacobian of h with respect to the error state [dp, dv, dtheta, db_a, db_g]

struct GNSS3DConfig {
  Eigen::MatrixXd position_noise; // R, 3x3
};

/**
 * @brief GNSS, reporting the 3D position directly.
 */
class GNSS3DMeasurementModel final : public MeasurementModel {
public:
  explicit GNSS3DMeasurementModel(const GNSS3DConfig &config);

  /**
   * @brief [x, y, z]
   */
  Eigen::VectorXd h(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Jacobian of h, [I 0], 3x15
   */
  Eigen::MatrixXd H(const Eigen::VectorXd &state,
                    const Input &input) const override;

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

private:
  Eigen::MatrixXd position_noise_;
};

struct Magnetometer3DConfig {
  double heading_noise_variance; // rad^2
};

/**
 * @brief A magnetometer that reports the heading (yaw) of the attitude, as
 * a tilt compensated compass does. Nonlinear in the quaternion, and the
 * composition operators wrap at +-pi.
 */
class Magnetometer3DMeasurementModel final : public MeasurementModel {
public:
  explicit Magnetometer3DMeasurementModel(const Magnetometer3DConfig &config);

  /**
   * @brief [psi], the yaw of R(q), atan2(R_10, R_00)
   */
  Eigen::VectorXd h(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Jacobian of h, 1x15, only the attitude error dtheta has entries
   */
  Eigen::MatrixXd H(const Eigen::VectorXd &state,
                    const Input &input) const override;

  Eigen::MatrixXd R(const Eigen::VectorXd &state,
                    const Input &input) const override;

  Eigen::VectorXd composition_plus(const Eigen::VectorXd &measurement,
                                   const Eigen::VectorXd &delta) const override;

  Eigen::VectorXd
  composition_minus(const Eigen::VectorXd &measurement_a,
                    const Eigen::VectorXd &measurement_b) const override;

private:
  double heading_noise_variance_;
};

struct DVL3DConfig {
  Eigen::MatrixXd velocity_noise; // R, 3x3
};

/**
 * @brief A DVL that reports body-frame velocity. Nonlinear, since
 * converting velocity to body frame needs the rotation matrix.
 */
class DVL3DMeasurementModel final : public MeasurementModel {
public:
  explicit DVL3DMeasurementModel(const DVL3DConfig &config);

  /**
   * @brief [u_body, v_body, w_body], R(q)^T v
   */
  Eigen::VectorXd h(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Jacobian of h, 3x15, [0 R^T [R^T v]_x 0 0]
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

private:
  Eigen::MatrixXd velocity_noise_;
};

struct AltimeterConfig {
  double z_noise_variance; // m^2
};

/**
 * @brief An altimeter or pressure sensor that reports the vertical position
 * z directly, linear in the state.
 */
class AltimeterMeasurementModel final : public MeasurementModel {
public:
  explicit AltimeterMeasurementModel(const AltimeterConfig &config);

  /**
   * @brief [z]
   */
  Eigen::VectorXd h(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Jacobian of h, 1x15
   */
  Eigen::MatrixXd H(const Eigen::VectorXd &state,
                    const Input &input) const override;

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

private:
  double z_noise_variance_;
};

#endif
