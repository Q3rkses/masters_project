#include "models/measurement_models.hpp"
#include "utilities.hpp"

// where each part sits in the state [x, y, psi, u, v, b_ax, b_ay, b_gyro]
namespace {
const int x_index = 0;
const int y_index = 1;
const int heading_index = 2;
const auto velocity_slice = Eigen::seqN(3, 2);
} // namespace

MagnetometerMeasurementModel::MagnetometerMeasurementModel(
    const MagnetometerConfig &config)
    : LinearMeasurementModel(ModelConfig{
          .H = Eigen::RowVectorXd::Unit(8, heading_index),
          .R = Eigen::MatrixXd::Constant(1, 1, config.heading_noise_variance),
      }) {}

Eigen::VectorXd MagnetometerMeasurementModel::composition_plus(
    const Eigen::VectorXd &measurement, const Eigen::VectorXd &delta) const {
  Eigen::VectorXd result = measurement + delta;
  result(0) = ssa(result(0)); // the only component is the heading itself
  return result;
}

Eigen::VectorXd MagnetometerMeasurementModel::composition_minus(
    const Eigen::VectorXd &measurement_a,
    const Eigen::VectorXd &measurement_b) const {
  Eigen::VectorXd result = measurement_a - measurement_b;
  result(0) = ssa(result(0));
  return result;
}

DVLMeasurementModel::DVLMeasurementModel(const DVLConfig &config)
    : position_velocity_noise_(config.position_velocity_noise) {}

Eigen::VectorXd DVLMeasurementModel::h(const Eigen::VectorXd &state,
                                       const Input &input) const {
  const double psi = state(heading_index);
  const Eigen::Vector2d velocity_nav = state(velocity_slice);
  // the DVL reports velocity along its own body-fixed beams
  const Eigen::Vector2d velocity_body =
      rotation_matrix_z_2D(psi).transpose() * velocity_nav;

  Eigen::VectorXd measurement(4);
  measurement << state(x_index), state(y_index), velocity_body;
  return measurement;
}

Eigen::MatrixXd DVLMeasurementModel::H(const Eigen::VectorXd &state,
                                       const Input &input) const {
  const double psi = state(heading_index);
  const Eigen::Vector2d velocity_nav = state(velocity_slice);
  const Eigen::Matrix2d R_mid = rotation_matrix_z_2D(psi);
  const Eigen::Matrix2d dR_mid = derivative_rotation_matrix_z_2D(psi);

  Eigen::MatrixXd H_matrix = Eigen::MatrixXd::Zero(4, 8);
  H_matrix(0, 0) = 1.0;
  H_matrix(1, 1) = 1.0;
  H_matrix.block<2, 1>(2, heading_index) = dR_mid.transpose() * velocity_nav;
  H_matrix.block<2, 2>(2, 3) = R_mid.transpose();
  return H_matrix;
}

Eigen::MatrixXd DVLMeasurementModel::R(const Eigen::VectorXd &state,
                                       const Input &input) const {
  // TODO: scale with the ASV/AUV's speed instead of staying fixed
  return position_velocity_noise_;
}

Eigen::VectorXd
DVLMeasurementModel::composition_plus(const Eigen::VectorXd &measurement,
                                      const Eigen::VectorXd &delta) const {
  return measurement + delta;
}

Eigen::VectorXd DVLMeasurementModel::composition_minus(
    const Eigen::VectorXd &measurement_a,
    const Eigen::VectorXd &measurement_b) const {
  return measurement_a - measurement_b;
}

bool DVLMeasurementModel::is_linear() const {
  return false; // H depends on state through psi
}
