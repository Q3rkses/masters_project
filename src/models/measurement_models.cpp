#include "models/measurement_models.hpp"
#include "utilities/math_utilities.hpp"

// where each part sits in the state [x, y, u, v, psi, b_ax, b_ay, b_gyro]
namespace {
const auto velocity_slice = Eigen::seqN(2, 2);
const int heading_index = 4;

Eigen::MatrixXd position_H() {
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 8);
  H.block<2, 2>(0, 0).setIdentity();
  return H;
}
} // namespace

GNSSMeasurementModel::GNSSMeasurementModel(const GNSSConfig &config)
    : LinearMeasurementModel(ModelConfig{
          .H = position_H(),
          .R = config.position_noise,
      }) {}

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
    : velocity_noise_(config.velocity_noise) {}

Eigen::VectorXd DVLMeasurementModel::h(const Eigen::VectorXd &state,
                                       const Input &input) const {
  const double psi = state(heading_index);
  const Eigen::Vector2d velocity_nav = state(velocity_slice);
  // the DVL reports velocity along its own body-fixed beams
  return rotation_matrix_z_2D(psi).transpose() * velocity_nav;
}

Eigen::MatrixXd DVLMeasurementModel::H(const Eigen::VectorXd &state,
                                       const Input &input) const {
  const double psi = state(heading_index);
  const Eigen::Vector2d velocity_nav = state(velocity_slice);
  const Eigen::Matrix2d R_mid = rotation_matrix_z_2D(psi);
  const Eigen::Matrix2d dR_mid = derivative_rotation_matrix_z_2D(psi);

  Eigen::MatrixXd H_matrix = Eigen::MatrixXd::Zero(2, 8);
  H_matrix.block<2, 2>(0, 2) = R_mid.transpose();
  H_matrix.block<2, 1>(0, heading_index) = dR_mid.transpose() * velocity_nav;
  return H_matrix;
}

Eigen::MatrixXd DVLMeasurementModel::R(const Eigen::VectorXd &state,
                                       const Input &input) const {
  // TODO: scale with the ASV/AUV's speed instead of staying fixed
  return velocity_noise_;
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

namespace {
// where each part sits in the state [p, v, q, b_a, b_g]
const auto position_slice_3d = Eigen::seqN(0, 3);
const auto velocity_slice_3d = Eigen::seqN(3, 3);
const auto quaternion_slice = Eigen::seqN(6, 4);

Eigen::Matrix3d rotation_matrix(const Eigen::VectorXd &state) {
  return quaternion_from_vector(state(quaternion_slice)).normalized().toRotationMatrix();
}
} // namespace

GNSS3DMeasurementModel::GNSS3DMeasurementModel(const GNSS3DConfig &config)
    : position_noise_(config.position_noise) {}

Eigen::VectorXd GNSS3DMeasurementModel::h(const Eigen::VectorXd &state,
                                          const Input &input) const {
  return state(position_slice_3d);
}

Eigen::MatrixXd GNSS3DMeasurementModel::H(const Eigen::VectorXd &state,
                                          const Input &input) const {
  Eigen::MatrixXd H_matrix = Eigen::MatrixXd::Zero(3, 15);
  H_matrix.block<3, 3>(0, 0).setIdentity();
  return H_matrix;
}

Eigen::MatrixXd GNSS3DMeasurementModel::R(const Eigen::VectorXd &state,
                                          const Input &input) const {
  return position_noise_;
}

Eigen::VectorXd GNSS3DMeasurementModel::composition_plus(
    const Eigen::VectorXd &measurement, const Eigen::VectorXd &delta) const {
  return measurement + delta;
}

Eigen::VectorXd GNSS3DMeasurementModel::composition_minus(
    const Eigen::VectorXd &measurement_a,
    const Eigen::VectorXd &measurement_b) const {
  return measurement_a - measurement_b;
}

Magnetometer3DMeasurementModel::Magnetometer3DMeasurementModel(
    const Magnetometer3DConfig &config)
    : heading_noise_variance_(config.heading_noise_variance) {}

Eigen::VectorXd Magnetometer3DMeasurementModel::h(const Eigen::VectorXd &state,
                                                  const Input &input) const {
  const Eigen::Matrix3d R_nb = rotation_matrix(state);
  return Eigen::VectorXd::Constant(1, std::atan2(R_nb(1, 0), R_nb(0, 0)));
}

Eigen::MatrixXd Magnetometer3DMeasurementModel::H(const Eigen::VectorXd &state,
                                                  const Input &input) const {
  const Eigen::Matrix3d R_nb = rotation_matrix(state);
  const double first_column_norm_squared =
      R_nb(0, 0) * R_nb(0, 0) + R_nb(1, 0) * R_nb(1, 0);

  // psi = atan2(R_10, R_00), and R(q (x) Exp(dtheta)) = R (I + [dtheta]_x), so
  // dR_i0 = dtheta_z R_i1 - dtheta_y R_i2, which gives dpsi in terms of dtheta
  Eigen::MatrixXd H_matrix = Eigen::MatrixXd::Zero(1, 15);
  H_matrix(0, 7) = -(R_nb(0, 0) * R_nb(1, 2) - R_nb(1, 0) * R_nb(0, 2)) / first_column_norm_squared;
  H_matrix(0, 8) = (R_nb(0, 0) * R_nb(1, 1) - R_nb(1, 0) * R_nb(0, 1)) / first_column_norm_squared;
  return H_matrix;
}

Eigen::MatrixXd Magnetometer3DMeasurementModel::R(const Eigen::VectorXd &state,
                                                  const Input &input) const {
  return Eigen::MatrixXd::Constant(1, 1, heading_noise_variance_);
}

Eigen::VectorXd Magnetometer3DMeasurementModel::composition_plus(
    const Eigen::VectorXd &measurement, const Eigen::VectorXd &delta) const {
  Eigen::VectorXd result = measurement + delta;
  result(0) = ssa(result(0)); // the only component is the heading itself
  return result;
}

Eigen::VectorXd Magnetometer3DMeasurementModel::composition_minus(
    const Eigen::VectorXd &measurement_a,
    const Eigen::VectorXd &measurement_b) const {
  Eigen::VectorXd result = measurement_a - measurement_b;
  result(0) = ssa(result(0));
  return result;
}

DVL3DMeasurementModel::DVL3DMeasurementModel(const DVL3DConfig &config)
    : velocity_noise_(config.velocity_noise) {}

Eigen::VectorXd DVL3DMeasurementModel::h(const Eigen::VectorXd &state,
                                         const Input &input) const {
  const Eigen::Vector3d velocity_nav = state(velocity_slice_3d);
  // the DVL reports velocity along its own body-fixed beams
  return rotation_matrix(state).transpose() * velocity_nav;
}

Eigen::MatrixXd DVL3DMeasurementModel::H(const Eigen::VectorXd &state,
                                         const Input &input) const {
  const Eigen::Vector3d velocity_nav = state(velocity_slice_3d);
  const Eigen::Matrix3d R_nb = rotation_matrix(state);

  // R(q (x) Exp(dtheta))^T v = (I - [dtheta]_x) R^T v
  Eigen::MatrixXd H_matrix = Eigen::MatrixXd::Zero(3, 15);
  H_matrix.block<3, 3>(0, 3) = R_nb.transpose();
  H_matrix.block<3, 3>(0, 6) = skew(R_nb.transpose() * velocity_nav);
  return H_matrix;
}

Eigen::MatrixXd DVL3DMeasurementModel::R(const Eigen::VectorXd &state,
                                         const Input &input) const {
  // TODO: scale with the ASV/AUV's speed instead of staying fixed
  return velocity_noise_;
}

Eigen::VectorXd DVL3DMeasurementModel::composition_plus(
    const Eigen::VectorXd &measurement, const Eigen::VectorXd &delta) const {
  return measurement + delta;
}

Eigen::VectorXd DVL3DMeasurementModel::composition_minus(
    const Eigen::VectorXd &measurement_a,
    const Eigen::VectorXd &measurement_b) const {
  return measurement_a - measurement_b;
}

AltimeterMeasurementModel::AltimeterMeasurementModel(
    const AltimeterConfig &config)
    : z_noise_variance_(config.z_noise_variance) {}

Eigen::VectorXd AltimeterMeasurementModel::h(const Eigen::VectorXd &state,
                                             const Input &input) const {
  return Eigen::VectorXd::Constant(1, state(2));
}

Eigen::MatrixXd AltimeterMeasurementModel::H(const Eigen::VectorXd &state,
                                             const Input &input) const {
  return Eigen::MatrixXd(Eigen::RowVectorXd::Unit(15, 2));
}

Eigen::MatrixXd AltimeterMeasurementModel::R(const Eigen::VectorXd &state,
                                             const Input &input) const {
  return Eigen::MatrixXd::Constant(1, 1, z_noise_variance_);
}

Eigen::VectorXd AltimeterMeasurementModel::composition_plus(
    const Eigen::VectorXd &measurement, const Eigen::VectorXd &delta) const {
  return measurement + delta;
}

Eigen::VectorXd AltimeterMeasurementModel::composition_minus(
    const Eigen::VectorXd &measurement_a,
    const Eigen::VectorXd &measurement_b) const {
  return measurement_a - measurement_b;
}
