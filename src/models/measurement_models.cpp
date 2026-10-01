#include "models/measurement_models.hpp"
#include "utilities.hpp"

// where each part sits in the state [x, y, psi, u, v, b_ax, b_ay, b_gyro]
namespace {
const int heading_index = 2;
const auto velocity_slice = Eigen::seqN(3, 2);
const int gnss_dim = 2;
const int magnetometer_dim = 1;
const int dvl_dim = 2;

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

void GNSSMeasurementModel::write_fix_csv(
    const std::filesystem::path &path, const std::vector<int> &fix_steps,
    const std::vector<Eigen::VectorXd> &measurements,
    const std::vector<Eigen::VectorXd> &innovations,
    const std::vector<Eigen::MatrixXd> &S) const {
  ::write_fix_csv(path, gnss_dim, fix_steps, measurements, innovations, S);
}

MagnetometerMeasurementModel::MagnetometerMeasurementModel(
    const MagnetometerConfig &config)
    : LinearMeasurementModel(ModelConfig{
          .H = Eigen::RowVectorXd::Unit(8, heading_index),
          .R = Eigen::MatrixXd::Constant(1, 1, config.heading_noise_variance),
      }) {}

void MagnetometerMeasurementModel::write_fix_csv(
    const std::filesystem::path &path, const std::vector<int> &fix_steps,
    const std::vector<Eigen::VectorXd> &measurements,
    const std::vector<Eigen::VectorXd> &innovations,
    const std::vector<Eigen::MatrixXd> &S) const {
  ::write_fix_csv(path, magnetometer_dim, fix_steps, measurements, innovations, S);
}

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
  H_matrix.block<2, 1>(0, heading_index) = dR_mid.transpose() * velocity_nav;
  H_matrix.block<2, 2>(0, 3) = R_mid.transpose();
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

void DVLMeasurementModel::write_fix_csv(
    const std::filesystem::path &path, const std::vector<int> &fix_steps,
    const std::vector<Eigen::VectorXd> &measurements,
    const std::vector<Eigen::VectorXd> &innovations,
    const std::vector<Eigen::MatrixXd> &S) const {
  ::write_fix_csv(path, dvl_dim, fix_steps, measurements, innovations, S);
}
