#include "models/nonlinear_models.hpp"
#include "utilities/math_utilities.hpp"
#include <stdexcept>

// where each part sits in the state [x, y, u, v, psi, b_ax, b_ay, b_g]
const Eigen::ArithmeticSequence<Eigen::Index, Eigen::Index> position_slice =
    Eigen::seqN(0, 2);
const Eigen::ArithmeticSequence<Eigen::Index, Eigen::Index> velocity_slice =
    Eigen::seqN(2, 2);
const int heading_index = 4;
const Eigen::ArithmeticSequence<Eigen::Index, Eigen::Index>
    accelerometer_bias_slice = Eigen::seqN(5, 2);
const int gyro_bias_index = 7;

// where each noise source sits in the noise [n_a(2), n_g, n_ba(2), n_bg]
const Eigen::ArithmeticSequence<Eigen::Index, Eigen::Index>
    accelerometer_noise_slice = Eigen::seqN(0, 2);
const int gyro_noise_index = 2;
const Eigen::ArithmeticSequence<Eigen::Index, Eigen::Index>
    accelerometer_bias_noise_slice = Eigen::seqN(3, 2);
const int gyro_bias_noise_index = 5;

// The quantities that f, F and Q all build on
struct MidpointQuantities {
  Eigen::Vector2d a_tilde; // accelerometer reading minus its bias
  double omega_tilde;      // gyro reading minus its bias
  double psi_mid;          // heading half way through the time step
  Eigen::Matrix2d R_mid;   // R(psi_mid)
  Eigen::Matrix2d dR_mid;  // dR/dpsi evaluated at psi_mid
  Eigen::Vector2d dRa;     // dR_mid * a_tilde
};

MidpointQuantities compute_midpoint_quantities(const State &state,
                                               const Input &input,
                                               const double dt) {
  MidpointQuantities mid;
  mid.a_tilde << input.a_x_ - state.bias_accelerometer_x_,
      input.a_y_ - state.bias_accelerometer_y_;
  mid.omega_tilde = input.omega_psi_ - state.bias_gyro_psi_;
  mid.psi_mid = state.psi_ + 0.5 * dt * mid.omega_tilde;
  mid.R_mid = rotation_matrix_z_2D(mid.psi_mid);
  mid.dR_mid = derivative_rotation_matrix_z_2D(mid.psi_mid);
  mid.dRa = mid.dR_mid * mid.a_tilde;
  return mid;
}

State::State(const Eigen::VectorXd initial_state)
    : x_(0.0), y_(0.0), u_(0.0), v_(0.0), psi_(0.0), bias_accelerometer_x_(0.0),
      bias_accelerometer_y_(0.0), bias_gyro_psi_(0.0) {
  eigen_to_state(initial_state);
}

Eigen::VectorXd State::state_to_eigen() {
  Eigen::VectorXd state(8);
  state << x_, y_, u_, v_, psi_, bias_accelerometer_x_, bias_accelerometer_y_,
      bias_gyro_psi_;
  return state;
}

void State::eigen_to_state(Eigen::VectorXd eigen_state) {
  if (eigen_state.size() != 8) {
    throw std::invalid_argument(
        "State expects a vector of length 8: [x, y, u, v, psi, "
        "bias_accelerometer_x, bias_accelerometer_y, bias_gyro_psi]");
  }
  x_ = eigen_state(0);
  y_ = eigen_state(1);
  u_ = eigen_state(2);
  v_ = eigen_state(3);
  psi_ = eigen_state(4);
  bias_accelerometer_x_ = eigen_state(5);
  bias_accelerometer_y_ = eigen_state(6);
  bias_gyro_psi_ = eigen_state(7);
}

// A Gauss-Markov bias with stationary variance s2 and time constant T is driven
// by white noise of intensity 2 * s2 / T, which is the driving_noise_variance
// clang-format off
StrapdownINS2D::StrapdownINS2D(const INS2DConfig &config)
    : dt_(config.dt),
      accelerometer_white_noise_intensity_(config.accelerometer_white_noise_intensity),
      gyro_white_noise_intensity_(config.gyro_white_noise_intensity),
      accelerometer_driving_noise_variance_((2.0 * config.accelerometer_noise_variance.array() / config.accelerometer_time_constant.array()).matrix()),
      accelerometer_gauss_markov_variance_(config.accelerometer_noise_variance),
      accelerometer_gauss_markov_timeconstant_(config.accelerometer_time_constant),
      gyro_driving_noise_variance_(2.0 * config.gyro_noise_variance / config.gyro_time_constant),
      gyro_gauss_markov_variance_(config.gyro_noise_variance),
      gyro_gauss_markov_timeconstant_(config.gyro_time_constant),
      accelerometer_random_walk_variance_(config.accelerometer_random_walk_variance),
      gyro_random_walk_variance_(config.gyro_random_walk_variance) {}
// clang-format on

Eigen::VectorXd StrapdownINS2D::f(const Eigen::VectorXd &input_state,
                                  const Input &input) const {
  State state(input_state);

  // prepare variables that will be used for updating the state
  MidpointQuantities mid = compute_midpoint_quantities(state, input, dt_);

  // rotate acceleration vector with midpoint psi
  Eigen::Vector2d rotated_acceleration = mid.R_mid * mid.a_tilde;

  // update all variables according to model, one line per equation
  // clang-format off
  double x_next = state.x_ + dt_ * state.u_ + 0.5 * dt_ * dt_ * rotated_acceleration(0);
  double y_next = state.y_ + dt_ * state.v_ + 0.5 * dt_ * dt_ * rotated_acceleration(1);
  double u_next = state.u_ + dt_ * rotated_acceleration(0);
  double v_next = state.v_ + dt_ * rotated_acceleration(1);
  double psi_next = state.psi_ + dt_ * mid.omega_tilde;
  double bias_accelerometer_x_next = std::exp(-dt_ / accelerometer_gauss_markov_timeconstant_(0)) * state.bias_accelerometer_x_;
  double bias_accelerometer_y_next = std::exp(-dt_ / accelerometer_gauss_markov_timeconstant_(1)) * state.bias_accelerometer_y_;
  double bias_gyro_psi_next = std::exp(-dt_ / gyro_gauss_markov_timeconstant_) * state.bias_gyro_psi_;

  Eigen::VectorXd next_state(8);
  next_state << x_next, y_next, u_next, v_next, psi_next, bias_accelerometer_x_next, bias_accelerometer_y_next, bias_gyro_psi_next;
  // clang-format on

  return next_state;
}

Eigen::MatrixXd StrapdownINS2D::F(const Eigen::VectorXd &input_state,
                                  const Input &input) const {
  State state(input_state);

  MidpointQuantities mid = compute_midpoint_quantities(state, input, dt_);
  const Eigen::Matrix2d I2 = Eigen::Matrix2d::Identity();

  // F_d, one line per block. Row = which next-state equation, column = what it
  // is differentiated with respect to. Blocks that are not assigned stay zero
  Eigen::MatrixXd F_matrix = Eigen::MatrixXd::Zero(8, 8);
  // clang-format off
  F_matrix(position_slice, position_slice) = I2;
  F_matrix(position_slice, velocity_slice) = dt_ * I2;
  F_matrix(position_slice, heading_index) = dt_ * dt_ / 2 * mid.dRa;
  F_matrix(position_slice, accelerometer_bias_slice) = -dt_ * dt_ / 2 * mid.R_mid;
  F_matrix(position_slice, gyro_bias_index) = -dt_ * dt_ * dt_ / 4 * mid.dRa;

  F_matrix(velocity_slice, velocity_slice) = I2;
  F_matrix(velocity_slice, heading_index) = dt_ * mid.dRa;
  F_matrix(velocity_slice, accelerometer_bias_slice) = -dt_ * mid.R_mid;
  F_matrix(velocity_slice, gyro_bias_index) = -dt_ * dt_ / 2 * mid.dRa;

  F_matrix(heading_index, heading_index) = 1.0;
  F_matrix(heading_index, gyro_bias_index) = -dt_;

  Eigen::Vector2d accelerometer_bias_decay = (-dt_ / accelerometer_gauss_markov_timeconstant_.array()).exp();
  F_matrix(accelerometer_bias_slice, accelerometer_bias_slice) = accelerometer_bias_decay.asDiagonal().toDenseMatrix();
  F_matrix(gyro_bias_index, gyro_bias_index) = std::exp(-dt_ / gyro_gauss_markov_timeconstant_);
  // clang-format on

  return F_matrix;
}

Eigen::MatrixXd StrapdownINS2D::Q(const Eigen::VectorXd &input_state,
                                  const Input &input) const {
  State state(input_state);

  MidpointQuantities mid = compute_midpoint_quantities(state, input, dt_);

  const Eigen::Matrix2d I2 = Eigen::Matrix2d::Identity();

  // G, contionous time, how each noise source enters each state.
  // clang-format off
  Eigen::MatrixXd G_matrix = Eigen::MatrixXd::Zero(8, 6);
  G_matrix(velocity_slice, accelerometer_noise_slice) = mid.R_mid;
  G_matrix(heading_index, gyro_noise_index) = 1.0;
  G_matrix(accelerometer_bias_slice, accelerometer_bias_noise_slice) = I2;
  G_matrix(gyro_bias_index, gyro_bias_noise_index) = 1.0;

  // D, the continuous-time intensity of each noise source, in noise order.
  Eigen::VectorXd D_diagonal(6);
  D_diagonal(accelerometer_noise_slice) = accelerometer_white_noise_intensity_;
  D_diagonal(gyro_noise_index) = gyro_white_noise_intensity_;
  D_diagonal(accelerometer_bias_noise_slice) = accelerometer_driving_noise_variance_;
  D_diagonal(gyro_bias_noise_index) = gyro_driving_noise_variance_;

  // First order approximation of Q_d
  Eigen::MatrixXd Q_d = G_matrix * D_diagonal.asDiagonal() * G_matrix.transpose() * dt_;
  // clang-format on

  // Exact solution with Van Loan's Formula. not yet implemented

  return Q_d;
}

Eigen::VectorXd
StrapdownINS2D::composition_plus(const Eigen::VectorXd &state,
                                 const Eigen::VectorXd &delta) const {
  Eigen::VectorXd result = state + delta;
  result(heading_index) = ssa(result(heading_index));
  return result;
}

Eigen::VectorXd
StrapdownINS2D::composition_minus(const Eigen::VectorXd &state_a,
                                  const Eigen::VectorXd &state_b) const {
  Eigen::VectorXd result = state_a - state_b;
  result(heading_index) = ssa(result(heading_index));
  return result;
}

namespace {

// where each part sits in the state [p, v, q, b_a, b_g]
const auto position_slice_3d = Eigen::seqN(0, 3);
const auto velocity_slice_3d = Eigen::seqN(3, 3);
const auto quaternion_slice = Eigen::seqN(6, 4);
const auto accelerometer_bias_slice_3d = Eigen::seqN(10, 3);
const auto gyro_bias_slice_3d = Eigen::seqN(13, 3);

// where each part sits in the error state [dp, dv, dtheta, db_a, db_g], the
// one the covariance, F, Q and the deltas of the composition operators use
const auto error_position_slice = Eigen::seqN(0, 3);
const auto error_velocity_slice = Eigen::seqN(3, 3);
const auto error_attitude_slice = Eigen::seqN(6, 3);
const auto error_accelerometer_bias_slice = Eigen::seqN(9, 3);
const auto error_gyro_bias_slice = Eigen::seqN(12, 3);

// where each noise source sits in the noise [n_a, n_g, n_ba, n_bg]
const auto accelerometer_noise_slice_3d = Eigen::seqN(0, 3);
const auto gyro_noise_slice_3d = Eigen::seqN(3, 3);
const auto accelerometer_bias_noise_slice_3d = Eigen::seqN(6, 3);
const auto gyro_bias_noise_slice_3d = Eigen::seqN(9, 3);

// The quantities that f, F and Q all build on
struct MidpointQuantities3D {
  Eigen::Vector3d a_tilde;     // accelerometer reading minus its bias
  Eigen::Vector3d omega_tilde; // gyro reading minus its bias
  Eigen::Matrix3d R;           // R(q)
  Eigen::Matrix3d R_half;      // rotation of half a time step, Exp(dt/2 * omega_tilde)
  Eigen::Matrix3d R_mid;       // R(q_mid), the attitude half way through the time step
};

MidpointQuantities3D compute_midpoint_quantities(const State3D &state,
                                               const Input &input,
                                               const double dt) {
  const Eigen::VectorXd reading = input.input_to_eigen_3d();

  MidpointQuantities3D mid;
  mid.a_tilde = reading.head<3>() - state.bias_accelerometer_;
  mid.omega_tilde = reading.tail<3>() - state.bias_gyro_;
  const Eigen::Quaterniond half_step = quaternion_exp(0.5 * dt * mid.omega_tilde);
  mid.R = state.attitude_.toRotationMatrix();
  mid.R_half = half_step.toRotationMatrix();
  mid.R_mid = (state.attitude_ * half_step).toRotationMatrix();
  return mid;
}

} // namespace

State3D::State3D(const Eigen::VectorXd initial_state)
    : position_(Eigen::Vector3d::Zero()), velocity_(Eigen::Vector3d::Zero()),
      attitude_(Eigen::Quaterniond::Identity()),
      bias_accelerometer_(Eigen::Vector3d::Zero()),
      bias_gyro_(Eigen::Vector3d::Zero()) {
  eigen_to_state(initial_state);
}

Eigen::VectorXd State3D::state_to_eigen() const {
  Eigen::VectorXd state(16);
  state << position_, velocity_, quaternion_to_vector(attitude_),
      bias_accelerometer_, bias_gyro_;
  return state;
}

void State3D::eigen_to_state(Eigen::VectorXd eigen_state) {
  if (eigen_state.size() != 16) {
    throw std::invalid_argument(
        "State3D expects a vector of length 16: [p, v, q, "
        "bias_accelerometer, bias_gyro]");
  }
  position_ = eigen_state(position_slice_3d);
  velocity_ = eigen_state(velocity_slice_3d);
  attitude_ = quaternion_from_vector(eigen_state(quaternion_slice)).normalized();
  bias_accelerometer_ = eigen_state(accelerometer_bias_slice_3d);
  bias_gyro_ = eigen_state(gyro_bias_slice_3d);
}

// A Gauss-Markov bias with stationary variance s2 and time constant T is driven
// by white noise of intensity 2 * s2 / T, which is the driving_noise_variance
// clang-format off
StrapdownINS3D::StrapdownINS3D(const INS3DConfig &config)
    : dt_(config.dt),
      gravity_(config.gravity),
      accelerometer_white_noise_intensity_(config.accelerometer_white_noise_intensity),
      gyro_white_noise_intensity_(config.gyro_white_noise_intensity),
      accelerometer_driving_noise_variance_((2.0 * config.accelerometer_noise_variance.array() / config.accelerometer_time_constant.array()).matrix()),
      accelerometer_gauss_markov_variance_(config.accelerometer_noise_variance),
      accelerometer_gauss_markov_timeconstant_(config.accelerometer_time_constant),
      gyro_driving_noise_variance_((2.0 * config.gyro_noise_variance.array() / config.gyro_time_constant.array()).matrix()),
      gyro_gauss_markov_variance_(config.gyro_noise_variance),
      gyro_gauss_markov_timeconstant_(config.gyro_time_constant) {}
// clang-format on

Eigen::VectorXd StrapdownINS3D::f(const Eigen::VectorXd &input_state,
                                  const Input &input) const {
  State3D state(input_state);

  // prepare variables that will be used for updating the state
  MidpointQuantities3D mid = compute_midpoint_quantities(state, input, dt_);

  // rotate acceleration vector with the midpoint attitude, and add gravity
  Eigen::Vector3d acceleration = mid.R_mid * mid.a_tilde + gravity_;

  // update all variables according to model, one line per equation
  // clang-format off
  Eigen::Vector3d position_next = state.position_ + dt_ * state.velocity_ + 0.5 * dt_ * dt_ * acceleration;
  Eigen::Vector3d velocity_next = state.velocity_ + dt_ * acceleration;
  Eigen::Quaterniond attitude_next = (state.attitude_ * quaternion_exp(dt_ * mid.omega_tilde)).normalized();
  Eigen::Vector3d bias_accelerometer_next = (-dt_ / accelerometer_gauss_markov_timeconstant_.array()).exp() * state.bias_accelerometer_.array();
  Eigen::Vector3d bias_gyro_next = (-dt_ / gyro_gauss_markov_timeconstant_.array()).exp() * state.bias_gyro_.array();

  Eigen::VectorXd next_state(16);
  next_state << position_next, velocity_next, quaternion_to_vector(attitude_next), bias_accelerometer_next, bias_gyro_next;
  // clang-format on

  return next_state;
}

Eigen::MatrixXd StrapdownINS3D::F(const Eigen::VectorXd &input_state,
                                  const Input &input) const {
  State3D state(input_state);

  MidpointQuantities3D mid = compute_midpoint_quantities(state, input, dt_);
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

  // How the rotated acceleration R_mid * a_tilde changes with the attitude
  // error, the accelerometer bias error and the gyro bias error
  // clang-format off
  const Eigen::Matrix3d dRa_attitude = -mid.R * skew(mid.R_half * mid.a_tilde);
  const Eigen::Matrix3d dRa_accelerometer_bias = -mid.R_mid;
  const Eigen::Matrix3d dRa_gyro_bias = 0.5 * dt_ * mid.R_mid * skew(mid.a_tilde) * right_jacobian_so3(0.5 * dt_ * mid.omega_tilde);

  // the attitude error after a step is the old one rotated back by the step,
  // and the gyro bias error turns the step itself
  const Eigen::Matrix3d R_step = quaternion_exp(dt_ * mid.omega_tilde).toRotationMatrix();
  const Eigen::Matrix3d J_step = right_jacobian_so3(dt_ * mid.omega_tilde);

  // F_d, one line per block. Row = which next-state equation, column = what it
  // is differentiated with respect to. Blocks that are not assigned stay zero
  Eigen::MatrixXd F_matrix = Eigen::MatrixXd::Zero(15, 15);
  F_matrix(error_position_slice, error_position_slice) = I3;
  F_matrix(error_position_slice, error_velocity_slice) = dt_ * I3;
  F_matrix(error_position_slice, error_attitude_slice) = dt_ * dt_ / 2 * dRa_attitude;
  F_matrix(error_position_slice, error_accelerometer_bias_slice) = dt_ * dt_ / 2 * dRa_accelerometer_bias;
  F_matrix(error_position_slice, error_gyro_bias_slice) = dt_ * dt_ / 2 * dRa_gyro_bias;

  F_matrix(error_velocity_slice, error_velocity_slice) = I3;
  F_matrix(error_velocity_slice, error_attitude_slice) = dt_ * dRa_attitude;
  F_matrix(error_velocity_slice, error_accelerometer_bias_slice) = dt_ * dRa_accelerometer_bias;
  F_matrix(error_velocity_slice, error_gyro_bias_slice) = dt_ * dRa_gyro_bias;

  F_matrix(error_attitude_slice, error_attitude_slice) = R_step.transpose();
  F_matrix(error_attitude_slice, error_gyro_bias_slice) = -dt_ * J_step;

  Eigen::Vector3d accelerometer_bias_decay = (-dt_ / accelerometer_gauss_markov_timeconstant_.array()).exp();
  Eigen::Vector3d gyro_bias_decay = (-dt_ / gyro_gauss_markov_timeconstant_.array()).exp();
  F_matrix(error_accelerometer_bias_slice, error_accelerometer_bias_slice) = accelerometer_bias_decay.asDiagonal().toDenseMatrix();
  F_matrix(error_gyro_bias_slice, error_gyro_bias_slice) = gyro_bias_decay.asDiagonal().toDenseMatrix();
  // clang-format on

  return F_matrix;
}

Eigen::MatrixXd StrapdownINS3D::Q(const Eigen::VectorXd &input_state,
                                  const Input &input) const {
  State3D state(input_state);

  MidpointQuantities3D mid = compute_midpoint_quantities(state, input, dt_);

  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

  // G, contionous time, how each noise source enters each error state.
  // clang-format off
  Eigen::MatrixXd G_matrix = Eigen::MatrixXd::Zero(15, 12);
  G_matrix(error_velocity_slice, accelerometer_noise_slice_3d) = mid.R_mid;
  G_matrix(error_attitude_slice, gyro_noise_slice_3d) = I3;
  G_matrix(error_accelerometer_bias_slice, accelerometer_bias_noise_slice_3d) = I3;
  G_matrix(error_gyro_bias_slice, gyro_bias_noise_slice_3d) = I3;

  // D, the continuous-time intensity of each noise source, in noise order.
  Eigen::VectorXd D_diagonal(12);
  D_diagonal(accelerometer_noise_slice_3d) = accelerometer_white_noise_intensity_;
  D_diagonal(gyro_noise_slice_3d) = gyro_white_noise_intensity_;
  D_diagonal(accelerometer_bias_noise_slice_3d) = accelerometer_driving_noise_variance_;
  D_diagonal(gyro_bias_noise_slice_3d) = gyro_driving_noise_variance_;

  // First order approximation of Q_d
  Eigen::MatrixXd Q_d = G_matrix * D_diagonal.asDiagonal() * G_matrix.transpose() * dt_;
  // clang-format on

  // Exact solution with Van Loan's Formula. not yet implemented

  return Q_d;
}

Eigen::VectorXd
StrapdownINS3D::composition_plus(const Eigen::VectorXd &state,
                                 const Eigen::VectorXd &delta) const {
  State3D composed(state);

  // the attitude is perturbed in its own (body) frame, the rest is additive
  composed.position_ += delta(error_position_slice);
  composed.velocity_ += delta(error_velocity_slice);
  composed.attitude_ =
      (composed.attitude_ * quaternion_exp(delta(error_attitude_slice)))
          .normalized();
  composed.bias_accelerometer_ += delta(error_accelerometer_bias_slice);
  composed.bias_gyro_ += delta(error_gyro_bias_slice);
  return composed.state_to_eigen();
}

Eigen::VectorXd
StrapdownINS3D::composition_minus(const Eigen::VectorXd &state_a,
                                  const Eigen::VectorXd &state_b) const {
  State3D a(state_a);
  State3D b(state_b);

  Eigen::VectorXd result(15);
  result << a.position_ - b.position_, a.velocity_ - b.velocity_,
      quaternion_log(b.attitude_.conjugate() * a.attitude_),
      a.bias_accelerometer_ - b.bias_accelerometer_,
      a.bias_gyro_ - b.bias_gyro_;
  return result;
}
