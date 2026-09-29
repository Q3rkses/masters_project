#include "models/nonlinear_models.hpp"
#include "utilities.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <stdexcept>

// where each part sits in the state [x, y, psi, u, v, b_ax, b_ay, b_g]
const Eigen::ArithmeticSequence<Eigen::Index, Eigen::Index> position_slice =
    Eigen::seqN(0, 2);
const int heading_index = 2;
const Eigen::ArithmeticSequence<Eigen::Index, Eigen::Index> velocity_slice =
    Eigen::seqN(3, 2);
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
    : x_(0.0), y_(0.0), psi_(0.0), u_(0.0), v_(0.0), bias_accelerometer_x_(0.0),
      bias_accelerometer_y_(0.0), bias_gyro_psi_(0.0) {
  eigen_to_state(initial_state);
}

Eigen::VectorXd State::state_to_eigen() {
  Eigen::VectorXd state(8);
  state << x_, y_, psi_, u_, v_, bias_accelerometer_x_, bias_accelerometer_y_,
      bias_gyro_psi_;
  return state;
}

void State::eigen_to_state(Eigen::VectorXd eigen_state) {
  if (eigen_state.size() != 8) {
    throw std::invalid_argument(
        "State expects a vector of length 8: [x, y, psi, u, v, "
        "bias_accelerometer_x, bias_accelerometer_y, bias_gyro_psi]");
  }
  x_ = eigen_state(0);
  y_ = eigen_state(1);
  psi_ = eigen_state(2);
  u_ = eigen_state(3);
  v_ = eigen_state(4);
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
  double psi_next = state.psi_ + dt_ * mid.omega_tilde;
  double u_next = state.u_ + dt_ * rotated_acceleration(0);
  double v_next = state.v_ + dt_ * rotated_acceleration(1);
  double bias_accelerometer_x_next = std::exp(-dt_ / accelerometer_gauss_markov_timeconstant_(0)) * state.bias_accelerometer_x_;
  double bias_accelerometer_y_next = std::exp(-dt_ / accelerometer_gauss_markov_timeconstant_(1)) * state.bias_accelerometer_y_;
  double bias_gyro_psi_next = std::exp(-dt_ / gyro_gauss_markov_timeconstant_) * state.bias_gyro_psi_;

  Eigen::VectorXd next_state(8);
  next_state << x_next, y_next, psi_next, u_next, v_next, bias_accelerometer_x_next, bias_accelerometer_y_next, bias_gyro_psi_next;
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
  F_matrix(position_slice, heading_index) = dt_ * dt_ / 2 * mid.dRa;
  F_matrix(position_slice, velocity_slice) = dt_ * I2;
  F_matrix(position_slice, accelerometer_bias_slice) = -dt_ * dt_ / 2 * mid.R_mid;
  F_matrix(position_slice, gyro_bias_index) = -dt_ * dt_ * dt_ / 4 * mid.dRa;

  F_matrix(heading_index, heading_index) = 1.0;
  F_matrix(heading_index, gyro_bias_index) = -dt_;

  F_matrix(velocity_slice, heading_index) = dt_ * mid.dRa;
  F_matrix(velocity_slice, velocity_slice) = I2;
  F_matrix(velocity_slice, accelerometer_bias_slice) = -dt_ * mid.R_mid;
  F_matrix(velocity_slice, gyro_bias_index) = -dt_ * dt_ / 2 * mid.dRa;

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
