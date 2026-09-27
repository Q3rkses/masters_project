#include "nonlinear_models.hpp"
#include "utilities.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <stdexcept>

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

StrapdownINS2D::StrapdownINS2D(const INS2DConfig &config)
    : dt_(config.dt),
      accelerometer_gauss_markov_variance_(config.accelerometer_noise_variance),
      accelerometer_gauss_markov_timeconstant_(
          config.accelerometer_time_constant),
      gyro_gauss_markov_variance_(config.gyro_noise_variance),
      gyro_gauss_markov_timeconstant_(config.gyro_time_constant),
      accelerometer_random_walk_variance_(
          config.accelerometer_random_walk_variance),
      gyro_random_walk_variance_(config.gyro_random_walk_variance) {
  if (dt_ <= 0.0) {
    throw std::invalid_argument("StrapdownINS2D: dt must be positive");
  }
}

Eigen::VectorXd StrapdownINS2D::f(const Eigen::VectorXd &input_state,
                                  const Input &input) const {
  State state(input_state);

  // prepare variables that will be used for updating the state
  double omega_tilde = input.omega_psi_ - state.bias_gyro_psi_;
  double acceleration_x_tilde = input.a_x_ - state.bias_accelerometer_x_;
  double acceleration_y_tilde = input.a_y_ - state.bias_accelerometer_y_;
  double psi_midpoint = state.psi_ + 0.5 * dt_ * omega_tilde;

  // rotate acceleration vector with midpoint psi
  Eigen::Vector2d rotated_acceleration =
      rotate_z_2D(acceleration_x_tilde, acceleration_y_tilde, psi_midpoint);

  // update all variables according to model
  double x_next =
      state.x_ + dt_ * state.u_ + 0.5 * dt_ * dt_ * rotated_acceleration(0);
  double y_next =
      state.y_ + dt_ * state.v_ + 0.5 * dt_ * dt_ * rotated_acceleration(1);
  double psi_next = state.psi_ + dt_ * omega_tilde;
  double u_next = state.u_ + dt_ * rotated_acceleration(0);
  double v_next = state.v_ + dt_ * rotated_acceleration(1);
  double bias_accelerometer_x_next =
      exp(-dt_ / accelerometer_gauss_markov_timeconstant_(0)) *
      state.bias_accelerometer_x_;
  double bias_accelerometer_y_next =
      exp(-dt_ / accelerometer_gauss_markov_timeconstant_(1)) *
      state.bias_accelerometer_y_;
  double bias_gyro_psi_next =
      exp(-dt_ / gyro_gauss_markov_timeconstant_) * state.bias_gyro_psi_;

  Eigen::VectorXd next_state(8);
  next_state << x_next, y_next, psi_next, u_next, v_next,
      bias_accelerometer_x_next, bias_accelerometer_y_next, bias_gyro_psi_next;

  return next_state;
}

Eigen::MatrixXd StrapdownINS2D::F(const Eigen::VectorXd &input_state,
                                  const Input &input) const {
  State state(input_state);

  double omega_tilde = input.omega_psi_ - state.bias_gyro_psi_;
  double acceleration_x_tilde = input.a_x_ - state.bias_accelerometer_x_;
  double acceleration_y_tilde = input.a_y_ - state.bias_accelerometer_y_;
  double psi_midpoint = state.psi_ + 0.5 * dt_ * omega_tilde;

  Eigen::Vector2d d_rotated_acceleration = derivative_rotate_z_2D(
      acceleration_x_tilde, acceleration_y_tilde, psi_midpoint);

  // the bare rotation matrix, needed by the b_a-column entries
  Eigen::Matrix2d R;
  R << std::cos(psi_midpoint), -std::sin(psi_midpoint), std::sin(psi_midpoint),
      std::cos(psi_midpoint);

  // state order: [x, y, psi, u, v, b_ax, b_ay, b_g]
  Eigen::MatrixXd F_matrix = Eigen::MatrixXd::Zero(8, 8);

  // p_{k+1} = p_k + dt*v_k + (dt^2/2) R(psi_mid) a_tilde
  F_matrix.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();       // dp/dp
  F_matrix.block<2, 2>(0, 3) = dt_ * Eigen::Matrix2d::Identity(); // dp/dv
  F_matrix.block<2, 1>(0, 2) =
      (dt_ * dt_ / 2.0) * d_rotated_acceleration;      // dp/dpsi
  F_matrix.block<2, 2>(0, 5) = -(dt_ * dt_ / 2.0) * R; // dp/db_a
  F_matrix.block<2, 1>(0, 7) =
      -(dt_ * dt_ * dt_ / 4.0) * d_rotated_acceleration; // dp/db_g

  // v_{k+1} = v_k + dt*R(psi_mid) a_tilde
  F_matrix.block<2, 2>(3, 3) = Eigen::Matrix2d::Identity();  // dv/dv
  F_matrix.block<2, 1>(3, 2) = dt_ * d_rotated_acceleration; // dv/dpsi
  F_matrix.block<2, 2>(3, 5) = -dt_ * R;                     // dv/db_a
  F_matrix.block<2, 1>(3, 7) =
      -(dt_ * dt_ / 2.0) * d_rotated_acceleration; // dv/db_g

  // psi_{k+1} = psi_k + dt*(omega_m - b_g)
  F_matrix(2, 2) = 1.0;
  F_matrix(2, 7) = -dt_;

  // bias decay
  F_matrix(5, 5) = std::exp(-dt_ / accelerometer_gauss_markov_timeconstant_(0));
  F_matrix(6, 6) = std::exp(-dt_ / accelerometer_gauss_markov_timeconstant_(1));
  F_matrix(7, 7) = std::exp(-dt_ / gyro_gauss_markov_timeconstant_);

  return F_matrix;
}

Eigen::MatrixXd StrapdownINS2D::Q(const Eigen::VectorXd &state,
                                  const Input &input) const {
  throw std::logic_error("StrapdownINS2D::Q is not implemented yet");
}

Eigen::VectorXd
StrapdownINS2D::composition_plus(const Eigen::VectorXd &state,
                                 const Eigen::VectorXd &delta) const {
  Eigen::VectorXd result = state + delta;
  result(2) = ssa(result(2));
  return result;
}

Eigen::VectorXd
StrapdownINS2D::composition_minus(const Eigen::VectorXd &state_a,
                                  const Eigen::VectorXd &state_b) const {
  Eigen::VectorXd result = state_a - state_b;
  result(2) = ssa(result(2));
  return result;
}
