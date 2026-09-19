#include "models.hpp"
#include <stdexcept>

Input::Input() : a_x_(0.0), a_y_(0.0), omega_psi_(0.0) {}

Input::Input(const Eigen::VectorXd initial_input)
    : a_x_(0.0), a_y_(0.0), omega_psi_(0.0) {
  eigen_to_input(initial_input);
}

Eigen::VectorXd Input::input_to_eigen() const {
  Eigen::VectorXd input(3);
  input << a_x_, a_y_, omega_psi_;
  return input;
}

void Input::eigen_to_input(Eigen::VectorXd eigen_input) {
  if (eigen_input.size() != 3) {
    throw std::invalid_argument(
        "Input expects a vector of length 3: [a_x, a_y, omega_psi]");
  }
  a_x_ = eigen_input(0);
  a_y_ = eigen_input(1);
  omega_psi_ = eigen_input(2);
}
