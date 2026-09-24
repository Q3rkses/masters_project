#include "nonlinear_models.hpp"
#include "utilities.hpp"
#include <stdexcept>

StrapdownINS2D::StrapdownINS2D(const double dt) : dt_(dt) {
  if (dt <= 0.0) {
    throw std::invalid_argument("StrapdownINS2D: dt must be positive");
  }
}

Eigen::VectorXd StrapdownINS2D::f(const Eigen::VectorXd &,
                                  const Input &) const {
  throw std::logic_error("StrapdownINS2D::f is not implemented yet");
}

Eigen::MatrixXd StrapdownINS2D::F(const Eigen::VectorXd &,
                                  const Input &) const {
  throw std::logic_error("StrapdownINS2D::F is not implemented yet");
}

Eigen::MatrixXd StrapdownINS2D::Q(const Eigen::VectorXd &,
                                  const Input &) const {
  throw std::logic_error("StrapdownINS2D::Q is not implemented yet");
}

Eigen::VectorXd
StrapdownINS2D::composition_plus(const Eigen::VectorXd &state,
                                 const Eigen::VectorXd &delta) const {
  Eigen::VectorXd result = state + delta;
  result(4) = ssa(result(4));
  return result;
}

Eigen::VectorXd
StrapdownINS2D::composition_minus(const Eigen::VectorXd &state_a,
                                  const Eigen::VectorXd &state_b) const {
  Eigen::VectorXd result = state_a - state_b;
  result(4) = ssa(result(4));
  return result;
}
