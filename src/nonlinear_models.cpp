#include "nonlinear_models.hpp"
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
