#include "linear_models.hpp"
#include <stdexcept>

LinearMotionModel::LinearMotionModel(const ModelConfig &model_config)
    : F_matrix_(model_config.F), Q_matrix_(model_config.Q) {
  if (F_matrix_.rows() != F_matrix_.cols() ||
      Q_matrix_.rows() != F_matrix_.rows() ||
      Q_matrix_.cols() != F_matrix_.rows()) {
    throw std::invalid_argument(
        "LinearMotionModel: F must be square (n x n) and Q must be n x n");
  }
}

Eigen::VectorXd LinearMotionModel::f(const Eigen::VectorXd &state,
                                     const Input &) const {
  return F_matrix_ * state;
}

Eigen::MatrixXd LinearMotionModel::F(const Eigen::VectorXd &,
                                     const Input &) const {
  return F_matrix_;
}

Eigen::MatrixXd LinearMotionModel::Q(const Eigen::VectorXd &,
                                     const Input &) const {
  return Q_matrix_;
}

Eigen::VectorXd
LinearMotionModel::composition_plus(const Eigen::VectorXd &state,
                                    const Eigen::VectorXd &delta) const {
  return state + delta;
}

Eigen::VectorXd
LinearMotionModel::composition_minus(const Eigen::VectorXd &state_a,
                                     const Eigen::VectorXd &state_b) const {
  return state_a - state_b;
}

LinearMeasurementModel::LinearMeasurementModel(const ModelConfig &model_config)
    : H_matrix_(model_config.H), R_matrix_(model_config.R) {
  if (R_matrix_.rows() != H_matrix_.rows() ||
      R_matrix_.cols() != H_matrix_.rows()) {
    throw std::invalid_argument(
        "LinearMeasurementModel: H must be m x n and R must be m x m");
  }
}

Eigen::VectorXd LinearMeasurementModel::h(const Eigen::VectorXd &state,
                                          const Input &) const {
  return H_matrix_ * state;
}

Eigen::MatrixXd LinearMeasurementModel::H(const Eigen::VectorXd &,
                                          const Input &) const {
  return H_matrix_;
}

Eigen::MatrixXd LinearMeasurementModel::R(const Eigen::VectorXd &,
                                          const Input &) const {
  return R_matrix_;
}

Eigen::VectorXd
LinearMeasurementModel::composition_plus(const Eigen::VectorXd &measurement,
                                         const Eigen::VectorXd &delta) const {
  return measurement + delta;
}

Eigen::VectorXd LinearMeasurementModel::composition_minus(
    const Eigen::VectorXd &measurement_a,
    const Eigen::VectorXd &measurement_b) const {
  return measurement_a - measurement_b;
}
