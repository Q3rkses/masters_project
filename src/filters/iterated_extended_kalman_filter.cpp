#include "filters/iterated_extended_kalman_filter.hpp"
#include "filters/bayesian_filter.hpp"
#include "models/models.hpp"
#include <Eigen/Dense>
#include <stdexcept>

IteratedExtendedKalmanFilter::IteratedExtendedKalmanFilter(
    const FilterConfig &filter_config,
    const LevenbergMarquardtConfig &lm_config)
    : motion_model_(filter_config.motion_model) {
  if (!motion_model_) {
    throw std::invalid_argument(
        "IteratedExtendedKalmanFilter: the filter config needs a motion model");
  }
};

FilterPredict
IteratedExtendedKalmanFilter::predict(const Eigen::VectorXd &x_current,
                                      const Eigen::MatrixXd &P_current,
                                      const Input &input) {

  Eigen::MatrixXd F = motion_model_->F(x_current, input);
  Eigen::VectorXd x_predicted = motion_model_->f(x_current, input);
  Eigen::MatrixXd P_predicted =
      F * P_current * F.transpose() + motion_model_->Q(x_current, input);
  FilterPredict filter_prediction{x_predicted, P_predicted};
  return filter_prediction;
}

FilterUpdate IteratedExtendedKalmanFilter::update(
    const Eigen::VectorXd &x_predicted, const Eigen::VectorXd &z_current,
    const Eigen::MatrixXd &P_predicted, const Input &input,
    const std::shared_ptr<const MeasurementModel> &measurement_model) {
  if (!measurement_model) {
    throw std::invalid_argument(
        "IteratedExtendedKalmanFilter::update: measurement model is null");
  }

  Eigen::MatrixXd H = measurement_model->H(x_predicted, input);
  Eigen::MatrixXd R = measurement_model->R(x_predicted, input);

  Eigen::VectorXd z_predicted = measurement_model->h(x_predicted, input);

  // use the proper compostion operation, rather than regular +, - operations
  Eigen::VectorXd innovation =
      measurement_model->composition_minus(z_current, z_predicted);

  Eigen::MatrixXd PHt = P_predicted * H.transpose();
  Eigen::MatrixXd S = H * PHt + R;

  // W = PHt * S^-1, but obtained by solving S * W^T = PHt^T rather than
  // invering because it is cheaper and more stable
  Eigen::MatrixXd W = S.ldlt().solve(PHt.transpose()).transpose();

  // use the proper compostion operation, rather than regular +, - operations
  Eigen::VectorXd x_updated =
      motion_model_->composition_plus(x_predicted, W * innovation);

  // Joseph form: (I - WH) P (I - WH)^T + W R W^T. Algebraically equal to the
  // textbook (I - WH) P, but always positive definite
  Eigen::MatrixXd I =
      Eigen::MatrixXd::Identity(P_predicted.rows(), P_predicted.rows());
  Eigen::MatrixXd IWH = I - W * H;
  Eigen::MatrixXd P_updated =
      IWH * P_predicted * IWH.transpose() + W * R * W.transpose();

  FilterUpdate filter_update{x_updated, innovation, S, P_updated};
  return filter_update;
}
