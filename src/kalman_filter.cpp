#include "kalman_filter.hpp"
#include "bayesian_filter.hpp"
#include "models.hpp"
#include <Eigen/Dense>
#include <stdexcept>

KalmanFilter::KalmanFilter(const FilterConfig &filter_config)
    : motion_model_(filter_config.motion_model),
      measurement_model_(filter_config.measurement_model) {
  if (!motion_model_ || !measurement_model_) {
    throw std::invalid_argument(
        "KalmanFilter: the filter config needs both a motion and a "
        "measurement model");
  }
};

FilterPredict KalmanFilter::predict(const Eigen::VectorXd &x_current,
                                    const Eigen::MatrixXd &P_current,
                                    const Input &input) {
  Eigen::MatrixXd F = motion_model_->F(x_current, input);
  Eigen::VectorXd x_predicted = motion_model_->f(x_current, input);
  Eigen::MatrixXd P_predicted =
      F * P_current * F.transpose() + motion_model_->Q(x_current, input);
  FilterPredict filter_prediction{x_predicted, P_predicted};
  return filter_prediction;
}

FilterUpdate KalmanFilter::update(const Eigen::VectorXd &x_predicted,
                                  const Eigen::VectorXd &z_current,
                                  const Eigen::MatrixXd &P_predicted,
                                  const Input &input) {

  Eigen::MatrixXd H = measurement_model_->H(x_predicted, input);
  Eigen::MatrixXd R = measurement_model_->R(x_predicted, input);

  Eigen::VectorXd z_predicted = measurement_model_->h(x_predicted, input);

  // remember to use the proper compostion operation, rather than
  // the default + or - operator when dealing with states / measurements
  Eigen::VectorXd innovation =
      measurement_model_->composition_minus(z_current, z_predicted);

  Eigen::MatrixXd PHt = P_predicted * H.transpose();
  Eigen::MatrixXd S = H * PHt + R;

  // W = PHt * S^-1, but obtained by solving S * W^T = PHt^T rather than
  // invering because it is cheaper and more stable
  Eigen::MatrixXd W = S.ldlt().solve(PHt.transpose()).transpose();

  // remember to use the proper compostion operation, rather than
  // the default + or - operator when dealing with states / measurements
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
