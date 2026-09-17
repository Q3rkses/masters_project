#include "kalman_filter.hpp"
#include "bayesian_filter.hpp"
#include "models.hpp"
#include <Eigen/Dense>

KalmanFilter::KalmanFilter(const FilterConfig &filter_config)
    : motion_model_(filter_config.motion_model),
      measurement_model_(filter_config.measurement_model) {};

FilterPredict KalmanFilter::predict(Eigen::VectorXd x_current,
                                    Eigen::MatrixXd P_current) {
  Eigen::VectorXd x_predicted = motion_model_.F * x_current;
  Eigen::MatrixXd P_predicted =
      motion_model_.F * P_current * motion_model_.F.transpose() +
      motion_model_.Q;
  FilterPredict filter_prediction{x_predicted, P_predicted};
  return filter_prediction;
}

FilterUpdate KalmanFilter::update(Eigen::VectorXd x_predicted,
                                  Eigen::VectorXd z_current,
                                  Eigen::MatrixXd P_predicted) {

  Eigen::VectorXd z_predicted = measurement_model_.H * x_predicted;
  Eigen::VectorXd innovation = z_current - z_predicted;

  Eigen::MatrixXd PHt = P_predicted * measurement_model_.H.transpose();
  Eigen::MatrixXd S = measurement_model_.H * PHt + measurement_model_.R;

  // W = PHt * S^-1, but obtained by solving S * W^T = PHt^T rather than
  // invering because it is cheaper and more stable
  Eigen::MatrixXd W = S.ldlt().solve(PHt.transpose()).transpose();

  Eigen::VectorXd x_updated = x_predicted + W * innovation;

  // Joseph form: (I - WH) P (I - WH)^T + W R W^T. Algebraically equal to the
  // textbook (I - WH) P, but always positive definite
  Eigen::MatrixXd I =
      Eigen::MatrixXd::Identity(P_predicted.rows(), P_predicted.rows());
  Eigen::MatrixXd IWH = I - W * measurement_model_.H;
  Eigen::MatrixXd P_updated = IWH * P_predicted * IWH.transpose() +
                              W * measurement_model_.R * W.transpose();

  FilterUpdate filter_update{x_updated, innovation, S, P_updated};
  return filter_update;
}
