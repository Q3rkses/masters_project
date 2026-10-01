#include "filters/unscented_kalman_filter.hpp"
#include "filters/bayesian_filter.hpp"
#include "models/models.hpp"
#include <Eigen/Dense>
#include <stdexcept>

UnscentedKalmanFilter::UnscentedKalmanFilter(const FilterConfig &filter_config)
    : motion_model_(filter_config.motion_model) {
  if (!motion_model_) {
    throw std::invalid_argument(
        "UnscentedKalmanFilter: the filter config needs a motion model");
  }
};

FilterPredict UnscentedKalmanFilter::predict(const Eigen::VectorXd &x_current,
                                             const Eigen::MatrixXd &P_current,
                                             const Input &input) {
  FilterPredict filter_predict{};
  return filter_predict;
}

FilterUpdate UnscentedKalmanFilter::update(
    const Eigen::VectorXd &x_predicted, const Eigen::VectorXd &z_current,
    const Eigen::MatrixXd &P_predicted, const Input &input,
    const std::shared_ptr<const MeasurementModel> &measurement_model) {
  if (!measurement_model) {
    throw std::invalid_argument(
        "UnscentedKalmanFilter::update: measurement model is null");
  }

  FilterUpdate filter_update{};
  return filter_update;
}
