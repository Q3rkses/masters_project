#include "filters/unscented_kalman_filter.hpp"
#include "filters/bayesian_filter.hpp"
#include "models/models.hpp"
#include "transforms/unscented_transform.hpp"
#include <Eigen/Dense>
#include <stdexcept>

UnscentedKalmanFilter::UnscentedKalmanFilter(
    const FilterConfig &filter_config, const TransformConfig &transform_config)
    : motion_model_(filter_config.motion_model),
      unscented_transform_(transform_config) {
  if (!motion_model_) {
    throw std::invalid_argument(
        "UnscentedKalmanFilter: the filter config needs a motion model");
  }
};

FilterPredict UnscentedKalmanFilter::predict(const Eigen::VectorXd &x_current,
                                             const Eigen::MatrixXd &P_current,
                                             const Input &input) {
  // the transform wants a function of x only, so the input is captured
  auto f = [&](const Eigen::VectorXd &x) { return motion_model_->f(x, input); };

  Eigen::MatrixXd Q = motion_model_->Q(x_current, input);

  // clang-format off
  Gaussian prior_gaussian{
    .mean = x_current,
    .covariance = P_current
  };
  // clang-format on

  GaussianMoments predicted_moments = unscented_transform_.transform(
      prior_gaussian, f, Q, *motion_model_, *motion_model_);

  // clang-format off
  // create and fill the output struct
  FilterPredict filter_predict{
    .x_predicted = predicted_moments.mean,
    .P_predicted = predicted_moments.covariance
  };
  filter_predict.x_predicted = predicted_moments.mean;
  filter_predict.P_predicted = predicted_moments.covariance;
  return filter_predict;
  // clang-format on
}

FilterUpdate UnscentedKalmanFilter::update(
    const Eigen::VectorXd &x_predicted, const Eigen::VectorXd &z_current,
    const Eigen::MatrixXd &P_predicted, const Input &input,
    const std::shared_ptr<const MeasurementModel> &measurement_model) {
  if (!measurement_model) {
    throw std::invalid_argument(
        "UnscentedKalmanFilter::update: measurement model is null");
  }

  // same as in predict, the input is captured
  auto h = [&](const Eigen::VectorXd &x) {
    return measurement_model->h(x, input);
  };

  // clang-format off
  Gaussian prior_gaussian{
    .mean = x_predicted,
    .covariance = P_predicted
  };
  // clang-format on

  Eigen::MatrixXd R = measurement_model->R(x_predicted, input);

  GaussianMoments predicted_moments = unscented_transform_.transform(
      prior_gaussian, h, R, *motion_model_, *measurement_model);

  Eigen::VectorXd z_predicted = predicted_moments.mean;
  Eigen::MatrixXd covariance = predicted_moments.covariance;
  Eigen::MatrixXd cross_covariance = predicted_moments.cross_covariance;

  Eigen::VectorXd innovation =
      measurement_model->composition_minus(z_current, z_predicted);

  // clang-format off
  Eigen::MatrixXd W = covariance.ldlt().solve(cross_covariance.transpose()).transpose();
  Eigen::VectorXd x_updated = motion_model_->composition_plus(x_predicted, W * innovation);
  Eigen::MatrixXd P_updated = P_predicted - W * covariance * W.transpose();

  FilterUpdate filter_update{
    .x_updated = x_updated,
    .innovation = innovation,
    .S = covariance, 
    .P_updated = P_updated
  };
  return filter_update;
  // clang-format on
}
