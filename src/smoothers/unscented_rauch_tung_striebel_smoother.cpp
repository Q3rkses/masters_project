#include "smoothers/unscented_rauch_tung_striebel_smoother.hpp"
#include "filters/bayesian_filter.hpp"
#include "models/models.hpp"
#include "smoothers/bayesian_smoother.hpp"
#include "transforms/unscented_transform.hpp"
#include <Eigen/Dense>
#include <stdexcept>

URTSSmoother::URTSSmoother(const SmootherConfig &smoother_config,
                           const TransformConfig &transform_config)
    : motion_model_(smoother_config.motion_model),
      unscented_transform_(transform_config) {
  if (!motion_model_) {
    throw std::invalid_argument(
        "URTSSmoother: the smoother config needs a motion model");
  }
};

SmootherUpdate URTSSmoother::backward_recursion(
    const FilterUpdate &filtered_current, const FilterPredict &predicted_next,
    const SmootherUpdate &smoothed_previous, const Input &input) {

  // the transform wants a function of x only, so the input is captured
  auto f = [&](const Eigen::VectorXd &x) { return motion_model_->f(x, input); };

  Eigen::MatrixXd Q = motion_model_->Q(filtered_current.x_updated, input);

  // clang-format off
  Gaussian prior_gaussian{
    .mean = filtered_current.x_updated,
    .covariance = filtered_current.P_updated
  };
  GaussianMoments gaussian_moments = unscented_transform_.transform(
      prior_gaussian, f, Q, *motion_model_, *motion_model_);

  // W = D (P^-)^-1, but obtained by solving P^- * W^T = D^T rather than
  // inverting, same as in the ERTS
  Eigen::MatrixXd W = gaussian_moments.covariance.ldlt().solve(
      gaussian_moments.cross_covariance.transpose()).transpose();

  Eigen::VectorXd delta = motion_model_->composition_minus(
      smoothed_previous.x_smoothed, gaussian_moments.mean);

  Eigen::VectorXd x_smoothed =
      motion_model_->composition_plus(filtered_current.x_updated, W * delta);

  Eigen::MatrixXd P_smoothed =
      filtered_current.P_updated + W * (smoothed_previous.P_smoothed - gaussian_moments.covariance) * W.transpose();

  SmootherUpdate smoothed_update{
    .x_smoothed = x_smoothed,
    .P_smoothed = P_smoothed
  };

  return smoothed_update;
  // clang-format on
};
