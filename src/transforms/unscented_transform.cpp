#include "transforms/unscented_transform.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <vector>

UnscentedTransform::UnscentedTransform(const TransformConfig &transform_config)
    : dimension_(transform_config.dimension) {
  if (dimension_ <= 0 || transform_config.alpha <= 0) {
    throw std::invalid_argument(
        "UnscentedTransform: dimension and alpha must be positive");
  }

  lambda_ = transform_config.alpha * transform_config.alpha *
                (dimension_ + transform_config.kappa) -
            dimension_;
  if (dimension_ + lambda_ <= 0) {
    throw std::invalid_argument(
        "UnscentedTransform: dimension + lambda must be positive");
  }

  // all outer points share one weight, only the centre point differs
  weights_mean_ = Eigen::VectorXd::Constant(2 * dimension_ + 1,
                                            1.0 / (2.0 * (dimension_ + lambda_)));
  weights_covariance_ = weights_mean_;

  weights_mean_(0) = lambda_ / (dimension_ + lambda_);
  weights_covariance_(0) =
      weights_mean_(0) + (1 - transform_config.alpha * transform_config.alpha +
                          transform_config.beta);
}

GaussianMoments UnscentedTransform::transform(
    const Gaussian &prior_gaussian,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &f,
    const Eigen::MatrixXd &noise_covariance_matrix, const Manifold &input_space,
    const Manifold &output_space) const {
  if (prior_gaussian.mean.size() != dimension_) {
    throw std::invalid_argument(
        "UnscentedTransform::transform: prior mean size does not match "
        "the dimension of the transform");
  }

  const int sigma_point_count = 2 * dimension_ + 1;

  // TODO: implement ldlt as fallback if llt fails
  const Eigen::MatrixXd L = prior_gaussian.covariance.llt().matrixL();

  // sigma points are the mean perturbed by delta in the tangent space,
  // the deltas are kept since the cross covariance needs them
  std::vector<Eigen::VectorXd> deltas;
  std::vector<Eigen::VectorXd> sigma_points;
  deltas.push_back(Eigen::VectorXd::Zero(dimension_));
  sigma_points.push_back(prior_gaussian.mean);

  for (int i = 0; i < dimension_; i++) {
    deltas.push_back(std::sqrt(dimension_ + lambda_) * L.col(i));
  }
  for (int i = 0; i < dimension_; i++) {
    deltas.push_back(-std::sqrt(dimension_ + lambda_) * L.col(i));
  }
  for (int i = 1; i < sigma_point_count; i++) {
    sigma_points.push_back(
        input_space.composition_plus(prior_gaussian.mean, deltas[i]));
  }

  // push the sigma points through nonlinearity
  std::vector<Eigen::VectorXd> transformed_sigma_points;
  for (int i = 0; i < sigma_point_count; i++) {
    transformed_sigma_points.push_back(f(sigma_points[i]));
  }

  // mean, one step in the tangent space around the transformed centre point
  const Eigen::VectorXd &y_ref = transformed_sigma_points[0];
  Eigen::VectorXd mean_delta =
      Eigen::VectorXd::Zero(noise_covariance_matrix.rows());
  for (int i = 0; i < sigma_point_count; i++) {
    mean_delta += weights_mean_(i) *
                  output_space.composition_minus(transformed_sigma_points[i],
                                                 y_ref);
  }
  Eigen::VectorXd mean = output_space.composition_plus(y_ref, mean_delta);

  // covariance and cross covariance about the computed mean
  Eigen::MatrixXd covariance = noise_covariance_matrix;
  Eigen::MatrixXd cross_covariance =
      Eigen::MatrixXd::Zero(dimension_, noise_covariance_matrix.rows());
  for (int i = 0; i < sigma_point_count; i++) {
    covariance += weights_covariance_(i) * output_space.composition_minus(transformed_sigma_points[i], mean) * output_space.composition_minus(transformed_sigma_points[i], mean).transpose();
    cross_covariance += weights_covariance_(i) * deltas[i] * output_space.composition_minus(transformed_sigma_points[i], mean).transpose();
  }
  covariance = 0.5 * (covariance + covariance.transpose());

  GaussianMoments moments{mean, covariance, cross_covariance};
  return moments;
}
