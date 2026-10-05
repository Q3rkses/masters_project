/**
 * @file unscented_transform.hpp
 * @brief Contains a basic unscented_transform class
 */

#ifndef UNSCENTED_TRANSFORM_HPP
#define UNSCENTED_TRANSFORM_HPP

#include "models/manifold.hpp"
#include <Eigen/Dense>
#include <functional>

struct TransformConfig {
  double alpha;
  double beta;
  double kappa;
  int dimension;
};

struct Gaussian {
  Eigen::VectorXd mean;
  Eigen::MatrixXd covariance;
};

struct GaussianMoments {
  Eigen::VectorXd mean;
  Eigen::MatrixXd covariance;
  Eigen::MatrixXd cross_covariance;
};

class UnscentedTransform {
public:
  /**
   * @brief Constructor for the Unscented Transform class
   * @param &transform_config, a reference to the transform_config
   * struct that will provide all parameters neccesary for initialization
   */
  explicit UnscentedTransform(const TransformConfig &transform_config);

  /**
   * @brief The Unscented Transform which will take a prior
   * gaussian as input, and produce (deterministically) moments
   * of the gaussian passed through the (possibly) nonlinear function
   * @param prior_gaussian, the prior gaussian distribution
   * @param f, the possibly nonlinear function we propagate sigma points through
   * @param noise_covariance_matrix, the covariance of the additive noise
   * @param input_space, the manifold of the prior, used to generate sigma
   * points
   * @param output_space, the manifold of f's output, used to compute the
   * mean and covariances of the propagated sigma points
   * @return returns mean, covariance and cross_covariance packaged in the
   * GaussianMoments struct. The covariance includes the noise, the
   * cross_covariance, Cov(x, f(x)), does not
   */
  GaussianMoments
  transform(const Gaussian &prior_gaussian,
            const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &f,
            const Eigen::MatrixXd &noise_covariance_matrix,
            const Manifold &input_space, const Manifold &output_space) const;

private:
  Eigen::VectorXd weights_mean_;
  Eigen::VectorXd weights_covariance_;
  double lambda_;
  int dimension_;
};

#endif
