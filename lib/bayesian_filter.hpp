/**
 * @file Bayesian_filter.hpp
 * @brief Contains a basic Bayesian_filter virtual class, that collects:
 * P_predicted, P_updated, x_predicted ,x_updated at each timestep
 */

#ifndef BAYESIAN_FILTER_HPP
#define BAYESIAN_FILTER_HPP

#include <eigen3/Eigen/Eigen>

/**
 * @brief The Bayesian Filter virtual class structure that the solvers will
 * inherit and override.
 */
class BayesianFilter {
public:
  /**
   * @brief Destructor for the Allocator class.
   */
  // virtual ~Allocator() = default;

  /**
   * @brief The prediction step in a Bayesian Filter
   * needs to be overridden by the filter that inherits
   * from the bayesian filter class
   * @param x_current, the density of the current state
   * @return returns x_predicted, P_predicted
   */
  // virtual predict(x_current) -> x_predicted, innovation, P_predicted

  /**
   * @brief The update step in a Bayesian Filter
   * needs to be overridden by the filter that inherits
   * from the bayesian filter class
   * @param x_predicted, the density of the current prediction
   * @param z_current, the current measurement
   * @return returns x_updated, P_updated
   */
  // update(x_predicted, z_current) -> x_updated, P_updated
};

#endif
