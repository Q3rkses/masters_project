/**
 * @file kalman_filter.hpp
 * @brief Contains a basic rts_smoother class, that smooths
 * a estimates that were previously obtained by utilizing a kalman
 * filter as a state estimator
 */

#ifndef RAUCH_TUNG_STRIEBEL_SMOOTHER_HPP
#define RAUCH_TUNG_STRIEBEL_SMOOTHER_HPP

#include <eigen3/Eigen/Eigen>

class RTSSmoother {
public:
  /**
   * @brief Constructor for the RTS smoother class.
   * @param filter_config all configuration parameters that are
   * used by the constructor
   */
  // explicit RTSSmoother(const SmootherConfig& smoother_config);

  /**
   * @brief the backwards recursion equation based on
   * Theorem 12.2 from Bayesian Smoothing and Filtering (Sarakka, p.255)
   * @param x_current, the density of the current state
   * @param x_previous, the density of the previous state
   * @param P_current, the covariance of the current density
   * @param P_previous, the covariance of the previous density
   * @return returns x_smoothed, P_smoothed
   */

  // backward_recursion(x_current, x_previous, P_current, P_previous) ->
  // x_smoothed, P_smoothed

private:
  // x_smoothed_list // the list of predicted states
  // P_smoothed_list // the list of predicted covariances

  // Quantities from the measurement and motion models
  // F
  // Q
  // H
  // R
};

#endif
