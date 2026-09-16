/**
 * @file kalman_filter.hpp
 * @brief Contains a basic kalman_filter class, that collects:
 * P_predicted, P_updated, x_predicted ,x_updated at each timestep
 */

#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "bayesian_filter.hpp"
#include "models.hpp"
#include <eigen3/Eigen/Eigen>

class KalmanFilter final : public BayesianFilter {
public:
  /**
   * @brief Constructor for the Kalman Filter class.
   * @param filter_config all configuration parameters that are
   * used by the constructor
   */
  // explicit KalmanFilter(const FilterConfig& filter_config);

  /**
   * @brief The prediction step in a Kalman Filter
   * based on algorithm 1 in Fundamentals of Sensorfusion (Brekke, p.56)
   * utilizes the Joseph form for better numerical stability of P
   * @param x_current, the density of the current state
   * @return returns x_predicted, P_predicted
   */
  // predict(x_current) override -> x_predicted, innovation, P_predicted

  /**
   * @brief The update step in a Kalman Filter
   * based on algorithm 1 in Fundamentals of Sensorfusion (Brekke, p.56)
   * utilizes the Joseph form for better numerical stability of P
   * @param x_predicted, the density of the current prediction
   * @param z_current, the current measurement
   * @return returns x_updated, P_updated
   */
  // update(x_predicted, z_current) override -> x_updated, P_updated

private:
  // x_predicted_list // the list of predicted states
  // P_predicted_list // the list of predicted covariances
  // x_updated_list // the list of the updated states
  // P_updated_list // the list of updated covariances
  //
  // innovation_list // list of all predicted measurements
  // (useful for visualization of the KF performance)

  // Quantities from the measurement and motion models
  // F
  // Q
  // H
  // R
};

#endif
