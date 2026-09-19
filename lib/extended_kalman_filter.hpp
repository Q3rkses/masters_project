
/**
 * @file extended_kalman_filter.hpp
 * @brief Contains a basic extended_kalman_filter class, that collects:
 * P_predicted, P_updated, x_predicted, x_updated at each timestep
 */

#ifndef EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_HPP

#include "bayesian_filter.hpp"
#include "models.hpp"
#include <Eigen/Dense>

class ExtendedKalmanFilter final : public BayesianFilter {
public:
  /**
   * @brief Constructor for the Extended Kalman Filter class.
   * @param filter_config all configuration parameters that are
   * used by the constructor
   */
  explicit ExtendedKalmanFilter(const FilterConfig &filter_config);

  /**
   * @brief The prediction step in an Extended Kalman Filter
   * based on algorithm 1 in Fundamentals of Sensorfusion (Brekke, p.56)
   * utilizes the Joseph form for better numerical stability of P
   * @param x_current, the density of the current state
   * @return returns x_predicted, P_predicted
   */
  FilterPredict predict(Eigen::VectorXd x_current,
                        Eigen::MatrixXd P_current) override;

  /**
   * @brief The update step in an Extended Kalman Filter
   * based on algorithm 1 in Fundamentals of Sensorfusion (Brekke, p.56)
   * utilizes the Joseph form for better numerical stability of P
   * @param x_predicted, the density of the current prediction
   * @param z_current, the current measurement
   * @return returns x_updated, innovation, P_updated
   */
  FilterUpdate update(Eigen::VectorXd x_predicted, Eigen::VectorXd z_current,
                      Eigen::MatrixXd P_current) override;

private:
  MotionModel motion_model_;
  MeasurementModel measurement_model_;
};

#endif
