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
   * used by the constructor.
   */
  explicit ExtendedKalmanFilter(const FilterConfig &filter_config);

  /**
   * @brief The prediction step in an Extended Kalman Filter
   * based on algorithm 1 in Fundamentals of Sensorfusion (Brekke, p.56)
   * utilizes the Joseph form for better numerical stability of P
   * @param x_current, the density of the current state
   * @param P_predicted, the covariance of the current prediciton
   * @param input, which is the input in the current timestep
   * @return returns x_predicted, P_predicted packaged in the
   * FiterPredict struct.
   */

  FilterPredict predict(const Eigen::VectorXd &x_current,
                        const Eigen::MatrixXd &P_current,
                        const Input &input) override;

  /**
   * @brief The update step in an Extended Kalman Filter
   * based on algorithm 1 in Fundamentals of Sensorfusion (Brekke, p.56)
   * utilizes the Joseph form for better numerical stability of P
   * @param x_predicted, the density of the current prediction
   * @param P_predicted, the covariance of the current prediciton
   * @param z_current, the current measurement
   * @param input, which is the input in the current timestep
   * @return returns x_updated, innovation, P_updated packaged in
   * the FilterUpdate struct.
   */
  FilterUpdate update(const Eigen::VectorXd &x_predicted,
                      const Eigen::VectorXd &z_current,
                      const Eigen::MatrixXd &P_predicted,
                      const Input &input) override;

private:
  std::shared_ptr<const MotionModel> motion_model_;
  std::shared_ptr<const MeasurementModel> measurement_model_;
};

#endif
