/**
 * @file kalman_filter.hpp
 * @brief Contains a basic kalman_filter class, that collects:
 * P_predicted, P_updated, x_predicted ,x_updated at each timestep
 */

#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "bayesian_filter.hpp"
#include "models.hpp"
#include <Eigen/Dense>
#include <memory>

class KalmanFilter final : public BayesianFilter {
public:
  /**
   * @brief Constructor for the Kalman Filter class.
   * @param filter_config all configuration parameters that are
   * used by the constructor
   */
  explicit KalmanFilter(const FilterConfig &filter_config);

  /**
   * @brief The prediction step in a Kalman Filter
   * based on algorithm 1 in Fundamentals of Sensorfusion (Brekke, p.56)
   * utilizes the Joseph form for better numerical stability of P
   * @param x_current, the density of the current state
   * @param input, the input to the system during this step
   * @return returns x_predicted, P_predicted
   */
  FilterPredict predict(const Eigen::VectorXd &x_current,
                        const Eigen::MatrixXd &P_current,
                        const Input &input) override;

  /**
   * @brief The update step in a Kalman Filter
   * based on algorithm 1 in Fundamentals of Sensorfusion (Brekke, p.56)
   * utilizes the Joseph form for better numerical stability of P
   * @param x_predicted, the density of the current prediction
   * @param z_current, the current measurement
   * @param input, the input to the system during this step
   * @return returns x_updated, innovation, P_updated
   */
  FilterUpdate update(const Eigen::VectorXd &x_predicted,
                      const Eigen::VectorXd &z_current,
                      const Eigen::MatrixXd &P_current,
                      const Input &input) override;

private:
  std::shared_ptr<const MotionModel> motion_model_;
  std::shared_ptr<const MeasurementModel> measurement_model_;
};

#endif
