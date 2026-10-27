/**
 * @file iterated_extended_kalman_filter.hpp
 * @brief Contains a basic iterated_extended_kalman_filter class, that collects:
 * P_predicted, P_updated, x_predicted, x_updated at each timestep
 */

#ifndef ITERATED_EXTENDED_KALMAN_FILTER_HPP
#define ITERATED_EXTENDED_KALMAN_FILTER_HPP

#include "filters/bayesian_filter.hpp"
#include "models/models.hpp"
#include <Eigen/Dense>

struct LevenbergMarquardtConfig {
  double lambda_initial;    // initial damping factor
  double lambda_min;        // floor: below this the step is pure Gauss-Newton
  double lambda_max;        // ceiling: give up on this iterate if exceeded
  int max_iter;             // maximum accepted iterations
  int max_rejections;       // maximum rejections per iterate
  double step_tol;          // step tolerance below which we terminate
  double cost_relative_tol; // relative cost tolerance below which we terminate
};

class IteratedExtendedKalmanFilter final : public BayesianFilter {
public:
  /**
   * @brief Constructor for the Iterated Extended Kalman Filter class.
   * @param filter_config all configuration parameters for the filter
   * @param lm_config all configuration parameters for the LM method
   */
  explicit IteratedExtendedKalmanFilter(
      const FilterConfig &filter_config,
      const LevenbergMarquardtConfig &lm_config);

  /**
   * @brief The prediction step in an Iterated Extended Kalman Filter
   * based on Algorithm 7.9 in Sarakka's Bayesian Filtering and Smoothing
   * @param x_current, the density of the current state
   * @param P_current, the covariance of the current state
   * @param input, which is the input in the current timestep
   * @return returns x_predicted, P_predicted packaged in the
   * FiterPredict struct.
   */

  FilterPredict predict(const Eigen::VectorXd &x_current,
                        const Eigen::MatrixXd &P_current,
                        const Input &input) override;

  /**
   * @brief The update step in an Iterated Extended Kalman Filter
   * based on Algorithm 7.9 in Sarakka's Bayesian Filtering and Smoothing
   * but utilizes the Levenberg Marquardt method instead of Gauss-Newton.
   * @param x_predicted, the mean of the current prediction
   * @param P_predicted, the covariance of the current prediciton
   * @param z_current, the current measurement
   * @param input, which is the input in the current timestep
   * @param measurement_model, the model of the sensor that produced
   * z_current
   * @return returns x_updated, innovation, P_updated packaged in
   * the FilterUpdate struct.
   */
  FilterUpdate
  update(const Eigen::VectorXd &x_predicted, const Eigen::VectorXd &z_current,
         const Eigen::MatrixXd &P_predicted, const Input &input,
         const std::shared_ptr<const MeasurementModel> &measurement_model)
      override;

private:
  std::shared_ptr<const MotionModel> motion_model_;
};

#endif
