/**
 * @file iterated_extended_kalman_filter.hpp
 * @brief Contains a basic iterated_extended_kalman_filter class, that collects:
 * P_predicted, P_updated, x_predicted, x_updated at each timestep
 */

#ifndef ITERATED_EXTENDED_KALMAN_FILTER_HPP
#define ITERATED_EXTENDED_KALMAN_FILTER_HPP

#include "filters/bayesian_filter.hpp"
#include "models/nonlinear_models.hpp"
#include <Eigen/Dense>

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
  /**
   * @brief The MAP cost of the update, which the LM method minimizes.
   * @param x, the state to evaluate the cost at
   * @param x_predicted, the mean of the current prediction
   * @param P_predicted_ldlt, factorisation of P_predicted, computed once
   * before the iteration
   * @param z_current, the current measurement
   * @param R_ldlt, factorisation of R, computed once before the iteration
   * @param input, which is the input in the current timestep
   * @param measurement_model, the model of the sensor that produced
   * z_current
   * @return the scalar cost
   */
  double cost(const Eigen::VectorXd &x, const Eigen::VectorXd &x_predicted,
              const Eigen::LDLT<Eigen::MatrixXd> &P_predicted_ldlt,
              const Eigen::VectorXd &z_current,
              const Eigen::LDLT<Eigen::MatrixXd> &R_ldlt, const Input &input,
              const MeasurementModel &measurement_model) const;

  std::shared_ptr<const MotionModel> motion_model_;
  const LevenbergMarquardtConfig &lm_config;
};

#endif
