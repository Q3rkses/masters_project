/**
 * @file unscented_kalman_filter.hpp
 * @brief Contains a basic unscented_kalman_filter class, that collects:
 * P_predicted, P_updated, x_predicted, x_updated at each timestep
 */

#ifndef UNSCENTED_KALMAN_FILTER_HPP
#define UNSCENTED_KALMAN_FILTER_HPP

#include "filters/bayesian_filter.hpp"
#include "models/models.hpp"
#include <Eigen/Dense>

class UnscentedKalmanFilter final : public BayesianFilter {
public:
  /**
   * @brief Constructor for the Unscented Kalman Filter class.
   * @param filter_config all configuration parameters that are
   * used by the constructor.
   */
  explicit UnscentedKalmanFilter(const FilterConfig &filter_config);

  /**
   * @brief The prediction step in an Unscented Kalman Filter
   * based on Bayesian Smoothing and Filtering (Sarkka)
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
   * @brief The update step in an Unscented Kalman Filter
   * based on Bayesian Smoothing and Filtering (Sarkka)
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
