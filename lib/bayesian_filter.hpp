/**
 * @file Bayesian_filter.hpp
 * @brief Contains a basic Bayesian_filter virtual class, that collects:
 * P_predicted, P_updated, x_predicted ,x_updated at each timestep
 */

#ifndef BAYESIAN_FILTER_HPP
#define BAYESIAN_FILTER_HPP

#include "models.hpp"
#include <Eigen/Dense>
#include <memory>

struct FilterPredict {
  Eigen::VectorXd x_predicted;
  Eigen::MatrixXd P_predicted;
};

struct FilterUpdate {
  Eigen::VectorXd x_updated;  // updated state estimate, x^_k in (Brekke 2025)
  Eigen::VectorXd innovation; // innovation eta in (Brekke, 2025)
  Eigen::MatrixXd S;          // innovation covariance S_k in (Brekke, 2025)
  Eigen::MatrixXd P_updated;  // posterior covariance P_k in (Brekke, 2025)
};

struct FilterConfig {
  Eigen::VectorXd x_prior;
  Eigen::MatrixXd P_prior;
  std::shared_ptr<const MotionModel> motion_model;
  std::shared_ptr<const MeasurementModel> measurement_model;
};

/**
 * @brief The Bayesian Filter virtual class structure will
 * be inherited and overridden.
 */
class BayesianFilter {
public:
  /**
   * @brief Destructor for the BayesianFilter class.
   */
  virtual ~BayesianFilter() = default;

  /**
   * @brief The prediction step in a Bayesian Filter
   * needs to be overridden by the filter that inherits
   * from the bayesian filter class
   * @param x_current, the density of the current state
   * @param input, the input to the system during this step
   * @return returns x_predicted, P_predicted packaged in the FilterPredict
   * class
   */
  virtual FilterPredict
  predict(const Eigen::VectorXd &x_current, const Eigen::MatrixXd &P_current,
          const Input &input) = 0; // -> x_predicted, P_predicted

  /**
   * @brief The update step in a Bayesian Filter
   * needs to be overridden by the filter that inherits
   * from the bayesian filter class
   * @param x_predicted, the density of the current prediction
   * @param z_current, the current measurement
   * @param input, the input to the system during this step
   * @return returns x_updated, innovation, P_updated packaged in the
   * FilterUpdate class
   */
  virtual FilterUpdate
  update(const Eigen::VectorXd &x_predicted, const Eigen::VectorXd &z_current,
         const Eigen::MatrixXd &P_predicted,
         const Input &input) = 0; // -> x_updated, innovation, P_updated
};

#endif
