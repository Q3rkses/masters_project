/**
 * @file Bayesian_filter.hpp
 * @brief Contains a basic Bayesian_filter virtual class, that collects:
 * P_predicted, P_updated, x_predicted ,x_updated at each timestep
 */

#ifndef BAYESIAN_FILTER_HPP
#define BAYESIAN_FILTER_HPP

#include "models.hpp"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

class FilterPredict {
  /**
   * @brief Constructor for the Filter Predict class.
   * @param filter_config all configuration parameters
   */
  explicit FilterPredict();

public:
  Eigen::VectorXd x_predicted;
  Eigen::MatrixXd P_predicted;
};

class FilterUpdate {
  /**
   * @brief Constructor for the Filter Update class.
   * @param filter_config all configuration parameters
   */
  explicit FilterUpdate();

public:
  Eigen::VectorXd x_updated;
  Eigen::VectorXd innovation;
  Eigen::MatrixXd P_updated;
};

class FilterConfig {
  /**
   * @brief Constructor for the Filter Config class.
   * @param filter_config all configuration parameters
   */
  explicit FilterConfig();
  MotionModel motion_model;
  MeasurementModel measurement_model;
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
   * @return returns x_predicted, P_predicted packaged in the FilterPredict
   * class
   */
  virtual FilterPredict
  predict(Eigen::VectorXd x_current); // -> x_predicted, P_predicted

  /**
   * @brief The update step in a Bayesian Filter
   * needs to be overridden by the filter that inherits
   * from the bayesian filter class
   * @param x_predicted, the density of the current prediction
   * @param z_current, the current measurement
   * @return returns x_updated, innovation, P_updated packaged in the
   * FilterUpdate class
   */
  virtual FilterUpdate
  update(Eigen::VectorXd x_predicted,
         Eigen::VectorXd z_current); // -> x_updated, innovation, P_updated
};

#endif
