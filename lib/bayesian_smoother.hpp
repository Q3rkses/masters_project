/**
 * @file bayesian_smoother.hpp
 * @brief Contains a basic Bayesian smoother virtual class, that collects:
 * x_smoothed, P_smoothed at each timestep
 */

#ifndef BAYESIAN_SMOOTHER_HPP
#define BAYESIAN_SMOOTHER_HPP

#include "bayesian_filter.hpp"
#include "models.hpp"
#include <Eigen/Dense>
#include <memory>

struct SmootherUpdate {
  Eigen::VectorXd x_smoothed;
  Eigen::MatrixXd P_smoothed;
};

struct SmootherConfig {
  Eigen::VectorXd x_prior;
  Eigen::MatrixXd P_prior;
  std::shared_ptr<const MotionModel> motion_model;
};

/**
 * @brief The Bayesian Smoother virtual class structure will
 * be inherited and overridden.
 */
class BayesianSmoother {
public:
  /**
   * @brief Destructor for the BayesianSmoother class.
   */
  virtual ~BayesianSmoother() = default;

  /**
   * @brief The backward recursion step in a Bayesian Smoother
   * needs to be overridden by the smoother that inherits
   * from the BayesianSmoother class
   * @param filtered_current, x_updated and P_updated at timestep k, as
   * produced by the forward filter pass
   * @param predicted_next, x_predicted and P_predicted at timestep k+1
   * @param input, the input the forward pass used when it predicted
   * timestep k+1 from timestep k
   * @return returns x_smoothed, P_smoothed at timestep k, packaged in
   * the SmootherUpdate class
   */
  virtual SmootherUpdate
  backward_recursion(const FilterUpdate &filtered_current,
                     const FilterPredict &predicted_next,
                     const SmootherUpdate &smoothed_previous,
                     const Input &input) = 0;
};

#endif
