/**
 * @file iterated_extended_kalman_smoother.hpp
 * @brief Contains an iterated_extended_rts_smoother class, that smooths
 * estimates that were previously obtained by utilizing an iterated extended
 * kalman filter as a state estimator
 */

#ifndef ITERATED_EXTENDED_RAUCH_TUNG_STRIEBEL_SMOOTHER_HPP
#define ITERATED_EXTENDED_RAUCH_TUNG_STRIEBEL_SMOOTHER_HPP

#include "models/nonlinear_models.hpp"
#include "smoothers/bayesian_smoother.hpp"
#include <Eigen/Dense>

class IteratedExtendedKalmanSmoother final : public BayesianSmoother {
public:
  /**
   * @brief Constructor for the IteratedExtendedRTS smoother class.
   * @param smoother_config all configuration parameters that are
   * used by the constructor
   */
  explicit IteratedExtendedKalmanSmoother(
      const SmootherConfig &smoother_config,
      const LevenbergMarquardtConfig &lm_config);

  /**
   * @brief the backwards recursion equation based on Algorithm
   * 13.5 from Bayesian Smoothing and Filtering (Sarkka, p.274)
   * @param filtered_current, x_updated and P_updated at timestep k as
   * produced by the forward filter pass
   * @param predicted_next, x_predicted and P_predicted at timestep k+1
   * @param input, the input the forward pass used when it predicted
   * timestep k+1 from timestep k
   * @return returns x_smoothed, P_smoothed at timestep k, packaged in
   * the SmootherUpdate class
   */
  SmootherUpdate backward_recursion(const FilterUpdate &filtered_current,
                                    const FilterPredict &predicted_next,
                                    const SmootherUpdate &smoothed_previous,
                                    const Input &input) override;

private:
  std::shared_ptr<const MotionModel> motion_model_;
  const LevenbergMarquardtConfig &lm_config;
};

#endif
