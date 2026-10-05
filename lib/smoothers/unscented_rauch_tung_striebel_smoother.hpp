/**
 * @file unscented_rauch_tung_striebel_smoother.hpp
 * @brief Contains a basic urts_smoother class, that smooths
 * estimates that were previously obtained by utilizing an unscented kalman
 * filter as a state estimator
 */

#ifndef UNSCENTED_RAUCH_TUNG_STRIEBEL_SMOOTHER_HPP
#define UNSCENTED_RAUCH_TUNG_STRIEBEL_SMOOTHER_HPP

#include "models/models.hpp"
#include "smoothers/bayesian_smoother.hpp"
#include "transforms/unscented_transform.hpp"
#include <Eigen/Dense>

class URTSSmoother final : public BayesianSmoother {
public:
  /**
   * @brief Constructor for the URTS smoother class.
   * @param smoother_config all configuration parameters that are
   * used by the constructor
   * @param transform_config the parameters of the unscented transform,
   * alpha, beta, kappa and the state dimension
   */
  URTSSmoother(const SmootherConfig &smoother_config,
               const TransformConfig &transform_config);

  /**
   * @brief the backwards recursion equation based on Algorithm
   * 14.8 from Bayesian Smoothing and Filtering (Sarkka, p.285)
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
  UnscentedTransform unscented_transform_;
};

#endif
