/**
 * @file rauch_tung_striebel_smoother.hpp
 * @brief Contains a basic rts_smoother class, that smooths
 * a estimates that were previously obtained by utilizing a kalman
 * filter as a state estimator
 */

#ifndef RAUCH_TUNG_STRIEBEL_SMOOTHER_HPP
#define RAUCH_TUNG_STRIEBEL_SMOOTHER_HPP

#include "bayesian_smoother.hpp"
#include "models.hpp"
#include <eigen3/Eigen/Eigen>

class RTSSmoother final : public BayesianSmoother {
public:
  /**
   * @brief Constructor for the RTS smoother class.
   * @param smoother_config all configuration parameters that are
   * used by the constructor
   */
  explicit RTSSmoother(const SmootherConfig &smoother_config);

  /**
   * @brief the backwards recursion equation based on
   * Theorem 12.2 from Bayesian Smoothing and Filtering (Sarakka, p.255)
   * @param filtered_current, x_updated and P_updated at timestep k as
   * produced by the forward filter pass
   * @param predicted_next, x_predicted and P_predicted at timestep k+1
   * @return returns x_smoothed, P_smoothed at timestep k, packaged in
   * the SmootherUpdate class
   */
  SmootherUpdate
  backward_recursion(const FilterUpdate &filtered_current,
                     const FilterPredict &predicted_next,
                     const SmootherUpdate &smoothed_previous) override;

private:
  MotionModel motion_model;
};

#endif
