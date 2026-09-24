/**
 * @file extended_rauch_tung_striebel_smoother.hpp
 * @brief Contains a basic erts_smoother class, that smooths
 * estimates that were previously obtained by utilizing an extended kalman
 * filter as a state estimator
 */

#ifndef EXTENDED_RAUCH_TUNG_STRIEBEL_SMOOTHER_HPP
#define EXTENDED_RAUCH_TUNG_STRIEBEL_SMOOTHER_HPP

#include "bayesian_smoother.hpp"
#include "models.hpp"
#include <Eigen/Dense>

class ERTSSmoother final : public BayesianSmoother {
public:
  /**
   * @brief Constructor for the ERTS smoother class.
   * @param smoother_config all configuration parameters that are
   * used by the constructor
   */
  explicit ERTSSmoother(const SmootherConfig &smoother_config);

  /**
   * @brief the backwards recursion equation based on Algorithm
   * 13.1 from Bayesian Smoothing and Filtering (Sarkka, p.255)
   * @param filtered_current, x_updated and P_updated at timestep k as
   * produced by the forward filter pass
   * @param predicted_next, x_predicted and P_predicted at timestep k+1
   * @param input, which is the input at time k
   * @return returns x_smoothed, P_smoothed at timestep k, packaged in
   * the SmootherUpdate class
   */
  SmootherUpdate backward_recursion(const FilterUpdate &filtered_current,
                                    const FilterPredict &predicted_next,
                                    const SmootherUpdate &smoothed_previous,
                                    const Input &input) override;

private:
  std::shared_ptr<const MotionModel> motion_model_;
};

#endif
