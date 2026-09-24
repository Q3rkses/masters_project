#include "extended_rauch_tung_striebel_smoother.hpp"
#include "bayesian_filter.hpp"
#include "bayesian_smoother.hpp"
#include "models.hpp"
#include <Eigen/Dense>
#include <stdexcept>

ERTSSmoother::ERTSSmoother(const SmootherConfig &smoother_config)
    : motion_model_(smoother_config.motion_model) {
  if (!motion_model_) {
    throw std::invalid_argument(
        "ERTSSmoother: the smoother config needs a motion model");
  }
};

SmootherUpdate ERTSSmoother::backward_recursion(
    const FilterUpdate &filtered_current, const FilterPredict &predicted_next,
    const SmootherUpdate &smoothed_previous, const Input &input) {
  // F is evaluated at the filtered state, with the same input the forward
  // pass used for this transition, so it is the F the filter linearized at.
  Eigen::MatrixXd F = motion_model_->F(filtered_current.x_updated, input);

  // solving linear system is faster and more algebraically stable than
  // matrix inversion.
  Eigen::MatrixXd FP = F * filtered_current.P_updated;
  Eigen::MatrixXd W = predicted_next.P_predicted.ldlt().solve(FP).transpose();

  // assemble the smoother equations from Algorithm 13.1
  Eigen::VectorXd x_smoothed =
      filtered_current.x_updated +
      W * (smoothed_previous.x_smoothed - predicted_next.x_predicted);
  Eigen::MatrixXd P_smoothed =
      filtered_current.P_updated +
      W * (smoothed_previous.P_smoothed - predicted_next.P_predicted) *
          W.transpose();

  // assemble into answer
  SmootherUpdate smoothed_update{x_smoothed, P_smoothed};
  return smoothed_update;
};
