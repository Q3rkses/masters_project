#include "smoothers/rauch_tung_striebel_smoother.hpp"
#include "filters/bayesian_filter.hpp"
#include "smoothers/bayesian_smoother.hpp"
#include "models/models.hpp"
#include <Eigen/Dense>
#include <stdexcept>

RTSSmoother::RTSSmoother(const SmootherConfig &smoother_config)
    : motion_model_(smoother_config.motion_model) {
  if (!motion_model_) {
    throw std::invalid_argument(
        "RTSSmoother: the smoother config needs a motion model");
  }
  if (!motion_model_->is_linear()) {
    throw std::invalid_argument(
        "RTSSmoother requires a linear motion model; use ERTSSmoother for "
        "nonlinear models such as StrapdownINS2D");
  }
};

SmootherUpdate
RTSSmoother::backward_recursion(const FilterUpdate &filtered_current,
                                const FilterPredict &predicted_next,
                                const SmootherUpdate &smoothed_previous,
                                const Input &input) {
  // F is evaluated at the filtered state, with the same input the forward
  // pass used for this transition, so it is the F the filter linearized at.
  Eigen::MatrixXd F = motion_model_->F(filtered_current.x_updated, input);

  // solving linear system is faster and more algebraically stable than
  // matrix inversion.
  Eigen::MatrixXd FP = F * filtered_current.P_updated;
  Eigen::MatrixXd W =
      predicted_next.P_predicted.ldlt().solve(FP).transpose();

  // remember to use the proper compostion operation, rather than
  // the default + or - operator when dealing with states
  Eigen::VectorXd delta = motion_model_->composition_minus(
      smoothed_previous.x_smoothed, predicted_next.x_predicted);
  Eigen::VectorXd x_smoothed =
      motion_model_->composition_plus(filtered_current.x_updated, W * delta);
  Eigen::MatrixXd P_smoothed =
      filtered_current.P_updated +
      W * (smoothed_previous.P_smoothed - predicted_next.P_predicted) *
          W.transpose();
  SmootherUpdate smoothed_update{x_smoothed, P_smoothed};
  return smoothed_update;
};
