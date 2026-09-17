#include "rauch_tung_striebel_smoother.hpp"
#include "bayesian_filter.hpp"
#include "bayesian_smoother.hpp"
#include "models.hpp"
#include <Eigen/Dense>

RTSSmoother::RTSSmoother(const SmootherConfig &smoother_config)
    : motion_model_(smoother_config.motion_model) {};

SmootherUpdate
RTSSmoother::backward_recursion(const FilterUpdate &filtered_current,
                                const FilterPredict &predicted_next,
                                const SmootherUpdate &smoothed_previous) {
  // solving linear system is faster and more algebraically stable than
  // matrix inversion.
  Eigen::MatrixXd FP = motion_model_.F * filtered_current.P_updated;
  Eigen::MatrixXd W =
      predicted_next.P_predicted.ldlt().solve(FP).transpose();

  Eigen::VectorXd x_smoothed =
      filtered_current.x_updated +
      W * (smoothed_previous.x_smoothed - predicted_next.x_predicted);
  Eigen::MatrixXd P_smoothed =
      filtered_current.P_updated +
      W * (smoothed_previous.P_smoothed - predicted_next.P_predicted) *
          W.transpose();
  SmootherUpdate smoothed_update{x_smoothed, P_smoothed};
  return smoothed_update;
};
