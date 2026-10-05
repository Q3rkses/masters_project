#include "smoothers/unscented_rauch_tung_striebel_smoother.hpp"
#include "filters/bayesian_filter.hpp"
#include "models/models.hpp"
#include "smoothers/bayesian_smoother.hpp"
#include <Eigen/Dense>
#include <stdexcept>

URTSSmoother::URTSSmoother(const SmootherConfig &smoother_config)
    : motion_model_(smoother_config.motion_model) {
  if (!motion_model_) {
    throw std::invalid_argument(
        "URTSSmoother: the smoother config needs a motion model");
  }
};

SmootherUpdate URTSSmoother::backward_recursion(
    const FilterUpdate &filtered_current, const FilterPredict &predicted_next,
    const SmootherUpdate &smoothed_previous, const Input &input) {
  SmootherUpdate smoothed_update{};
  return smoothed_update;
};
