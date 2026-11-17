/**
 * @file bayesian_batch_smoother.hpp
 * @brief Contains a basic Bayesian batch smoother virtual class. A batch
 * smoother needs the whole run at once, since it relinearizes the models
 * about a full trajectory.
 */

#ifndef BAYESIAN_BATCH_SMOOTHER_HPP
#define BAYESIAN_BATCH_SMOOTHER_HPP

#include "models/models.hpp"
#include "results.hpp"
#include "sensors.hpp"
#include "smoothers/bayesian_smoother.hpp"
#include <Eigen/Dense>
#include <vector>

/**
 * @brief The Bayesian batch smoother virtual class
 */
class BayesianBatchSmoother {
public:
  /**
   * @brief Destructor for the BayesianBatchSmoother class.
   */
  virtual ~BayesianBatchSmoother() = default;

  /**
   * @brief smooths the whole run. The forward pass predicts step k with
   * inputs[k], so the transition from step k - 1 to step k uses inputs[k].
   * @param smoother_config the smoother's prior and motion model
   * @param inputs the input of every step
   * @param sensors the sensors, with their measurements and when they fire
   * @param initial_guess a filter's recorded states, the trajectory the
   * smoother starts from
   * @return the smoothed state at every step
   */
  virtual SmootherResult smooth(const SmootherConfig &smoother_config,
                                const std::vector<Input> &inputs,
                                const std::vector<Sensor> &sensors,
                                const FilterResult &initial_guess) = 0;
};

#endif
