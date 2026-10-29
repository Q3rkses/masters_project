/**
 * @file estimation.hpp
 * @brief Runs a filter forward and a smoother backward over simulated data.
 * Nothing here depends on the scenario, any set of sensors and any motion
 * model that follow the filter and smoother interfaces can be run through it.
 */

#ifndef ESTIMATION_HPP
#define ESTIMATION_HPP

#include "filters/bayesian_filter.hpp"
#include "models/models.hpp"
#include "results.hpp"
#include "sensors.hpp"
#include "smoothers/bayesian_smoother.hpp"
#include <Eigen/Dense>
#include <string>
#include <vector>

/**
 * @brief What one sensor reported and how a filter saw it, one entry per fix:
 * the step, the raw measurement, the innovation and the innovation covariance
 * S. The innovations and S are what the NIS is computed from.
 */
struct SensorInnovationLog {
  std::string sensor;
  int dimension; // dimension of the measurement
  std::vector<int> steps;
  std::vector<Eigen::VectorXd> z;
  std::vector<Eigen::VectorXd> innovations;
  std::vector<Eigen::MatrixXd> S;

  /**
   * @brief records one fix
   * @param k the step of the fix
   * @param measurement the raw measurement
   * @param update the filter's update with that measurement
   */
  void add(const int k, const Eigen::VectorXd &measurement,
           const FilterUpdate &update);
};

/**
 * @brief The result of running a filter over the whole run: the predicted and
 * updated state at every step, and one innovation log per sensor, in the
 * order of the sensors the filter was run with.
 */
struct FilterRun {
  FilterResult result;
  std::vector<SensorInnovationLog> sensor_logs;
};

/**
 * @brief Runs any filter over the whole run. It predicts once per step, then
 * every sensor that fires at that step updates in turn, each building on the
 * state the previous one left. The state recorded for a step is the one after
 * all of its sensors, its innovation and S are left empty, each sensor's
 * innovations are in its own log instead.
 * @param filter the filter to run
 * @param filter_config the filter's prior and motion model
 * @param inputs the input of every step
 * @param sensors the sensors, with their measurements and when they fire
 * @return the recorded states and the innovation log of every sensor
 */
FilterRun run_filter(BayesianFilter &filter, const FilterConfig &filter_config,
                     const std::vector<Input> &inputs,
                     const std::vector<Sensor> &sensors);

/**
 * @brief Runs any smoother backwards over a filter's recorded states. The
 * filter predicts step k with inputs[k], so the transition from step k to
 * step k + 1 uses inputs[k + 1].
 * @param smoother the smoother to run
 * @param smoother_config the smoother's prior and motion model
 * @param filter_result the filter's recorded predictions and updates
 * @param inputs the input of every step
 * @return the smoothed state at every step
 */
SmootherResult run_smoother(BayesianSmoother &smoother,
                            const SmootherConfig &smoother_config,
                            const FilterResult &filter_result,
                            const std::vector<Input> &inputs);

#endif
