/**
 * @file results.hpp
 * @brief Contains the filter_result and smoother_result classes
 * that collect: P_predicted, P_updated, P_smoothed, x_predicted,
 * x_updated, x_smoothed during simulation and can later be exported
 * to a file, or can be read directly by a smoother
 */

#ifndef FILTER_RESULT_HPP
#define FILTER_RESULT_HPP

#include "bayesian_filter.hpp"
#include "bayesian_smoother.hpp"
#include <eigen3/Eigen/Eigen>
#include <string>
#include <vector>

class FilterResult {
public:
  /**
   * @brief Constructor for the Filter Result class.
   * @param filter_config all configuration parameters that are
   */
  explicit FilterResult(const FilterConfig &filter_config);

  /**
   * @brief records one timestep's predict/update pair. Must be called
   * once per timestep in order for to_csv to have intended behaviour
   */
  void add(const FilterPredict &prediction, const FilterUpdate &update);

  /**
   * @brief exports all recorded filter data to a csv file. One row per
   * timestep, with columns (state dim n, measurement dim m):
   * timestep,
   * x_predicted_0..x_predicted_{n-1},
   * P_predicted_0_0..P_predicted_{n-1}_{n-1} (row-major),
   * x_updated_0..x_updated_{n-1},
   * innovation_0..innovation_{m-1},
   * S_0_0..S_{m-1}_{m-1} (row-major),
   * P_updated_0_0..P_updated_{n-1}_{n-1} (row-major)
   * @param path where to write the csv file
   */
  void to_csv(const std::string &path) const;

private:
  std::vector<FilterPredict> predictions_; // one entry per timestep
  std::vector<FilterUpdate> updates_;      // one entry per timestep

  FilterConfig filter_config_; // F, Q, H, R used to produce this result
};

class SmootherResult {
public:
  /**
   * @brief Constructor for the Smoother Result class.
   * @param smoother_config all configuration parameters that are
   */
  explicit SmootherResult(const SmootherConfig &smoother_config);

  /**
   * @brief records one timestep's smoothed estimate. Must be called once
   * per timestep, in order, for to_csv() to produce a sensible file.
   */
  void add(const SmootherUpdate &smoothed);

  /**
   * @brief exports all recorded smoother data to a csv file. One row per
   * timestep, with columns (state dim n):
   * timestep,
   * x_smoothed_0..x_smoothed_{n-1},
   * P_smoothed_0_0..P_smoothed_{n-1}_{n-1} (row-major)
   * @param path where to write the csv file
   */
  void to_csv(const std::string &path) const;

private:
  std::vector<SmootherUpdate> smoothed_; // one entry per timestep

  SmootherConfig smoother_config_; // F, Q used to produce this result
};
#endif
