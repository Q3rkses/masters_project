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
#include <vector>

class FilterResult {
public:
  /**
   * @brief Constructor for the Filter Result class.
   * @param filter_config all configuration parameters that are
   */
  // explicit FilterResult(const FilterConfig& filter_config);

  /**
   * @brief a function to export all filter data to a csv file
   * @param filter_config all configuration parameters that are
   */
  // void to_csv(const std::string& path) const;

private:
  std::vector<FilterPredict> predictions; // one entry per timestep
  std::vector<FilterUpdate> updates;      // one entry per timestep

  FilterConfig filter_config; // F, Q, H, R used to produce this result
};

class SmootherResult {
public:
  /**
   * @brief Constructor for the Smoother Result class.
   * @param smoother_config all configuration parameters that are
   */
  // explicit SmootherResult(const SmootherConfig& smoother_config);

  /**
   * @brief a function to export all smoother data to a csv file
   * @param filter_config all configuration parameters that are
   */
  // void to_csv(const std::string& path) const;

private:
  std::vector<SmootherUpdate> smoothed; // one entry per timestep

  SmootherConfig smoother_config; // F, Q used to produce this result
};
#endif
