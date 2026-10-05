/**
 * @file scenarios.hpp
 * @brief File that is meant to offload scenario building from main,
 * such that the only functionality of main is to run.
 * config file and hands the parsed config to the matching runner.
 */

#ifndef SCENARIOS_HPP
#define SCENARIOS_HPP

#include <filesystem>
#include <yaml-cpp/yaml.h>

/**
 * @brief Simulates, runs the a filter and smoother on a dim-dimensional random
 * walk that is observed directly, and writes truth.csv, filter.csv and
 * smoother.csv. Expected keys: seed, timesteps, dim, q, r, x0 (list of length
 * dim), p0.
 * @param config the parsed yaml config
 * @param output_dir where the csv files are written
 */
void run_random_walk(const YAML::Node &config,
                     const std::filesystem::path &output_dir);

/**
 * @brief Simulates a trajectory through a 2D strapdown INS model, runs the
 * EKF forward pass (with GNSS arriving at random intervals) and the ERTS
 * backward pass, then the UKF and URTSS on the same data. Writes truth.csv,
 * filter.csv, smoother.csv, ukf_filter.csv and ukf_smoother.csv, plus a
 * set of fix files per filter, the UKF's prefixed with ukf_.
 * @param config the parsed yaml config
 * @param output_dir where the csv files are written
 */
void run_strapdown_ins_2d(const YAML::Node &config,
                         const std::filesystem::path &output_dir);

#endif
