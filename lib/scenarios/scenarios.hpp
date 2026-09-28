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

#endif
