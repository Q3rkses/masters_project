/**
 * @file scenarios.hpp
 * @brief The scenarios main can run. Each one simulates its own data from a
 * parsed config and runs the filters and smoothers over it, so main only
 * picks the scenario named in the config.
 */

#ifndef SCENARIOS_HPP
#define SCENARIOS_HPP

#include "models/nonlinear_models.hpp"
#include <Eigen/Dense>
#include <filesystem>
#include <yaml-cpp/yaml.h>

/**
 * @brief Reads a yaml list of numbers, which the strapdown INS scenarios
 * share, defined in the 2D scenario
 * @param node the yaml list
 * @return the elements of the list
 */
Eigen::VectorXd read_vector(const YAML::Node &node);

/**
 * @brief Reads the Levenberg-Marquardt parameters of the iterated methods, the
 * ones a config leaves out get defaults. Defined in the 2D scenario
 * @param config the parsed yaml config
 * @return the parameters
 */
LevenbergMarquardtConfig read_lm_config(const YAML::Node &config);

/**
 * @brief Simulates a dim-dimensional random walk that is observed directly,
 * runs a Kalman filter and an RTS smoother on it, and writes truth.csv,
 * filter.csv, smoother.csv and measurement_fixes.csv, which holds the
 * measurement at every step with its innovation and S. Expected keys: seed,
 * timesteps, dim, q, r, x0 (list of length dim), p0.
 * @param config the parsed yaml config
 * @param output_dir where the csv files are written
 */
void run_random_walk(const YAML::Node &config,
                     const std::filesystem::path &output_dir);

/**
 * @brief Simulates a trajectory through a 2D strapdown INS model with GNSS,
 * magnetometer and DVL fixes arriving at random intervals, runs the EKF and
 * ERTS and then the UKF and URTSS on the same data, and the IEKF with the
 * ERTS if the config sets run_iekf. Writes truth.csv, filter.csv and
 * smoother.csv, the same files prefixed ukf_ (and iekf_), and one fix file
 * per sensor and filter, e.g. gnss_fixes.csv.
 * @param config the parsed yaml config
 * @param output_dir where the csv files are written
 */
void run_strapdown_ins_2d(const YAML::Node &config,
                         const std::filesystem::path &output_dir);

/**
 * @brief Simulates a trajectory through a 3D strapdown INS model, with the
 * attitude as a unit quaternion and the covariance on the 15D error state,
 * with GNSS, magnetometer, DVL and altimeter fixes arriving at random
 * intervals. Runs the EKF and ERTS and then the UKF and URTSS on the same
 * data, and the IEKF with the ERTS if the config sets run_iekf. Writes
 * truth.csv, filter.csv and smoother.csv, the same files prefixed ukf_ (and
 * iekf_), and one fix file per sensor and filter, e.g. gnss_fixes.csv.
 * @param config the parsed yaml config
 * @param output_dir where the csv files are written
 */
void run_strapdown_ins_3d(const YAML::Node &config,
                          const std::filesystem::path &output_dir);

#endif
