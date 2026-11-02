/**
 * @file file_utilities.hpp
 * @brief Contains the helpers that write the results of a run to disk
 */

#ifndef FILE_UTILITIES_HPP
#define FILE_UTILITIES_HPP

#include <Eigen/Dense>
#include <filesystem>
#include <vector>

/**
 * @brief writes one row per fix: timestep, the raw measurement, the
 * innovation and the innovation covariance S. The header is always written
 * for `dim`, even with zero fixes (e.g. a sensor disabled for the whole
 * run), so downstream readers always see the expected columns.
 * @param path, where to write the CSV
 * @param dim, the measurement dimension of this sensor
 * @param fix_steps, the timestep of each fix
 * @param measurements, the raw measurement at each fix
 * @param innovations, the innovation at each fix
 * @param S, the innovation covariance at each fix
 */
void write_fix_csv(const std::filesystem::path &path, const int dim,
                   const std::vector<int> &fix_steps,
                   const std::vector<Eigen::VectorXd> &measurements,
                   const std::vector<Eigen::VectorXd> &innovations,
                   const std::vector<Eigen::MatrixXd> &S);

#endif
