/**
 * @file sensors.hpp
 * @brief Contains the sensors a filter is fed by and the schedule that
 * decides at which steps each of them fires
 */

#ifndef SENSORS_HPP
#define SENSORS_HPP

#include "models/models.hpp"
#include <Eigen/Dense>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

/**
 * @brief Everything the estimation needs to know about one sensor.
 */
struct Sensor {
  std::string name; // names the sensor in the config keys and the output files
  std::shared_ptr<const MeasurementModel> model;
  Eigen::MatrixXd noise_covariance; // covariance of the noise the data was simulated with
  std::vector<Eigen::VectorXd> measurements; // one per step, whether or not the sensor fires there
  std::vector<bool> due;                     // one per step, true where the sensor fires
};

/**
 * @brief Draws at which steps each sensor fires. Sensor `name` reads
 * `<name>_period_min` and `<name>_period_max` from the config, the time
 * between two of its fixes is uniform between the two. The optional keys
 * `<name>_denied_start` and `<name>_denied_end` give a time window with no
 * fixes from that sensor, e.g. to simulate an outage in the middle of a run.
 * All sensors draw from the same random engine, step by step in the order of
 * `names`, so that order decides the schedule a seed gives.
 * @param config the parsed yaml config
 * @param rng the random engine
 * @param names the sensors, in the order they draw
 * @param timesteps the amount of steps in the run
 * @param dt the time between two steps
 * @return for each sensor, one entry per step that is true if it fires
 */
std::vector<std::vector<bool>>
build_sensor_schedules(const YAML::Node &config, std::mt19937_64 &rng,
                       const std::vector<std::string> &names,
                       const int timesteps, const double dt);

#endif
