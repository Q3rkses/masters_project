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
#include <vector>
#include <yaml-cpp/yaml.h>

// which sensors fire at which step. Drawn once up front, so every filter
// that runs on this data sees exactly the same fixes. Three sensors, three
// schedules, all uniform. A step is only skipped by GNSS if it falls inside
// the denied window
struct SensorSchedule {
  std::vector<bool> gnss;
  std::vector<bool> magnetometer;
  std::vector<bool> dvl;
};

/**
 * @brief Draws the schedule for all three sensors from the config
 * @param config the parsed yaml config
 * @param rng the random engine
 * @param timesteps the amount of steps in the run
 * @param dt the time between two steps
 * @return one entry per step for each sensor, true if it fires
 */
SensorSchedule build_sensor_schedule(const YAML::Node &config,
                                     std::mt19937_64 &rng, const int timesteps,
                                     const double dt);

// everything the forward pass needs to know about one sensor
struct Sensor {
  std::shared_ptr<const MeasurementModel> model;
  const std::vector<Eigen::VectorXd> &measurements;
  const std::vector<bool> &due; // one entry per step
};

#endif
