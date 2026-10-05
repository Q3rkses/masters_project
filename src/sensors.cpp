#include "sensors.hpp"
#include "simulation.hpp"

SensorSchedule build_sensor_schedule(const YAML::Node &config,
                                     std::mt19937_64 &rng, const int timesteps,
                                     const double dt) {
  const double gnss_period_min = config["gnss_period_min"].as<double>();
  const double gnss_period_max = config["gnss_period_max"].as<double>();
  // a time window with no GNSS at all, e.g. to simulate an outage in the
  // middle of a run; defaults to never denied if not set in the config
  const double gnss_denied_start = config["gnss_denied_start"].as<double>(1e9);
  const double gnss_denied_end = config["gnss_denied_end"].as<double>(1e9);
  double next_gnss_time = sample_uniform(rng, gnss_period_min, gnss_period_max);

  const double magnetometer_period_min =
      config["magnetometer_period_min"].as<double>();
  const double magnetometer_period_max =
      config["magnetometer_period_max"].as<double>();
  double next_magnetometer_time =
      sample_uniform(rng, magnetometer_period_min, magnetometer_period_max);

  const double dvl_period_min = config["dvl_period_min"].as<double>();
  const double dvl_period_max = config["dvl_period_max"].as<double>();
  double next_dvl_time = sample_uniform(rng, dvl_period_min, dvl_period_max);

  SensorSchedule schedule{std::vector<bool>(timesteps, false),
                          std::vector<bool>(timesteps, false),
                          std::vector<bool>(timesteps, false)};
  for (int k = 0; k < timesteps; k++) {
    const double t = k * dt;

    const bool gnss_denied = t >= gnss_denied_start && t < gnss_denied_end;
    if (t >= next_gnss_time && !gnss_denied) {
      schedule.gnss[k] = true;
      next_gnss_time = t + sample_uniform(rng, gnss_period_min, gnss_period_max);
    }
    if (t >= next_magnetometer_time) {
      schedule.magnetometer[k] = true;
      next_magnetometer_time =
          t + sample_uniform(rng, magnetometer_period_min, magnetometer_period_max);
    }
    if (t >= next_dvl_time) {
      schedule.dvl[k] = true;
      next_dvl_time = t + sample_uniform(rng, dvl_period_min, dvl_period_max);
    }
  }
  return schedule;
}
