#include "sensors.hpp"
#include "simulation.hpp"

std::vector<std::vector<bool>>
build_sensor_schedules(const YAML::Node &config, std::mt19937_64 &rng,
                       const std::vector<std::string> &names,
                       const int timesteps, const double dt) {
  const int count = names.size();
  std::vector<double> period_min(count), period_max(count);
  std::vector<double> denied_start(count), denied_end(count);
  std::vector<double> next_time(count);

  for (int i = 0; i < count; i++) {
    period_min[i] = config[names[i] + "_period_min"].as<double>();
    period_max[i] = config[names[i] + "_period_max"].as<double>();
    // a sensor that is never denied has its window far outside the run
    denied_start[i] = config[names[i] + "_denied_start"].as<double>(1e9);
    denied_end[i] = config[names[i] + "_denied_end"].as<double>(1e9);
    next_time[i] = sample_uniform(rng, period_min[i], period_max[i]);
  }

  std::vector<std::vector<bool>> schedules(
      count, std::vector<bool>(timesteps, false));
  for (int k = 0; k < timesteps; k++) {
    const double t = k * dt;
    for (int i = 0; i < count; i++) {
      const bool denied = t >= denied_start[i] && t < denied_end[i];
      if (t >= next_time[i] && !denied) {
        schedules[i][k] = true;
        next_time[i] = t + sample_uniform(rng, period_min[i], period_max[i]);
      }
    }
  }
  return schedules;
}
