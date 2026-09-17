#include "simulation.hpp"
#include <Eigen/Dense>
#include <random>

GaussianWhiteNoise::GaussianWhiteNoise(
    const SimulationConfig &simulation_config)
    : mean_(simulation_config.mean), variance_(simulation_config.variance),
      rng_(simulation_config.rng) {};

double GaussianWhiteNoise::sample_from_distribution() {
  std::normal_distribution<double> GWN(this->mean_, std::sqrt(this->variance_));
  return GWN(this->rng_);
};

Eigen::VectorXd GaussianWhiteNoise::simulate(const int timesteps) {
  Eigen::VectorXd timeseries = Eigen::VectorXd::Zero(timesteps);

  for (int i = 0; i < timesteps; i++) {
    double sample = this->sample_from_distribution();
    timeseries(i) = sample;
  }

  return timeseries;
};

RandomWalk::RandomWalk(const SimulationConfig &simulation_config)
    : mean_(simulation_config.mean), variance_(simulation_config.variance),
      rng_(simulation_config.rng) {};

double RandomWalk::sample_from_distribution() {
  std::normal_distribution<double> GWN(this->mean_, std::sqrt(this->variance_));
  this->accumulated_variance_ += this->variance_;
  return GWN(this->rng_);
};

Eigen::VectorXd RandomWalk::simulate(const int timesteps) {
  Eigen::VectorXd timeseries = Eigen::VectorXd::Zero(timesteps);

  double state = 0;
  for (int i = 0; i < timesteps; i++) {
    double sample = this->sample_from_distribution();
    state += sample;
    timeseries(i) = state;
  }

  return timeseries;
};
