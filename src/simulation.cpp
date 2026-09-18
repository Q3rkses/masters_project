#include "simulation.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <random>
#include <stdexcept>
#include <vector>

Eigen::VectorXd sample_multivariate_normal(std::mt19937_64 &rng,
                                           const Eigen::VectorXd &mean,
                                           const Eigen::MatrixXd &covariance) {
  Eigen::LLT<Eigen::MatrixXd> llt(covariance);
  if (llt.info() != Eigen::Success) {
    throw std::runtime_error("covariance must be symmetric positive definite");
  }

  std::normal_distribution<double> standard_normal(0.0, 1.0);
  Eigen::VectorXd standard_normal_sample = Eigen::VectorXd::Zero(mean.size());
  for (int i = 0; i < mean.size(); i++) {
    standard_normal_sample[i] = standard_normal(rng);
  }
  return mean + llt.matrixL() * standard_normal_sample;
}

GaussianWhiteNoise::GaussianWhiteNoise(
    const SimulationConfig &simulation_config)
    : mean_(simulation_config.mean), covariance_(simulation_config.covariance),
      rng_(simulation_config.rng) {};

Eigen::VectorXd GaussianWhiteNoise::sample_from_distribution() {
  return sample_multivariate_normal(rng_, mean_, covariance_);
};

std::vector<Eigen::VectorXd> GaussianWhiteNoise::simulate(const int timesteps) {
  std::vector<Eigen::VectorXd> timeseries;

  for (int i = 0; i < timesteps; i++) {
    Eigen::VectorXd sample = sample_from_distribution();
    timeseries.push_back(sample);
  }

  return timeseries;
};

RandomWalk::RandomWalk(const SimulationConfig &simulation_config)
    : mean_(simulation_config.mean), covariance_(simulation_config.covariance),
      rng_(simulation_config.rng) {};

Eigen::VectorXd RandomWalk::sample_from_distribution() {
  return sample_multivariate_normal(rng_, mean_, covariance_);
};

std::vector<Eigen::VectorXd> RandomWalk::simulate(const int timesteps) {
  std::vector<Eigen::VectorXd> timeseries;

  Eigen::VectorXd state = Eigen::VectorXd::Zero(mean_.size());
  for (int i = 0; i < timesteps; i++) {
    state += sample_from_distribution();
    timeseries.push_back(state);
  }

  return timeseries;
};
