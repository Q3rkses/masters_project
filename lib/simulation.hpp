/**
 * @file models.hpp
 * @brief Contains basic stochastic dynamical models of motion
 * used for simulation.
 */

#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <eigen3/Eigen/Eigen>
#include <random>

struct SimulationConfig {
  std::mt19937_64 &rng;
  double variance;
  double mean;
};

class GaussianWhiteNoise {
public:
  /**
   * @brief Constructor for the Gaussian White Noise model class.
   * @param model_config all configuration parameters that are
   * used by the constructor
   */
  explicit GaussianWhiteNoise(const SimulationConfig &simulation_config);

  /**
   * @brief Simulates a GWN process
   * @param timesteps, the amount of timesteps we simulate for
   * @return returns an array of GWN samples
   */
  Eigen::VectorXd simulate(const int timesteps);

private:
  /**
   * @brief utilizes GWN to create a sample
   * @return returns a sample from the GWN distribution
   */
  double sample_from_distribution();

  double mean_;          // the mean of the distribution
  double variance_;      // the variance of the distribution
  std::mt19937_64 &rng_; // the engine that draws from stochastic distributions
};

class RandomWalk {
public:
  /**
   * @brief Constructor for the RandomWalk class.
   * @param model_config all configuration parameters that are
   * used by the constructor
   */
  explicit RandomWalk(const SimulationConfig &simulation_config);

  /**
   * @brief Simulates a random walk by summing up samples from
   * a GWN proces
   * @param timesteps, the amount of timesteps we simulate for
   * @return returns an array of samples
   */
  Eigen::VectorXd simulate(int timesteps);

private:
  /**
   * @brief utilizes GWN to create a sample
   * @return returns a sample from the GWN distribution
   */
  double sample_from_distribution();

  /**
   * @brief propagates the system dynamics e.g by utilizing
   * a numerical integration method such as the explicit
   * euler method or RK4.
   * @return returns a sample from the GWN distribution
   */
  double propagate_dynamics(double x_previous);

  double mean_;                  // the mean of the distribution
  double variance_;              // the variance of the distribution
  double accumulated_variance_ = 0.0; // accumulated variance over k timesteps
  std::mt19937_64
      &rng_; // the engine that draws from stochastic distributions
};

class ConstantVelocity {
public:
  /**
   * @brief Constructor for the ConstantVelocity class.
   * @param model_config all configuration parameters that are
   * used by the constructor
   */
  explicit ConstantVelocity(const SimulationConfig &simulation_config);

  /**
   * @brief Simulates a 2d particle following the constant velocity model
   * @param timesteps, the amount of timesteps we simulate
   * @return returns an array of samples
   */

  // simulate(timesteps) -> array of 2d points

private:
  /**
   * @brief propagates the model by 1 timestep, could essentially be
   * a numeric integration algorithm like explicit euler or RK4
   * @return returns the next state given a previous state
   */

  // propagate_dynamics(x_previous)

  // velocity // the velocity at which the partice is traveling at
  std::mt19937_64 &rng; // the engine that draws from stochastic distributions
};

class CoordinatedTurn {
public:
  /**
   * @brief Constructor for the CoordinatedTurn class.
   * @param model_config all configuration parameters that are
   * used by the constructor
   */
  explicit CoordinatedTurn(const SimulationConfig &simulation_config);

  /**
   * @brief Simulates a 2d particle following the coordinated turn model
   * @param timesteps, the amount of timesteps we simulate for
   * @return returns an array of samples
   */

  // simulate(timesteps) -> array of 2d points

private:
  /**
   * @brief propagates the model by 1 timestep, could essentially be
   * a numeric integration algorithm like explicit euler or RK4
   * @return returns the next state given a previous state
   */

  // propagate_dynamics(x_previous)

  // velocity // the velocity at which the partice is traveling at
  // turn_rate // the turn rate of the particle
  std::mt19937_64 &rng; // the engine that draws from stochastic distributions
};

#endif
