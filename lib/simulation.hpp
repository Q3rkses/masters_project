
/**
 * @file models.hpp
 * @brief Contains basic stochastic dynamical models of motion
 * used for simulation.
 */

#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <eigen3/Eigen/Eigen>

class GaussianWhiteNoise {
public:
  /**
   * @brief Constructor for the Gaussian White Noise model class.
   * @param model_config all configuration parameters that are
   * used by the constructor
   */
  // explicit GaussianWhiteNoise(const SimulationConfig& simulation_config);

  /**
   * @brief Simulates a GWN process
   * @param timesteps, the amount of timesteps we simulate for
   * @return returns an array of GWN samples
   */

  // simulate(timesteps) -> array of 2d points

private:
  /**
   * @brief utilizes GWN to create a sample
   * @return returns a sample from the GWN distribution
   */

  // sample_from_distribution() -> singular GWN sample

  // mean // the mean of the distribution
  // variance // the variance of the distribution
};

class RandomWalk {
public:
  /**
   * @brief Constructor for the RandomWalk class.
   * @param model_config all configuration parameters that are
   * used by the constructor
   */
  // explicit RandomWalk(const SimulationConfig& simulation_config);

  /**
   * @brief Simulates a random walk by summing up samples from
   * a GWN proces
   * @param timesteps, the amount of timesteps we simulate for
   * @return returns an array of samples
   */

  // simulate(timesteps) -> array of 2d points

private:
  /**
   * @brief utilizes GWN to create a sample
   * @return returns a sample from the GWN distribution
   */

  // sample_from_distribution() -> singular GWN sample
  // propagate_dynamics(x_previous)

  // mean // the mean of the GWN distribution
  // GWN_variance // the variance of the GWN distribution
  // accumulated_variance // the accumulated variance over the entire
  // random walk process
};

class ConstantVelocity {
public:
  /**
   * @brief Constructor for the ConstantVelocity class.
   * @param model_config all configuration parameters that are
   * used by the constructor
   */
  // explicit ConstantVelocity(const SimulationConfig& simulation_config);

  /**
   * @brief Simulates a 2d particle following the constant velocity model
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
};

class CoordinatedTurn {
public:
  /**
   * @brief Constructor for the CoordinatedTurn class.
   * @param model_config all configuration parameters that are
   * used by the constructor
   */
  // explicit CoordinatedTurn(const SimulationConfig& simulation_config);

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
};

#endif
