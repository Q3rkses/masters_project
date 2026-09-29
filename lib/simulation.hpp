/**
 * @file models.hpp
 * @brief Contains basic stochastic dynamical models of motion
 * used for simulation.
 */

#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <random>
#include <vector>

struct SimulationConfig {
  std::mt19937_64 &rng;
  Eigen::MatrixXd covariance;
  Eigen::VectorXd mean;
  Eigen::VectorXd timeconstant;
};

/**
 * @brief Samples from a multivaraite standard normal distribution
 * then utilizes cholesky L and mean to convert the samples to
 * the normal distribution we are looking for
 * @param &rng, the random engine
 * @param &mean, the mean of the desired distribution
 * @param &covariance, the covariance of the desired distribution
 * @return returns an array of GWN samples
 */
Eigen::VectorXd sample_multivariate_normal(std::mt19937_64 &rng,
                                           const Eigen::VectorXd &mean,
                                           const Eigen::MatrixXd &covariance);

/**
 * @brief Samples from a continuous uniform distribution on [low, high).
 * @param &rng, the random engine
 * @param low, the lower bound of the interval, inclusive
 * @param high, the upper bound of the interval, exclusive
 * @return a single sample drawn uniformly from [low, high)
 */
double sample_uniform(std::mt19937_64 &rng, double low, double high);

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
  std::vector<Eigen::VectorXd> simulate(const int timesteps) const;

private:
  /**
   * @brief utilizes GWN to create a sample
   * @return returns a sample from the GWN distribution
   */
  Eigen::VectorXd sample_from_distribution() const;

  const Eigen::VectorXd mean_;       // the mean of the distribution
  const Eigen::MatrixXd covariance_; // the variance of the distribution
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
  std::vector<Eigen::VectorXd> simulate(const int timesteps) const;

private:
  /**
   * @brief utilizes GWN to create a sample
   * @return returns a sample from the GWN distribution
   */
  Eigen::VectorXd sample_from_distribution() const;

  /**
   * @brief propagates the system dynamics e.g by utilizing
   * a numerical integration method such as the explicit
   * euler method or RK4.
   * @return returns a sample from the GWN distribution
   */
  Eigen::VectorXd propagate_dynamics(const Eigen::VectorXd x_previous);

  const Eigen::VectorXd mean_;       // the mean of the distribution
  const Eigen::MatrixXd covariance_; // the variance of the distribution
  std::mt19937_64 &rng_; // the engine that draws from stochastic distributions
};

class GaussMarkov {
public:
  /**
   * @brief Constructor for the GaussMarkov class. Precomputes the decay and
   * the driving noise covariance
   * @param dt, the discretization time interval
   */
  GaussMarkov(const SimulationConfig &simulation_config, const double dt);

  /**
   * @brief Simulates a Gauss Markov process by decaying the previous sample
   * and adding that step's driving noise.
   * @param timesteps, the amount of timesteps we simulate for
   * @return returns an array of samples
   */
  std::vector<Eigen::VectorXd> simulate(const int timesteps) const;

private:
  /**
   * @brief draws the driving noise for one step. A Gauss-Markov process
   * with stationary variance sigma^2 and time constant T is driven, over a
   * step dt, by noise with variance sigma^2 * (1 - exp(-2dt/T))
   */
  Eigen::VectorXd sample_from_distribution() const;

  /**
   * @brief propagates the system dynamics: exponential decay toward zero
   * with time constant T, i.e. x_next = exp(-dt/T) * x_previous
   * @param x_previous, the previous sample
   * @return the decayed previous sample, before that step's noise is added
   */
  Eigen::VectorXd propagate_dynamics(const Eigen::VectorXd &x_previous) const;

  const Eigen::VectorXd mean_;
  const Eigen::VectorXd decay_; // exp(-dt/T), per axis
  const Eigen::MatrixXd
      driving_covariance_; // sigma^2 * (1 - decay^2), per axis
  std::mt19937_64 &rng_; // the engine that draws from stochastic distributions
};

#endif
