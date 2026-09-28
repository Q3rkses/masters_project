/**
 * @file nonlinear_models.hpp
 * @brief contains the nonlinear motion and measurement models
 */

#ifndef NONLINEAR_MODELS_HPP
#define NONLINEAR_MODELS_HPP

#include "models.hpp"
#include <Eigen/Dense>

/**
 * All values must be non-negative
 */
struct INS2DConfig {
  double dt;
  double gyro_time_constant;
  Eigen::Vector2d accelerometer_time_constant;
  double gyro_noise_variance;
  Eigen::Vector2d accelerometer_noise_variance;
  double gyro_white_noise_intensity;
  Eigen::Vector2d accelerometer_white_noise_intensity;
  double gyro_random_walk_variance;
  Eigen::Vector2d accelerometer_random_walk_variance;
};

class State {
public:
  /**
   * @brief Constructor for the State class.
   * @param state which is an Eigen vector of length either
   * 1,2,8 and later 15 which will decide which model we use.
   * used by the constructor
   */
  explicit State(const Eigen::VectorXd initial_state);

  /**
   * @brief packages the state parameters into an Eigen vectorXd
   * @return returns the state as an Eigen vectorXd
   */
  Eigen::VectorXd state_to_eigen();

  /**
   * @brief packages the state parameters into an Eigen vectorXd
   * @param the state as an Eigen VectorXd
   * @returns void, simply assigns the member variables
   */
  void eigen_to_state(Eigen::VectorXd eigen_state);

  double x_;
  double y_;
  double psi_;
  double u_;
  double v_;
  double bias_accelerometer_x_;
  double bias_accelerometer_y_;
  double bias_gyro_psi_;
};

class StrapdownINS2D final : public MotionModel {
public:
  /**
   * @brief Constructor for the StrapdownINS2D class.
   * @param dt the time between two IMU samples in seconds, which is
   * assumed to be fixed
   */
  explicit StrapdownINS2D(const INS2DConfig &config);

  /**
   * @brief Gets f for a 2d particle following a strapdown INS model.
   * The model has 8 states, [x, y, psi, u, w, b_x, b_y, b_psi].
   * I.e 2 for position, 2 for speed, 1 heading, 2 bias accel, 1 bias gyro
   * @param state, the state of the system
   * @param input, the IMU input to the system, [a_x, a_y, omega_psi]
   */
  Eigen::VectorXd f(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Gets F, the Jacobian of f evaluated at the state and input
   * @param state, the state of the system
   * @param input, the IMU input to the system
   */
  Eigen::MatrixXd F(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Gets Q, the process noise, which depends on dt, the state
   * and the input
   * @param state, the state of the system
   * @param input, the IMU input to the system
   */
  Eigen::MatrixXd Q(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief The composition operator of the 2D strapdown INS
   * model, which is defined in order to respect the manifold
   * that the state lies on.
   * @param state, the state of the system
   * @param delta, the pertrubation of the state
   * @return the composed state
   */
  virtual Eigen::VectorXd
  composition_plus(const Eigen::VectorXd &state,
                   const Eigen::VectorXd &delta) const override;

  /**
   * @brief The composition operator of the 2D strapdown INS
   * model, which is defined in order to respect the manifold
   * that the state lies on.
   * @param state_a, the state we subtract from
   * @param state_b, the state we subtract
   * @return the delta between the two states
   */
  virtual Eigen::VectorXd
  composition_minus(const Eigen::VectorXd &state_a,
                    const Eigen::VectorXd &state_b) const override;

private:
  double dt_;

  // White noise on the raw IMU readings (continuous-time intensities)
  Eigen::Vector2d accelerometer_white_noise_intensity_;
  double gyro_white_noise_intensity_;

  // Accelerometer bias, modeled as Gauss-Markov
  Eigen::Vector2d accelerometer_driving_noise_variance_;
  Eigen::Vector2d accelerometer_gauss_markov_variance_;
  Eigen::Vector2d accelerometer_gauss_markov_timeconstant_;

  // Gyro bias, modeled as Gauss-Markov
  double gyro_driving_noise_variance_;
  double gyro_gauss_markov_variance_;
  double gyro_gauss_markov_timeconstant_;

  // Accelerometer bias, simpler but more pessimistic alternative
  Eigen::Vector2d accelerometer_random_walk_variance_;
  // Gyro bias, simpler but more pessimistic alternative
  double gyro_random_walk_variance_;
};

#endif
