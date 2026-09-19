/**
 * @file nonlinear_models.hpp
 * @brief contains the nonlinear motion and measurement models
 */

#ifndef NONLINEAR_MODELS_HPP
#define NONLINEAR_MODELS_HPP

#include "models.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

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
  double u_;
  double v_;
  double psi_;
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
   * @throws std::invalid_argument if dt is not positive
   */
  explicit StrapdownINS2D(double dt);

  /**
   * @brief Gets f for a 2d particle following a strapdown INS model.
   * The model has 8 states, [x, y, u, w, psi, b_x, b_y, b_psi].
   * I.e 2 for position, 2 for speed, 1 heading, 2 bias accel, 1 bias gyro
   * @param state, the state of the system
   * @param input, the IMU input to the system, [a_x, a_y, omega_psi]
   * @throws std::logic_error since it is not implemented yet
   */
  Eigen::VectorXd f(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Gets F, the Jacobian of f evaluated at the state and input
   * @param state, the state of the system
   * @param input, the IMU input to the system
   * @throws std::logic_error since it is not implemented yet
   */
  Eigen::MatrixXd F(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Gets Q, the process noise, which depends on dt, the state
   * and the input
   * @param state, the state of the system
   * @param input, the IMU input to the system
   * @throws std::logic_error since it is not implemented yet
   */
  Eigen::MatrixXd Q(const Eigen::VectorXd &state,
                    const Input &input) const override;

private:
  double dt_;
  // the noise densities and the Gauss-Markov parameters (time constants and
  // sigmas) will be added here
};

#endif
