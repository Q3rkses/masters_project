/**
 * @file models.hpp
 * @brief contains the interfaces for the motion and measurement models,
 * and the Input class that is handed to them
 */

#ifndef MODELS_HPP
#define MODELS_HPP

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

class Input {
public:
  /**
   * @brief Default constructor for the Input class, an input of all zeros.
   * Used by scenarios without an input, such as the random walk.
   */
  Input();

  /**
   * @brief Constructor for the Input class.
   * @param initial_input which is an Eigen vector of length 3,
   * [a_x, a_y, omega_psi]
   */
  explicit Input(const Eigen::VectorXd initial_input);

  /**
   * @brief packages the input parameters into an Eigen vectorXd
   * @return returns the input as an Eigen vectorXd, [a_x, a_y, omega_psi]
   */
  Eigen::VectorXd input_to_eigen() const;

  /**
   * @brief assigns the input parameters from an Eigen vectorXd
   * @param eigen_input the input as an Eigen VectorXd, [a_x, a_y, omega_psi]
   * @returns void, simply assigns the member variables
   */
  void eigen_to_input(Eigen::VectorXd eigen_input);

  double a_x_;
  double a_y_;
  double omega_psi_;
};

class MotionModel {
public:
  virtual ~MotionModel() = default;

  /**
   * @brief Gets f, which is the possibly nonlinear
   * motion function
   * @param state, the state of the system
   * @param input, the input to the system
   */
  virtual Eigen::VectorXd f(const Eigen::VectorXd &state,
                            const Input &input) const = 0;
  /**
   * @brief Gets F, which the Jacobian of f, or in the linear
   * case it is simply the matrix F of the motion model
   * @param state, the state of the system
   * @param input, the input to the system
   */
  virtual Eigen::MatrixXd F(const Eigen::VectorXd &state,
                            const Input &input) const = 0;

  /**
   * @brief Gets Q which is the process noise
   * @param state, the state of the system
   * @param input, the input to the system
   */
  virtual Eigen::MatrixXd Q(const Eigen::VectorXd &state,
                            const Input &input) const = 0;

  /**
   * @brief The generic composition operator that the motion
   * model has to define in order to respect the manifold
   * @param state, the state of the system
   * @param delta, the pertrubation of the state
   * @return the composed state
   */
  virtual Eigen::VectorXd
  composition_plus(const Eigen::VectorXd &state,
                   const Eigen::VectorXd &delta) const = 0;

  /**
   * @brief The generic composition operator that the motion
   * model has to define in order to respect the manifold
   * @param state_a, the state we subtract from
   * @param state_b, the state we subtract
   * @return the delta between the two states
   */
  virtual Eigen::VectorXd
  composition_minus(const Eigen::VectorXd &state_a,
                    const Eigen::VectorXd &state_b) const = 0;
};

class MeasurementModel {
public:
  virtual ~MeasurementModel() = default;

  /**
   * @brief Gets h, which is the possibly nonlinear
   * measurement function
   * @param state, the state of the system
   * @param input, the input to the system
   */
  virtual Eigen::VectorXd h(const Eigen::VectorXd &state,
                            const Input &input) const = 0;

  /**
   * @brief Gets H, which the Jacobian of h, or in the linear
   * case it is simply the matrix H of the measurement model
   * @param state, the state of the system
   * @param input, the input to the system
   */
  virtual Eigen::MatrixXd H(const Eigen::VectorXd &state,
                            const Input &input) const = 0;

  /**
   * @brief Gets R which is the measurement noise
   * @param state, the state of the system
   * @param input, the input to the system
   */
  virtual Eigen::MatrixXd R(const Eigen::VectorXd &state,
                            const Input &input) const = 0;

  /**
   * @brief The generic composition operator that the motion
   * model has to define in order to respect the manifold
   * @param state, the state of the system
   * @param delta, the pertrubation of the state
   * @return the composed state
   */
  virtual Eigen::VectorXd
  composition_plus(const Eigen::VectorXd &state,
                   const Eigen::VectorXd &delta) const = 0;

  /**
   * @brief The generic composition operator that the motion
   * model has to define in order to respect the manifold
   * @param state_a, the state we subtract from
   * @param state_b, the state we subtract
   * @return the delta between the two states
   */
  virtual Eigen::VectorXd
  composition_minus(const Eigen::VectorXd &state_a,
                    const Eigen::VectorXd &state_b) const = 0;
};

#endif
