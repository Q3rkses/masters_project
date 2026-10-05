/**
 * @file manifold.hpp
 * @brief contains the interface for the composition operators that
 * the motion and measurement models define in order to respect the
 * manifold their vectors live on
 */

#ifndef MANIFOLD_HPP
#define MANIFOLD_HPP

#include <Eigen/Dense>

class Manifold {
public:
  virtual ~Manifold() = default;

  /**
   * @brief The generic composition operator that the model
   * has to define in order to respect the manifold
   * @param state, the state of the system
   * @param delta, the pertrubation of the state
   * @return the composed state
   */
  virtual Eigen::VectorXd
  composition_plus(const Eigen::VectorXd &state,
                   const Eigen::VectorXd &delta) const = 0;

  /**
   * @brief The generic composition operator that the model
   * has to define in order to respect the manifold
   * @param state_a, the state we subtract from
   * @param state_b, the state we subtract
   * @return the delta between the two states
   */
  virtual Eigen::VectorXd
  composition_minus(const Eigen::VectorXd &state_a,
                    const Eigen::VectorXd &state_b) const = 0;
};

#endif
