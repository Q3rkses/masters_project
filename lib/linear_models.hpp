/**
 * @file linear_models.hpp
 * @brief contains the linear motion and measurement models, where
 * f(x) = Fx and h(x) = Hx, so the Jacobians are the matrices themselves
 */

#ifndef LINEAR_MODELS_HPP
#define LINEAR_MODELS_HPP

#include "models.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

struct ModelConfig {
  Eigen::MatrixXd F;
  Eigen::MatrixXd H;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
};

class LinearMotionModel final : public MotionModel {
public:
  /**
   * @brief Constructor for the LinearMotionModel class, only F and Q of
   * the config are used.
   * @param model_config F must be square (n x n) and Q must be n x n
   * @throws std::invalid_argument if the dimensions do not fit
   */
  explicit LinearMotionModel(const ModelConfig &model_config);

  /**
   * @brief Gets f, which for a linear model is simply F x
   * @param state, the state of the system
   * @param input, the input to the system, which is ignored
   */
  Eigen::VectorXd f(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Gets F, which is the same matrix regardless of state and input
   * @param state, the state of the system, which is ignored
   * @param input, the input to the system, which is ignored
   */
  Eigen::MatrixXd F(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Gets Q, which is the same matrix regardless of state and input
   * @param state, the state of the system, which is ignored
   * @param input, the input to the system, which is ignored
   */
  Eigen::MatrixXd Q(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief implements the generic composition operator which will
   * for linear models not including angles in the state always be
   * the default + operator
   * @param state, the state of the system
   * @param delta, the pertrubation of the state
   * @return the composed state
   */
  Eigen::VectorXd composition_plus(const Eigen::VectorXd &state,
                                   const Eigen::VectorXd &delta) const override;

  /**
   * @brief implements the generic composition operator which will
   * for linear models not including angles in the state always be
   * the default - operator
   * @param state_a, the state we subtract from
   * @param state_b, the state we subtract
   * @return the delta between the two states
   */
  Eigen::VectorXd
  composition_minus(const Eigen::VectorXd &state_a,
                    const Eigen::VectorXd &state_b) const override;

private:
  Eigen::MatrixXd F_matrix_;
  Eigen::MatrixXd Q_matrix_;
};

class LinearMeasurementModel final : public MeasurementModel {
public:
  /**
   * @brief Constructor for the LinearMeasurementModel class, only H and R
   * of the config are used.
   * @param model_config H must be m x n and R must be m x m
   * @throws std::invalid_argument if the dimensions do not fit
   */
  explicit LinearMeasurementModel(const ModelConfig &model_config);

  /**
   * @brief Gets h, which for a linear model is simply H x
   * @param state, the state of the system
   * @param input, the input to the system, which is ignored
   */
  Eigen::VectorXd h(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Gets H, which is the same matrix regardless of state and input
   * @param state, the state of the system, which is ignored
   * @param input, the input to the system, which is ignored
   */
  Eigen::MatrixXd H(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Gets R, which is the same matrix regardless of state and input
   * @param state, the state of the system, which is ignored
   * @param input, the input to the system, which is ignored
   */
  Eigen::MatrixXd R(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief implements the generic composition operator which will
   * for linear models not including angles in the state always be
   * the default + operator
   * @param measurement, the measurement of the system
   * @param delta, the pertrubation of the state
   * @return the composed measurement
   */
  Eigen::VectorXd composition_plus(const Eigen::VectorXd &measurement,
                                   const Eigen::VectorXd &delta) const override;

  /**
   * @brief implements the generic composition operator which will
   * for linear models not including angles in the state always be
   * the default - operator
   * @param measurement_a, the measurement we subtract from
   * @param measurement_b, the measurement we subtract
   * @return the delta between the two measurements
   */
  Eigen::VectorXd
  composition_minus(const Eigen::VectorXd &measurement_a,
                    const Eigen::VectorXd &measurement_b) const override;

private:
  Eigen::MatrixXd H_matrix_;
  Eigen::MatrixXd R_matrix_;
};

#endif
