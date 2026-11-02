/**
 * @file nonlinear_models.hpp
 * @brief contains the nonlinear motion models of the 2D and 3D strapdown INS,
 * and the configs they share with the iterated methods
 */

#ifndef NONLINEAR_MODELS_HPP
#define NONLINEAR_MODELS_HPP

#include "models/models.hpp"
#include <Eigen/Dense>

/**
 * Configuration values for the LM method
 */
struct LevenbergMarquardtConfig {
  double lambda_initial;
  double lambda_min;
  double lambda_max;
  double lambda_factor; // lambda is multiplied on a rejection, divided on an accept
  int max_iter;
  int max_rejections;
  double step_tolerance;
  double cost_relative_tolerance;
};

/**
 * Configuration values for the 2D INS case
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
   */
  explicit StrapdownINS2D(const INS2DConfig &config);

  /**
   * @brief Gets f for a 2d particle following a strapdown INS model.
   * The model has 8 states, [x, y, u, v, psi, b_x, b_y, b_psi].
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


/**
 * Configuration values for the 3D INS case
 */
struct INS3DConfig {
  double dt;
  Eigen::Vector3d gravity; // in the navigation frame
  Eigen::Vector3d gyro_time_constant;
  Eigen::Vector3d accelerometer_time_constant;
  Eigen::Vector3d gyro_noise_variance;
  Eigen::Vector3d accelerometer_noise_variance;
  Eigen::Vector3d gyro_white_noise_intensity;
  Eigen::Vector3d accelerometer_white_noise_intensity;
};

class State3D {
public:
  /**
   * @brief Constructor for the State3D class.
   * @param state which is an Eigen vector of length 16,
   * [p, v, q, b_a, b_g], with the quaternion as [w, x, y, z]
   */
  explicit State3D(const Eigen::VectorXd initial_state);

  /**
   * @brief packages the state parameters into an Eigen vectorXd
   * @return returns the state as an Eigen vectorXd
   */
  Eigen::VectorXd state_to_eigen() const;

  /**
   * @brief assigns the state parameters from an Eigen vectorXd
   * @param the state as an Eigen VectorXd
   * @returns void, simply assigns the member variables
   */
  void eigen_to_state(Eigen::VectorXd eigen_state);

  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Quaterniond attitude_; // body to navigation frame
  Eigen::Vector3d bias_accelerometer_;
  Eigen::Vector3d bias_gyro_;
};

/**
 * @brief The 3D strapdown INS. The state has 16 elements, the attitude is a
 * unit quaternion and therefore overparametrised, so the covariance, F and Q
 * live on the 15D error state [dp, dv, dtheta, db_a, db_g] instead, where the
 * attitude error is a rotation vector in the body frame, q (x) Exp(dtheta).
 */
class StrapdownINS3D final : public MotionModel {
public:
  /**
   * @brief Constructor for the StrapdownINS3D class.
   * @param config the time between two IMU samples in seconds, which is
   * assumed to be fixed, gravity and the noise of the IMU
   */
  explicit StrapdownINS3D(const INS3DConfig &config);

  /**
   * @brief Gets f for a 3D particle following a strapdown INS model.
   * The model has 16 states, [x, y, z, u, v, w, q_w, q_x, q_y, q_z, b_ax,
   * b_ay, b_az, b_gx, b_gy, b_gz]. I.e 3 for position, 3 for velocity,
   * 4 for the attitude quaternion, 3 bias accel, 3 bias gyro
   * @param state, the state of the system
   * @param input, the IMU input to the system, [a_x, a_y, a_z, omega_x,
   * omega_y, omega_z]
   */
  Eigen::VectorXd f(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Gets F, the Jacobian of f evaluated at the state and input. It
   * is 15x15, it maps the error state to the error state, where the
   * attitude error is applied as q (x) Exp(dtheta)
   * @param state, the state of the system
   * @param input, the IMU input to the system
   */
  Eigen::MatrixXd F(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief Gets Q, the process noise of the error state, 15x15, which
   * depends on dt, the state and the input
   * @param state, the state of the system
   * @param input, the IMU input to the system
   */
  Eigen::MatrixXd Q(const Eigen::VectorXd &state,
                    const Input &input) const override;

  /**
   * @brief The composition operator of the 3D strapdown INS
   * model, which is defined in order to respect the manifold
   * that the state lies on. The state is 16D, the delta is 15D.
   * @param state, the state of the system
   * @param delta, the pertrubation of the error state
   * @return the composed state
   */
  virtual Eigen::VectorXd
  composition_plus(const Eigen::VectorXd &state,
                   const Eigen::VectorXd &delta) const override;

  /**
   * @brief The composition operator of the 3D strapdown INS
   * model, which is defined in order to respect the manifold
   * that the state lies on. The states are 16D, the delta is 15D.
   * @param state_a, the state we subtract from
   * @param state_b, the state we subtract
   * @return the delta between the two states, in the error state
   */
  virtual Eigen::VectorXd
  composition_minus(const Eigen::VectorXd &state_a,
                    const Eigen::VectorXd &state_b) const override;

private:
  double dt_;
  Eigen::Vector3d gravity_;

  // White noise on the raw IMU readings (continuous-time intensities)
  Eigen::Vector3d accelerometer_white_noise_intensity_;
  Eigen::Vector3d gyro_white_noise_intensity_;

  // Accelerometer bias, modeled as Gauss-Markov
  Eigen::Vector3d accelerometer_driving_noise_variance_;
  Eigen::Vector3d accelerometer_gauss_markov_variance_;
  Eigen::Vector3d accelerometer_gauss_markov_timeconstant_;

  // Gyro bias, modeled as Gauss-Markov
  Eigen::Vector3d gyro_driving_noise_variance_;
  Eigen::Vector3d gyro_gauss_markov_variance_;
  Eigen::Vector3d gyro_gauss_markov_timeconstant_;
};

#endif
