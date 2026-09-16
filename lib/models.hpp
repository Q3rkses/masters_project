/**
 * @file models.hpp
 * @brief contains the motion, and measurement model classes
 */

#ifndef MODELS_HPP
#define MODELS_HPP

#include <eigen3/Eigen/Eigen>

// Can be expanded later to utilize a template for F and H s.t
// we can use the Jacobian when needed and the actual functions
// f and h, when possible in the EKF
class ModelConfig {

  explicit ModelConfig(Eigen::MatrixXd F, Eigen::MatrixXd H, Eigen::MatrixXd Q,
                       Eigen::MatrixXd R);

public:
  Eigen::MatrixXd F;
  Eigen::MatrixXd H;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
};

class MotionModel {
public:
  /**
   * @brief Constructor for the Motion model class.
   * @param model_config all configuration parameters that are
   * used by the constructor
   */
  explicit MotionModel(const ModelConfig &model_config);

private:
  Eigen::MatrixXd F;
  Eigen::MatrixXd Q; // the uncertainty of the Motion
};

class MeasurementModel {
public:
  /**
   * @brief Constructor for the Measurement model class.
   * @param model_config all configuration parameters that are
   * used by the constructor
   */
  // explicit Measurement(const ModelConfig& model_config);

private:
  Eigen::MatrixXd H;
  Eigen::MatrixXd R; // the uncertainty of the Measurement
};

#endif
