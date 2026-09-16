/**
 * @file models.hpp
 * @brief contains the motion, and measurement model classes
 */

#ifndef MODELS_HPP
#define MODELS_HPP

#include <eigen3/Eigen/Eigen>

class MotionModel {
public:
  /**
   * @brief Constructor for the Motion model class.
   * @param model_config all configuration parameters that are
   * used by the constructor
   */
  // explicit MotionModel(const ModelConfig& model_config);

private:
  // F
  // Q // the uncertainty of the Motion
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
  // H
  // R // the uncertainty of the Measurement
};

#endif
