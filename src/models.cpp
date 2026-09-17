#include "models.hpp"

MotionModel::MotionModel(const ModelConfig &model_config)
    : F(model_config.F), Q(model_config.Q) {}

MeasurementModel::MeasurementModel(const ModelConfig &model_config)
    : H(model_config.H), R(model_config.R) {}
