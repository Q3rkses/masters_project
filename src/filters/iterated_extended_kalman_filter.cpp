#include "filters/iterated_extended_kalman_filter.hpp"
#include "filters/bayesian_filter.hpp"
#include "models/models.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>

IteratedExtendedKalmanFilter::IteratedExtendedKalmanFilter(
    const FilterConfig &filter_config,
    const LevenbergMarquardtConfig &lm_config)
    : motion_model_(filter_config.motion_model), lm_config(lm_config) {
  if (!motion_model_) {
    throw std::invalid_argument(
        "IteratedExtendedKalmanFilter: the filter config needs a motion model");
  }
  if (lm_config.lambda_factor <= 1.0) {
    throw std::invalid_argument(
        "IteratedExtendedKalmanFilter: lambda_factor has to be above 1");
  }
};

FilterPredict
IteratedExtendedKalmanFilter::predict(const Eigen::VectorXd &x_current,
                                      const Eigen::MatrixXd &P_current,
                                      const Input &input) {

  Eigen::MatrixXd F = motion_model_->F(x_current, input);
  Eigen::VectorXd x_predicted = motion_model_->f(x_current, input);
  Eigen::MatrixXd P_predicted =
      F * P_current * F.transpose() + motion_model_->Q(x_current, input);
  FilterPredict filter_prediction{x_predicted, P_predicted};
  return filter_prediction;
}

double IteratedExtendedKalmanFilter::cost(
    const Eigen::VectorXd &x, const Eigen::VectorXd &x_predicted,
    const Eigen::LDLT<Eigen::MatrixXd> &P_predicted_ldlt,
    const Eigen::VectorXd &z_current,
    const Eigen::LDLT<Eigen::MatrixXd> &R_ldlt, const Input &input,
    const MeasurementModel &measurement_model) const {

  // Cost function from Sarkka, Bayesian Filtering and Smoothing, chapter 7.
  Eigen::VectorXd measurement_residual = measurement_model.composition_minus(
      z_current, measurement_model.h(x, input));
  Eigen::VectorXd model_residual =
      motion_model_->composition_minus(x, x_predicted);

  return 0.5 * (measurement_residual.dot(R_ldlt.solve(measurement_residual)) +
                model_residual.dot(P_predicted_ldlt.solve(model_residual)));
}

FilterUpdate IteratedExtendedKalmanFilter::update(
    const Eigen::VectorXd &x_predicted, const Eigen::VectorXd &z_current,
    const Eigen::MatrixXd &P_predicted, const Input &input,
    const std::shared_ptr<const MeasurementModel> &measurement_model) {
  if (!measurement_model) {
    throw std::invalid_argument(
        "IteratedExtendedKalmanFilter::update: measurement model is null");
  }

  Eigen::MatrixXd H = measurement_model->H(x_predicted, input);
  Eigen::MatrixXd R = measurement_model->R(x_predicted, input);

  // Innovation and S are reported the way the EKF reports them, at the
  // prediction, so the NIS stays comparable with the other filters.
  Eigen::VectorXd innovation = measurement_model->composition_minus(
      z_current, measurement_model->h(x_predicted, input));
  Eigen::MatrixXd S = H * P_predicted * H.transpose() + R;

  Eigen::MatrixXd I =
      Eigen::MatrixXd::Identity(P_predicted.rows(), P_predicted.rows());

  // factorise once, only the cost needs them
  Eigen::LDLT<Eigen::MatrixXd> P_predicted_LDLT = P_predicted.ldlt();
  Eigen::LDLT<Eigen::MatrixXd> R_LDLT = R.ldlt();

  double lambda = lm_config.lambda_initial;
  int rejections = 0;

  // the iteration starts at the prediction, as in the EKF
  Eigen::VectorXd x_i_previous = x_predicted;
  double current_cost = cost(x_i_previous, x_predicted, P_predicted_LDLT,
                             z_current, R_LDLT, input, *measurement_model);

  for (int i = 0; i < lm_config.max_iter; ++i) {
    Eigen::MatrixXd H_i = measurement_model->H(x_i_previous, input);
    Eigen::MatrixXd R_i = measurement_model->R(x_i_previous, input);

    // use the proper composition operation, rather than regular +, -
    Eigen::VectorXd innovation_i = measurement_model->composition_minus(
        z_current, measurement_model->h(x_i_previous, input));
    Eigen::VectorXd prior_delta =
        motion_model_->composition_minus(x_i_previous, x_predicted);

    // Levenberg-Marquardt damping of lambda * P^-1 in the hessian
    Eigen::MatrixXd P_damped = P_predicted / (1.0 + lambda);
    Eigen::MatrixXd PHt_i = P_damped * H_i.transpose();
    Eigen::MatrixXd S_i = H_i * PHt_i + R_i;
    Eigen::MatrixXd W_i = S_i.ldlt().solve(PHt_i.transpose()).transpose();

    // the step is taken from the current iterate, not from x_predicted
    Eigen::VectorXd delta =
        W_i * innovation_i - (I - W_i * H_i) * prior_delta / (1.0 + lambda);
    Eigen::VectorXd x_candidate =
        motion_model_->composition_plus(x_i_previous, delta);

    double candidate_cost = cost(x_candidate, x_predicted, P_predicted_LDLT,
                                 z_current, R_LDLT, input, *measurement_model);

    // accept step if cost went down, and trust the linear model more
    if (candidate_cost < current_cost) {
      double relative_decrease = (current_cost - candidate_cost) / current_cost;
      x_i_previous = x_candidate;
      current_cost = candidate_cost;
      lambda = std::max(lambda / lm_config.lambda_factor, lm_config.lambda_min);
      rejections = 0;

      // check if we have converged
      if (std::sqrt(delta.dot(P_predicted_LDLT.solve(delta))) <
              lm_config.step_tolerance ||
          relative_decrease < lm_config.cost_relative_tolerance) {
        break;
      }
    }
    // reject step if cost went up, stay put and trust the linear model less
    else {
      lambda *= lm_config.lambda_factor;
      rejections++;
      if (lambda > lm_config.lambda_max ||
          rejections >= lm_config.max_rejections) {
        break;
      }
    }
  }

  // The posterior uses the undamped gain at the final iterate, otherwise it
  // would be inflated by lambda.
  Eigen::VectorXd x_updated = x_i_previous;
  Eigen::MatrixXd H_i = measurement_model->H(x_updated, input);
  Eigen::MatrixXd R_i = measurement_model->R(x_updated, input);
  Eigen::MatrixXd PHt_i = P_predicted * H_i.transpose();
  Eigen::MatrixXd S_i = H_i * PHt_i + R_i;
  Eigen::MatrixXd W_i = S_i.ldlt().solve(PHt_i.transpose()).transpose();

  // Joseph form: (I - WH) P (I - WH)^T + W R W^T. Algebraically equal to the
  // textbook (I - WH) P, but always positive definite
  Eigen::MatrixXd IWH = I - W_i * H_i;
  Eigen::MatrixXd P_updated =
      IWH * P_predicted * IWH.transpose() + W_i * R_i * W_i.transpose();

  FilterUpdate filter_update{x_updated, innovation, S, P_updated};
  return filter_update;
}
