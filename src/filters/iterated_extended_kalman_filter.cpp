#include "filters/iterated_extended_kalman_filter.hpp"
#include "filters/bayesian_filter.hpp"
#include "models/models.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Cholesky/LDLT.h>
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <stdexcept>

IteratedExtendedKalmanFilter::IteratedExtendedKalmanFilter(
    const FilterConfig &filter_config,
    const LevenbergMarquardtConfig &lm_config)
    : motion_model_(filter_config.motion_model), lm_config(lm_config) {
  if (!motion_model_) {
    throw std::invalid_argument(
        "IteratedExtendedKalmanFilter: the filter config needs a motion model");
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

  // Cost function from Sarakka Bayesian Smoothing and Filtering chapter 7.
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

  Eigen::VectorXd z_predicted = measurement_model->h(x_predicted, input);

  // use the proper composition operation, rather than regular +, - operations
  Eigen::VectorXd innovation =
      measurement_model->composition_minus(z_current, z_predicted);

  Eigen::MatrixXd PHt = P_predicted * H.transpose();
  Eigen::MatrixXd S = H * PHt + R;

  // W = PHt * S^-1, but obtained by solving S * W^T = PHt^T rather than
  // invering because it is cheaper and more stable
  Eigen::MatrixXd W = S.ldlt().solve(PHt.transpose()).transpose();

  // First estimate is the plain EKF update.
  Eigen::VectorXd x_updated =
      motion_model_->composition_plus(x_predicted, W * innovation);

  // initalize lambda as initial value from config
  double lambda = lm_config.lambda_initial;

  // calculate P and R ldlt once
  Eigen::LDLT<Eigen::MatrixXd> P_predicted_LDLT = P_predicted.ldlt();
  Eigen::LDLT<Eigen::MatrixXd> R_LDLT = R.ldlt();

  double initial_cost = cost(x_updated, x_predicted, P_predicted_LDLT,
                             z_current, R_LDLT, input, *measurement_model);

  // TODO(LM): the iteration. Rough shape, in this order:
  //  1. before the loop: factorise P_predicted and R once (LDLT, for
  //  cost()),
  //     set lambda = lm_config.lambda_initial, evaluate the cost at
  //     x_updated
  //  2. per iterate: linearise H, z_predicted and the innovation at
  //  x_updated
  //     instead of x_predicted. The step is the (13.23) update with that
  //     linearisation point, so it needs the term that corrects for
  //     x_updated != x_predicted. Where to look: Sarkka (13.23), the
  //     v_k line, and Alg. 7.9
  //  3. damp the step with lambda. Where to look: Sarkka's Levenberg-
  //     Marquardt discussion in Ch. 13 (IEKS), Nocedal & Wright Sec. 10.3.
  //     Work out what damping does to the Hessian and to the gain, there is
  //     a form that keeps your W, S structure
  //  4. accept if cost decreased (lambda down), else reject and re-solve
  //     from the same iterate (lambda up). Limits: lambda_min, lambda_max,
  //     max_rejections. Nocedal & Wright Alg. 4.1 for the gain ratio
  //  5. stop on step_tol / cost_relative_tol / max_iter, see the comments
  //  in the yaml

  Eigen::MatrixXd H_i, S_i, W_i;
  Eigen::VectorXd innovation_i;

  Eigen::VectorXd x_i_previous = x_updated;
  for (int i = 0; i < lm_config.max_iter; ++i) {
    H_i = measurement_model->H(x_i_previous, input);
    Eigen::MatrixXd R_i = measurement_model->R(x_i_previous, input);

    innovation_i =
        measurement_model->composition_minus(
            z_current, measurement_model->h(x_i_previous, input)) -
        H_i * motion_model_->composition_minus(x_predicted, x_i_previous);

    Eigen::MatrixXd PHt_i = P_predicted * H_i.transpose();
    S_i = H_i * PHt_i + R_i;
    W_i = S_i.ldlt().solve(PHt_i.transpose()).transpose();

    // m = m^- + K v, from the prediction, not from the previous iterate
    Eigen::VectorXd x_candidate =
        motion_model_->composition_plus(x_predicted, W_i * innovation_i);

    // TODO(LM): steps 3-5 go here. Until then, take every step
    x_i_previous = x_candidate;
  }
  x_updated = x_i_previous;

  // TODO(LM): H and W here must be the undamped ones evaluated at the final
  //  x_updated, otherwise P_updated is inflated by lambda.

  // Joseph form: (I - WH) P (I - WH)^T + W R W^T. Algebraically equal to the
  // textbook (I - WH) P, but always positive definite
  Eigen::MatrixXd I =
      Eigen::MatrixXd::Identity(P_predicted.rows(), P_predicted.rows());
  Eigen::MatrixXd IWH = I - W * H;
  Eigen::MatrixXd P_updated =
      IWH * P_predicted * IWH.transpose() + W * R * W.transpose();

  // TODO(consistency): this reports the EKF-style innovation and S at
  //  x_predicted. Once iterating, decide whether to keep that or report the
  //  converged point's, so the NIS plots compare like with like. Note the
  //  choice in the guide.
  FilterUpdate filter_update{x_updated, innovation, S, P_updated};
  return filter_update;
}
