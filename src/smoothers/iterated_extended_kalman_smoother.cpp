#include "smoothers/iterated_extended_kalman_smoother.hpp"
#include "models/nonlinear_models.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>

IteratedExtendedKalmanSmoother::IteratedExtendedKalmanSmoother(
    const SmootherConfig &smoother_config,
    const LevenbergMarquardtConfig &lm_config)
    : motion_model_(smoother_config.motion_model), lm_config_(lm_config) {
  if (!motion_model_) {
    throw std::invalid_argument("IteratedExtendedKalmanSmoother: the smoother "
                                "config needs a motion model");
  }
  if (lm_config.lambda_factor <= 1.0) {
    throw std::invalid_argument(
        "IteratedExtendedKalmanSmoother: lambda_factor has to be above 1");
  }
};

double IteratedExtendedKalmanSmoother::cost(
    const Trajectory &x, const SmootherConfig &smoother_config,
    const std::vector<Input> &inputs, const std::vector<Sensor> &sensors) const {
  // The cost is the negative log posterior of the whole trajectory, a prior
  // term, a motion term for every transition and a measurement term for every
  // fix. Position has no process noise in the INS models, so Q is singular and
  // the motion term uses its pseudo-inverse, only the noise driven part of the
  // residual is penalized.
  Eigen::VectorXd prior_residual =
      motion_model_->composition_minus(x[0], smoother_config.x_prior);
  double total =
      0.5 * prior_residual.dot(smoother_config.P_prior.ldlt().solve(prior_residual));

  const int timesteps = inputs.size();
  for (int k = 0; k < timesteps; k++) {
    Eigen::VectorXd motion_residual = motion_model_->composition_minus(
        x[k + 1], motion_model_->f(x[k], inputs[k]));
    Eigen::MatrixXd Q = motion_model_->Q(x[k], inputs[k]);
    total += 0.5 * motion_residual.dot(
                       Q.completeOrthogonalDecomposition().solve(motion_residual));

    for (const Sensor &sensor : sensors) {
      if (!sensor.due[k]) {
        continue;
      }
      Eigen::VectorXd measurement_residual = sensor.model->composition_minus(
          sensor.measurements[k], sensor.model->h(x[k + 1], inputs[k]));
      Eigen::MatrixXd R = sensor.model->R(x[k + 1], inputs[k]);
      total += 0.5 * measurement_residual.dot(R.ldlt().solve(measurement_residual));
    }
  }
  return total;
}

IteratedExtendedKalmanSmoother::LinearSolution
IteratedExtendedKalmanSmoother::solve_linearized(
    const Trajectory &x_ref, const SmootherConfig &smoother_config,
    const std::vector<Input> &inputs, const std::vector<Sensor> &sensors,
    const std::vector<Eigen::MatrixXd> &damping_covariance,
    const double lambda) const {
  const int timesteps = inputs.size();
  const int nodes = timesteps + 1;
  const Eigen::MatrixXd I =
      Eigen::MatrixXd::Identity(smoother_config.P_prior.rows(),
                                smoother_config.P_prior.rows());

  // One Kalman update, written for the error coordinates. H maps the step
  // delta to the measurement, y is the innovation of the reference.
  auto kalman_update = [&](Eigen::VectorXd &delta, Eigen::MatrixXd &P,
                           const Eigen::VectorXd &y, const Eigen::MatrixXd &H,
                           const Eigen::MatrixXd &R) {
    Eigen::MatrixXd PHt = P * H.transpose();
    Eigen::MatrixXd S = H * PHt + R;
    Eigen::MatrixXd W = S.ldlt().solve(PHt.transpose()).transpose();
    delta += W * (y - H * delta);
    // Joseph form, always positive definite
    Eigen::MatrixXd IWH = I - W * H;
    P = IWH * P * IWH.transpose() + W * R * W.transpose();
  };

  // Levenberg-Marquardt damping, a measurement delta = 0 with covariance
  // D / lambda, so a large lambda keeps the step small
  auto damp = [&](Eigen::VectorXd &delta, Eigen::MatrixXd &P, const int node) {
    if (lambda > 0.0) {
      kalman_update(delta, P, Eigen::VectorXd::Zero(delta.size()), I,
                    damping_covariance[node] / lambda);
    }
  };

  // forward pass, the filtered and the predicted step of every node
  LinearSolution solution{std::vector<Eigen::VectorXd>(nodes),
                          std::vector<Eigen::MatrixXd>(nodes)};
  std::vector<Eigen::VectorXd> delta_predicted(nodes);
  std::vector<Eigen::MatrixXd> P_predicted(nodes);

  solution.delta[0] =
      motion_model_->composition_minus(smoother_config.x_prior, x_ref[0]);
  solution.P[0] = smoother_config.P_prior;
  damp(solution.delta[0], solution.P[0], 0);

  for (int k = 0; k < timesteps; k++) {
    const int node = k + 1;
    // the reference is not an exact solution of the motion model, c is how far
    // off it is
    Eigen::MatrixXd F = motion_model_->F(x_ref[k], inputs[k]);
    Eigen::VectorXd c = motion_model_->composition_minus(
        motion_model_->f(x_ref[k], inputs[k]), x_ref[node]);

    Eigen::VectorXd delta = c + F * solution.delta[k];
    Eigen::MatrixXd P = F * solution.P[k] * F.transpose() +
                        motion_model_->Q(x_ref[k], inputs[k]);
    delta_predicted[node] = delta;
    P_predicted[node] = P;

    for (const Sensor &sensor : sensors) {
      if (!sensor.due[k]) {
        continue;
      }
      Eigen::VectorXd y = sensor.model->composition_minus(
          sensor.measurements[k], sensor.model->h(x_ref[node], inputs[k]));
      kalman_update(delta, P, y, sensor.model->H(x_ref[node], inputs[k]),
                    sensor.model->R(x_ref[node], inputs[k]));
    }
    damp(delta, P, node);
    solution.delta[node] = delta;
    solution.P[node] = P;
  }

  // backward pass, an RTS smoother in the error coordinates. The filtered step
  // and covariance are overwritten with the smoothed ones.
  for (int node = nodes - 2; node >= 0; node--) {
    Eigen::MatrixXd F = motion_model_->F(x_ref[node], inputs[node]);
    Eigen::MatrixXd FP = F * solution.P[node];
    Eigen::MatrixXd W = P_predicted[node + 1].ldlt().solve(FP).transpose();

    solution.delta[node] +=
        W * (solution.delta[node + 1] - delta_predicted[node + 1]);
    solution.P[node] +=
        W * (solution.P[node + 1] - P_predicted[node + 1]) * W.transpose();
  }
  return solution;
}

SmootherResult IteratedExtendedKalmanSmoother::smooth(
    const SmootherConfig &smoother_config, const std::vector<Input> &inputs,
    const std::vector<Sensor> &sensors, const FilterResult &initial_guess) {
  const std::vector<FilterUpdate> &updates = initial_guess.updates();
  const int timesteps = inputs.size();
  const int nodes = timesteps + 1;

  // node 0 is the prior, node k + 1 is step k. The damping of a node is scaled
  // by the filter's covariance there.
  Trajectory x(nodes);
  std::vector<Eigen::MatrixXd> damping_covariance(nodes);
  x[0] = smoother_config.x_prior;
  damping_covariance[0] = smoother_config.P_prior;
  for (int k = 0; k < timesteps; k++) {
    x[k + 1] = updates[k].x_updated;
    damping_covariance[k + 1] = updates[k].P_updated;
  }

  double lambda = lm_config_.lambda_initial;
  int rejections = 0;
  double current_cost = cost(x, smoother_config, inputs, sensors);

  for (int i = 0; i < lm_config_.max_iter; ++i) {
    LinearSolution step = solve_linearized(x, smoother_config, inputs, sensors,
                                           damping_covariance, lambda);

    Trajectory candidate(nodes);
    double step_norm_squared = 0.0;
    for (int node = 0; node < nodes; node++) {
      candidate[node] = motion_model_->composition_plus(x[node], step.delta[node]);
      step_norm_squared += step.delta[node].dot(
          damping_covariance[node].ldlt().solve(step.delta[node]));
    }
    // the step in the filter's covariance norm, averaged over the nodes
    double step_norm = std::sqrt(step_norm_squared / nodes);
    double candidate_cost = cost(candidate, smoother_config, inputs, sensors);

    // accept step if cost went down, and trust the linear model more
    if (candidate_cost < current_cost) {
      double relative_decrease = (current_cost - candidate_cost) / current_cost;
      x = candidate;
      current_cost = candidate_cost;
      lambda = std::max(lambda / lm_config_.lambda_factor, lm_config_.lambda_min);
      rejections = 0;

      if (step_norm < lm_config_.step_tolerance ||
          relative_decrease < lm_config_.cost_relative_tolerance) {
        break;
      }
    }
    // reject step if cost went up, stay put and trust the linear model less
    else {
      // a step this small that does not help is the minimum, not a bad step
      if (step_norm < lm_config_.step_tolerance) {
        break;
      }
      lambda *= lm_config_.lambda_factor;
      rejections++;
      if (lambda > lm_config_.lambda_max ||
          rejections >= lm_config_.max_rejections) {
        break;
      }
    }
  }

  // The covariance is the undamped one at the final trajectory, otherwise it
  // would be inflated by lambda.
  LinearSolution final_solution = solve_linearized(
      x, smoother_config, inputs, sensors, damping_covariance, 0.0);

  SmootherResult smoother_result(smoother_config);
  for (int k = 0; k < timesteps; k++) {
    smoother_result.add(SmootherUpdate{x[k + 1], final_solution.P[k + 1]});
  }
  return smoother_result;
}
