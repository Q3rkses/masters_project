#include "kalman_filter.hpp"
#include "models.hpp"
#include "rauch_tung_striebel_smoother.hpp"
#include "results.hpp"
#include "simulation.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <filesystem>
#include <iostream>
#include <random>
#include <vector>

int main() {
  const int timesteps = 1000;
  std::mt19937_64 rng(42);

  // the model we are utilizing is a 2-D random walk observed directly:
  //   x_k = x_{k-1} + w_k,  w_k ~ N(0, Q)  ->  F = I_2, Q = q * I_2
  //   z_k = x_k     + v_k,  v_k ~ N(0, R)  ->  H = I_2, R = r * I_2

  const int dim = 2;
  const double q = 0.05; // process noise variance (random walk step size)
  const double r = 0.5;  // measurement noise variance
  const Eigen::VectorXd gwn_mean = Eigen::VectorXd::Zero(dim); // gwn mean
  const Eigen::VectorXd rw_mean = Eigen::VectorXd::Zero(dim);  // rw mean
  const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(dim);       // prior mean
  const Eigen::MatrixXd p0 =
      Eigen::MatrixXd::Identity(dim, dim); // prior covariance

  ModelConfig model_config{
      .F = Eigen::MatrixXd::Identity(dim, dim),
      .H = Eigen::MatrixXd::Identity(dim, dim),
      .Q = q * Eigen::MatrixXd::Identity(dim, dim),
      .R = r * Eigen::MatrixXd::Identity(dim, dim),
  };

  MotionModel motion_model(model_config);
  MeasurementModel measurement_model(model_config);

  FilterConfig filter_config{
      .x_prior = x0,
      .P_prior = p0,
      .motion_model = motion_model,
      .measurement_model = measurement_model,
  };

  SmootherConfig smoother_config{
      .x_prior = filter_config.x_prior,
      .P_prior = filter_config.P_prior,
      .motion_model = motion_model,
  };

  // this was maybe a wierd design choice on my part, i could have done the
  // simulation stepping inside of a for loop where i step through and add the
  // results to the truth vector, measurement vector etc... then filter and
  // smooth in two other for loops but ehhhh look at it tomorrow.
  SimulationConfig initial_config{.rng = rng, .covariance = p0, .mean = x0};
  GaussianWhiteNoise initial_state(initial_config);
  const Eigen::VectorXd x_true_initial = initial_state.simulate(1).at(0);

  SimulationConfig walk_config{
      .rng = rng, .covariance = model_config.Q, .mean = gwn_mean};
  RandomWalk random_walk(walk_config);
  const std::vector<Eigen::VectorXd> walk = random_walk.simulate(timesteps);

  SimulationConfig noise_config{
      .rng = rng, .covariance = model_config.R, .mean = rw_mean};
  GaussianWhiteNoise measurement_noise(noise_config);
  const std::vector<Eigen::VectorXd> noise =
      measurement_noise.simulate(timesteps);

  TruthResult truth_result;
  std::vector<Eigen::VectorXd> measurements;
  measurements.reserve(timesteps);
  for (int k = 0; k < timesteps; k++) {
    Eigen::VectorXd x_true = x_true_initial + walk.at(k);
    Eigen::VectorXd z = model_config.H * x_true + noise.at(k);
    truth_result.add(x_true, z);
    measurements.push_back(z);
  }

  // Kalman filter forward pass
  KalmanFilter kalman_filter(filter_config);
  FilterResult filter_result(filter_config);

  Eigen::VectorXd x = filter_config.x_prior;
  Eigen::MatrixXd P = filter_config.P_prior;
  for (int k = 0; k < timesteps; k++) {
    FilterPredict prediction = kalman_filter.predict(x, P);
    FilterUpdate update = kalman_filter.update(
        prediction.x_predicted, measurements[k], prediction.P_predicted);
    filter_result.add(prediction, update);
    x = update.x_updated;
    P = update.P_updated;
  }

  // RTS Smoother backwards pass
  RTSSmoother rts_smoother(smoother_config);
  const std::vector<FilterPredict> &predictions = filter_result.predictions();
  const std::vector<FilterUpdate> &updates = filter_result.updates();

  std::vector<SmootherUpdate> smoothed(timesteps);
  smoothed[timesteps - 1] = SmootherUpdate{updates[timesteps - 1].x_updated,
                                           updates[timesteps - 1].P_updated};
  for (int k = timesteps - 2; k >= 0; k--) {
    smoothed[k] = rts_smoother.backward_recursion(
        updates[k], predictions[k + 1], smoothed[k + 1]);
  }

  SmootherResult smoother_result(smoother_config);
  for (const SmootherUpdate &smoothed_step : smoothed) {
    smoother_result.add(smoothed_step);
  }

  // Code to utilize the to_csv functionality and to store the data so that it
  // can later be analyzed by a python script
  std::filesystem::path output_dir =
      std::filesystem::path(PROJECT_ROOT_DIR) / "data";
  std::filesystem::create_directories(output_dir);

  truth_result.to_csv((output_dir / "truth.csv").string());
  filter_result.to_csv((output_dir / "filter.csv").string());
  smoother_result.to_csv((output_dir / "smoother.csv").string());

  std::cout << "Wrote " << timesteps << " timesteps to " << output_dir << "\n";

  return 0;
}
