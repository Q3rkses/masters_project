#include "kalman_filter.hpp"
#include "models.hpp"
#include "rauch_tung_striebel_smoother.hpp"
#include "results.hpp"
#include "simulation.hpp"

#include <Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <random>
#include <vector>

int main() {
  const int timesteps = 1000;
  std::mt19937_64 rng(42);

  // the model we are utilizing is a 1-D random walk observed directly:
  //   x_k = x_{k-1} + w_k,  w_k ~ N(0, q)  ->  F = 1, Q = q
  //   z_k = x_k     + v_k,  v_k ~ N(0, r)  ->  H = 1, R = r

  const double q = 0.05; // process noise variance (random walk step size)
  const double r = 0.5;  // measurement noise variance
  const double x0 = 0.0; // prior mean
  const double p0 = 1.0; // prior variance

  ModelConfig model_config{
      .F = Eigen::MatrixXd::Identity(1, 1),
      .H = Eigen::MatrixXd::Identity(1, 1),
      .Q = Eigen::MatrixXd::Constant(1, 1, q),
      .R = Eigen::MatrixXd::Constant(1, 1, r),
  };

  MotionModel motion_model(model_config);
  MeasurementModel measurement_model(model_config);

  FilterConfig filter_config{
      .x_prior = Eigen::VectorXd::Constant(1, x0),
      .P_prior = Eigen::MatrixXd::Constant(1, 1, p0),
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
  SimulationConfig initial_config{.rng = rng, .variance = p0, .mean = x0};
  GaussianWhiteNoise initial_state(initial_config);
  const double x_true_initial = initial_state.simulate(1)(0);

  SimulationConfig walk_config{.rng = rng, .variance = q, .mean = 0.0};
  RandomWalk random_walk(walk_config);
  const Eigen::VectorXd walk = random_walk.simulate(timesteps);

  SimulationConfig noise_config{.rng = rng, .variance = r, .mean = 0.0};
  GaussianWhiteNoise measurement_noise(noise_config);
  const Eigen::VectorXd noise = measurement_noise.simulate(timesteps);

  TruthResult truth_result;
  std::vector<Eigen::VectorXd> measurements;
  measurements.reserve(timesteps);
  for (int k = 0; k < timesteps; k++) {
    Eigen::VectorXd x_true =
        Eigen::VectorXd::Constant(1, x_true_initial + walk(k));
    Eigen::VectorXd z =
        model_config.H * x_true + Eigen::VectorXd::Constant(1, noise(k));
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
