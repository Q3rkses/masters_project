#include "scenarios/scenarios.hpp"

#include "filters/extended_kalman_filter.hpp"
#include "models/linear_models.hpp"
#include "models/nonlinear_models.hpp"
#include "results.hpp"
#include "simulation.hpp"
#include "smoothers/extended_rauch_tung_striebel_smoother.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

Eigen::VectorXd read_vector(const YAML::Node &node) {
  int size = node.size();
  Eigen::VectorXd output_vector = Eigen::VectorXd::Zero(size);

  for (int i = 0; i < size; i++) {
    output_vector[i] = node[i].as<double>();
  }

  return output_vector;
}
// clang-format off
INS2DConfig read_ins_config(const YAML::Node &config, ) {
  INS2DConfig ins{};
  ins.dt = config["dt"].as<double>();
  ins.gyro_time_constant = config["gyro_time_constant"].as<double>();
  ins.accelerometer_time_constant = read_vector(config["accelerometer_time_constant"]);
  ins.gyro_noise_variance = config["gyro_noise_variance"].as<double>();
  ins.accelerometer_noise_variance = read_vector(config["accelerometer_noise_variance"]);
  ins.gyro_white_noise_intensity = config["gyro_white_noise_intensity"].as<double>();
  ins.accelerometer_white_noise_intensity = read_vector(config["accelerometer_white_noise_intensity"]);
  // unused by Q for now, but keep them defined
  ins.gyro_random_walk_variance = 0.0;
  ins.accelerometer_random_walk_variance = Eigen::Vector2d::Zero();
  return ins;
} //clang-format on

void run_strapdown_ins_2d(const YAML::Node &config,
                          const std::filesystem::path &output_dir) {
  std::mt19937_64 rng(config["seed"].as<std::uint64_t>());
  const int timesteps = config["timesteps"].as<int>();

  // The motion model we are utilizing is a 2d strapdown INS model
  // x_k = f(x_{k-1}, u_k) + w_k, w_k ~ N(0,Q)
  // 
  // The measurement model can be kept linear, and depending on H
  // we can determine what quantities we observe, e.g with GNSS
  // H will be 2x8 matrix [I_(2x2), 0_(2x6)]
  //
  // z_k = Hx_k + v_k, v_k ~ N(0, R)

  // 1. models
  auto motion_model = std::make_shared<StrapdownINS2D>(read_ins_config(config));

  // GNSS model
  Eigen::MatrixXd gnss_H = Eigen::MatrixXd::Zero(2, 8);
  gnss_H.block<2, 2>(0, 0).setIdentity(); // position only
  const double gnss_r = config["gnss_noise_variance"].as<double>();
  ModelConfig gnss_measurement_config{.H = gnss_H,
                                 .R = gnss_r * Eigen::MatrixXd::Identity(2, 2)};
  auto gnss_measurement_model =
      std::make_shared<LinearMeasurementModel>(gnss_measurement_config);

  // GNSS dual antenna (gnss + heading) model

  Eigen::MatrixXd gnss_and_heading_H = Eigen::MatrixXd::Zero(3, 8);
  gnss_and_heading_H.block<3, 3>(0, 0).setIdentity(); // position and heading
  Eigen::VectorXd gnss_and_heading_r = read_vector(config["gnss_and_heading_noise_variance"]);
  ModelConfig gnss_and_heading_measurement_config{.H = gnss_and_heading_H,
                                 .R = gnss_and_heading_r * Eigen::MatrixXd::Identity(gnss_and_heading_r.size(),gnss_and_heading_r.size())};
  auto gnss_and_heading_measurement_model =
      std::make_shared<LinearMeasurementModel>(gnss_and_heading_measurement_config);

  // TODO: add more measurement models such as DVL
  
  // 2. prior and configs
  INS2DConfig ins_config = read_ins_config(config);

  const Eigen::VectorXd x0 = read_vector(config["x0"]);
  const Eigen::MatrixXd p0 =
      Eigen::MatrixXd(read_vector(config["p0_diagonal"]).asDiagonal());
  FilterConfig filter_config{.x_prior = x0,
                             .P_prior = p0,
                             .motion_model = motion_model,
                             .measurement_model = gnss_measurement_model};
  SmootherConfig smoother_config{
      .x_prior = x0, .P_prior = p0, .motion_model = motion_model};

  SimulationConfig accelerometer_gauss_markov_config{
    .rng = rng, .covariance = ins_config.accelerometer_noise_variance, .time_constant = accelerometer_time_constant, .mean = 0};
  SimulationConfig accelerometer_white_measurement_noise_config{
    .rng = rng, .covariance = ins_config.accelerometer_white_noise_intensity, .mean = 0};
  }
  SimulationConfig gyro_gauss_markov_config{
    .rng = rng, .covariance = ins_config.gyro_noise_variance, .time_constant = gyro_time_constant, .mean = 0};
  SimulationConfig gyro_white_measurement_noise_config{
    .rng = rng, .covariance = ins_config.gyro_white_noise_intensity, .mean = 0};
  }

  // 3. simulate truth and IMU, and keep the inputs for the filter and smoother
  TruthResult truth_result;
  std::vector<Input> inputs;
  std::vector<Eigen::VectorXd> measurements;
  measurements.reserve(timesteps);
  for (int k = 0; k < timesteps; k++) {
    // TODO: ideal a_x, a_y, omega for step k from config["trajectory"]
    // TODO: simulate the biases, reading = ideal + true bias + white noise
    // TODO: x_true = f(x_true, true input) + process noise (see point 1 above)
    // TODO: z = H * x_true + v, v ~ N(0, R)
    // truth_result.add(x_true, z, input_true); inputs.push_back(reading);
    // measurements.push_back(z);
  }

  // 4. EKF forward pass, same loop as the random walk with inputs[k]
  ExtendedKalmanFilter ekf(filter_config);
  FilterResult ekf_result(filter_config);
  Eigen::VectorXd x = x0;
  Eigen::MatrixXd P = p0;
  for (int k = 0; k < timesteps; k++) {
    FilterPredict prediction = ekf.predict(x, P, inputs[k]);
    FilterUpdate update = ekf.update(prediction.x_predicted, measurements[k],
                                        prediction.P_predicted, inputs[k]);
    ekf_result.add(prediction, update);
    x = update.x_updated;
    P = update.P_updated;
  }

  // 5. ERTS backward pass, same as the RTS one with inputs[k]
  ERTSSmoother smoother(smoother_config);
  // ...

  // 6. write truth.csv, filter.csv, smoother.csv to output_dir
  std::filesystem::create_directories(output_dir);

  truth_result.to_csv((output_dir / "truth.csv").string());
  filter_result.to_csv((output_dir / "filter.csv").string());
  smoother_result.to_csv((output_dir / "smoother.csv").string());

  std::cout << "Wrote " << timesteps << " timesteps to " << output_dir << "\n";
}
