#include "scenarios/scenarios.hpp"

#include "filters/extended_kalman_filter.hpp"
#include "models/linear_models.hpp"
#include "models/measurement_models.hpp"
#include "models/nonlinear_models.hpp"
#include "results.hpp"
#include "simulation.hpp"
#include "smoothers/extended_rauch_tung_striebel_smoother.hpp"
#include "trajectory.hpp"
#include "utilities.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
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
// prefix picks which set of IMU noise keys to read: "" for the truth that
// actually gets simulated, "filter_" for what the EKF/ERTS believe, kept
// separate so the two can be mismatched on purpose to test an inconsistent
// filter. dt is never prefixed, it's a property of the simulation, not a
// belief.
INS2DConfig read_ins_config(const YAML::Node &config, const std::string &prefix = "") {
  INS2DConfig ins{};
  ins.dt = config["dt"].as<double>();
  ins.gyro_time_constant = config[prefix + "gyro_time_constant"].as<double>();
  ins.accelerometer_time_constant = read_vector(config[prefix + "accelerometer_time_constant"]);
  ins.gyro_noise_variance = config[prefix + "gyro_noise_variance"].as<double>();
  ins.accelerometer_noise_variance = read_vector(config[prefix + "accelerometer_noise_variance"]);
  ins.gyro_white_noise_intensity = config[prefix + "gyro_white_noise_intensity"].as<double>();
  ins.accelerometer_white_noise_intensity = read_vector(config[prefix + "accelerometer_white_noise_intensity"]);
  // unused by Q for now, but keep them defined
  ins.gyro_random_walk_variance = 0.0;
  ins.accelerometer_random_walk_variance = Eigen::Vector2d::Zero();
  return ins;
} //clang-format on

struct Trajectory {
  Path path;
  bool is_closed; // loop around the path instead of stopping at its end
};

// picks the trajectory named by the "trajectory" key and builds it from its
// own set of keys; all three shapes' keys are expected to be present in the
// config, whichever one is actually picked
Trajectory build_trajectory(const YAML::Node &config) {
  const std::string trajectory = config["trajectory"].as<std::string>();

  if (trajectory == "rounded_rectangle") {
    return {make_rounded_rectangle(config["rectangle_width"].as<double>(),
                                   config["rectangle_height"].as<double>(),
                                   config["rectangle_corner_radius"].as<double>()),
           /*is_closed=*/true};
  }
  if (trajectory == "circle") {
    return {make_circle(config["circle_radius"].as<double>()), /*is_closed=*/true};
  }
  if (trajectory == "straight_into_turn") {
    const double turn_sweep = config["turn_sweep_deg"].as<double>() * M_PI / 180.0;
    return {make_straight_into_turn(config["straight_length"].as<double>(),
                                    config["turn_radius"].as<double>(), turn_sweep),
           /*is_closed=*/false};
  }

  throw std::runtime_error("unknown trajectory: " + trajectory);
}

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
  //
  // Only the DVL measurement model is nonlinear as it will report
  // NAV frame through a rotation matrix.
  // measurements in the body frame which need to be transformed to the
  //
  //z_k = h(x_k) + v_k, v_k ~ N(0, R) 
  
  // 1. models
  // the filter's own belief about IMU noise seperate from ground truth model noise
  auto motion_model =
      std::make_shared<StrapdownINS2D>(read_ins_config(config, "filter_"));

  // GNSS model
  const double gnss_r = config["gnss_noise_variance"].as<double>();
  GNSSConfig gnss_config{.position_noise = gnss_r * Eigen::MatrixXd::Identity(2, 2)};
  auto gnss_measurement_model =
      std::make_shared<GNSSMeasurementModel>(gnss_config);

  // GNSS dual antenna (gnss + heading) model

  Eigen::MatrixXd gnss_and_heading_H = Eigen::MatrixXd::Zero(3, 8);
  gnss_and_heading_H.block<3, 3>(0, 0).setIdentity(); // position and heading
  Eigen::VectorXd gnss_and_heading_r = read_vector(config["gnss_and_heading_noise_variance"]);
  ModelConfig gnss_and_heading_measurement_config{
      .H = gnss_and_heading_H,
      .R = Eigen::MatrixXd(gnss_and_heading_r.asDiagonal())};
  auto gnss_and_heading_measurement_model =
      std::make_shared<LinearMeasurementModel>(gnss_and_heading_measurement_config);

  // Magnetometer model, reports heading directly
  MagnetometerConfig magnetometer_config{
      .heading_noise_variance =
          config["magnetometer_noise_variance"].as<double>()};
  auto magnetometer_measurement_model =
      std::make_shared<MagnetometerMeasurementModel>(magnetometer_config);

  // DVL model, reports body-frame velocity
  const Eigen::VectorXd dvl_r = read_vector(config["dvl_noise_variance"]);
  const Eigen::MatrixXd dvl_R = Eigen::MatrixXd(dvl_r.asDiagonal());
  DVLConfig dvl_config{.velocity_noise = dvl_R};
  auto dvl_measurement_model =
      std::make_shared<DVLMeasurementModel>(dvl_config);

  // 2. prior and configs
  INS2DConfig ins_config = read_ins_config(config);

  const Eigen::VectorXd x0 = read_vector(config["x0"]);
  const Eigen::MatrixXd p0 =
      Eigen::MatrixXd(read_vector(config["p0_diagonal"]).asDiagonal());
  FilterConfig filter_config{
      .x_prior = x0, .P_prior = p0, .motion_model = motion_model};
  SmootherConfig smoother_config{
      .x_prior = x0, .P_prior = p0, .motion_model = motion_model};

  SimulationConfig accelerometer_gauss_markov_config{
      .rng = rng,
      .covariance = Eigen::MatrixXd(ins_config.accelerometer_noise_variance.asDiagonal()),
      .mean = Eigen::VectorXd::Zero(2),
      .timeconstant = ins_config.accelerometer_time_constant,
  };
  SimulationConfig accelerometer_white_measurement_noise_config{
      .rng = rng,
      .covariance = Eigen::MatrixXd(ins_config.accelerometer_white_noise_intensity.asDiagonal()),
      .mean = Eigen::VectorXd::Zero(2),
  };
  SimulationConfig gyro_gauss_markov_config{
      .rng = rng,
      .covariance = Eigen::MatrixXd::Constant(1, 1, ins_config.gyro_noise_variance),
      .mean = Eigen::VectorXd::Zero(1),
      .timeconstant = Eigen::VectorXd::Constant(1, ins_config.gyro_time_constant),
  };
  SimulationConfig gyro_white_measurement_noise_config{
      .rng = rng,
      .covariance = Eigen::MatrixXd::Constant(1, 1, ins_config.gyro_white_noise_intensity),
      .mean = Eigen::VectorXd::Zero(1),
  };

  GaussMarkov gyro_gauss_markov{gyro_gauss_markov_config, ins_config.dt};
  GaussMarkov accelerometer_gauss_markov{accelerometer_gauss_markov_config, ins_config.dt};

  // 3. simulate truth and IMU, and keep the inputs for the filter and smoother
  const Trajectory trajectory = build_trajectory(config);
  const double path_speed = config["path_speed"].as<double>();

  GaussianWhiteNoise accelerometer_white_noise(
      accelerometer_white_measurement_noise_config);
  GaussianWhiteNoise gyro_white_noise(gyro_white_measurement_noise_config);
  SimulationConfig gnss_noise_config{.rng = rng,
                                     .covariance = gnss_config.position_noise,
                                     .mean = Eigen::VectorXd::Zero(2)};
  GaussianWhiteNoise gnss_noise(gnss_noise_config);
  SimulationConfig magnetometer_noise_config{
      .rng = rng,
      .covariance = Eigen::MatrixXd::Constant(
          1, 1, config["magnetometer_noise_variance"].as<double>()),
      .mean = Eigen::VectorXd::Zero(1)};
  GaussianWhiteNoise magnetometer_noise(magnetometer_noise_config);
  SimulationConfig dvl_noise_config{
      .rng = rng, .covariance = dvl_R, .mean = Eigen::VectorXd::Zero(2)};
  GaussianWhiteNoise dvl_noise(dvl_noise_config);

  // each of these is simulated for the whole run up front, one entry per k
  const std::vector<Eigen::VectorXd> accelerometer_bias_series =
      accelerometer_gauss_markov.simulate(timesteps);
  const std::vector<Eigen::VectorXd> gyro_bias_series =
      gyro_gauss_markov.simulate(timesteps);
  const std::vector<Eigen::VectorXd> accelerometer_white_series =
      accelerometer_white_noise.simulate(timesteps);
  const std::vector<Eigen::VectorXd> gyro_white_series =
      gyro_white_noise.simulate(timesteps);
  const std::vector<Eigen::VectorXd> gnss_noise_series =
      gnss_noise.simulate(timesteps);
  const std::vector<Eigen::VectorXd> magnetometer_noise_series =
      magnetometer_noise.simulate(timesteps);
  const std::vector<Eigen::VectorXd> dvl_noise_series =
      dvl_noise.simulate(timesteps);

  TruthResult truth_result;
  std::vector<Input> inputs;
  std::vector<Eigen::VectorXd> measurements;
  std::vector<Eigen::VectorXd> magnetometer_measurements;
  std::vector<Eigen::VectorXd> dvl_measurements;
  inputs.reserve(timesteps);
  measurements.reserve(timesteps);
  magnetometer_measurements.reserve(timesteps);
  dvl_measurements.reserve(timesteps);

  for (int k = 0; k < timesteps; k++) {
    const double t = k * ins_config.dt;
    const double arc_length = trajectory.is_closed
                                  ? std::fmod(path_speed * t, trajectory.path.length())
                                  : path_speed * t;
    const PathSample sample = trajectory.path.sample(arc_length);

    const double psi = sample.heading;
    const double omega_true = path_speed * sample.curvature;
    const Eigen::Vector2d tangent(std::cos(psi), std::sin(psi));
    const Eigen::Vector2d normal(-std::sin(psi), std::cos(psi));
    const Eigen::Vector2d velocity = path_speed * tangent;
    // constant speed -> the only acceleration is centripetal
    const Eigen::Vector2d acceleration_nav =
        path_speed * path_speed * sample.curvature * normal;
    // the IMU measures specific force in the body frame, not the nav frame
    const Eigen::Vector2d acceleration_body =
        rotation_matrix_z_2D(psi).transpose() * acceleration_nav;

    Eigen::VectorXd x_true(8);
    x_true << sample.position, psi, velocity, accelerometer_bias_series[k],
        gyro_bias_series[k](0);

    // reading = ideal + true bias + white noise
    const Eigen::Vector2d accelerometer_reading = acceleration_body +
        accelerometer_bias_series[k] + accelerometer_white_series[k];
    const double gyro_reading =
        omega_true + gyro_bias_series[k](0) + gyro_white_series[k](0);
    Eigen::VectorXd reading_vector(3);
    reading_vector << accelerometer_reading, gyro_reading;
    Input reading(reading_vector);

    const Eigen::VectorXd z =
        gnss_measurement_model->h(x_true, reading) + gnss_noise_series[k];
    const Eigen::VectorXd magnetometer_z =
        magnetometer_measurement_model->h(x_true, reading) +
        magnetometer_noise_series[k];
    const Eigen::VectorXd dvl_z =
        dvl_measurement_model->h(x_true, reading) + dvl_noise_series[k];

    truth_result.add(x_true, z);
    inputs.push_back(reading);
    measurements.push_back(z);
    magnetometer_measurements.push_back(magnetometer_z);
    dvl_measurements.push_back(dvl_z);
  }

  // EKF forward pass. Three sensors, three schedules, all uniform. Predict
  // once per step, then each due sensor updates in turn, building on
  // whatever the previous one this step already did. filter.csv stays
  // GNSS-shaped (innovation/S only mean something on a GNSS step);
  // magnetometer and DVL get their own fix files, see below.
  ExtendedKalmanFilter ekf(filter_config);
  FilterResult ekf_result(filter_config);
  Eigen::VectorXd x = x0;
  Eigen::MatrixXd P = p0;

  const double gnss_period_min = config["gnss_period_min"].as<double>();
  const double gnss_period_max = config["gnss_period_max"].as<double>();
  double next_gnss_time = sample_uniform(rng, gnss_period_min, gnss_period_max);
  std::vector<int> gnss_fix_steps;
  std::vector<Eigen::VectorXd> gnss_fix_z;
  std::vector<Eigen::VectorXd> gnss_innovations;
  std::vector<Eigen::MatrixXd> gnss_S;

  const double magnetometer_period_min =
      config["magnetometer_period_min"].as<double>();
  const double magnetometer_period_max =
      config["magnetometer_period_max"].as<double>();
  double next_magnetometer_time =
      sample_uniform(rng, magnetometer_period_min, magnetometer_period_max);
  std::vector<int> magnetometer_fix_steps;
  std::vector<Eigen::VectorXd> magnetometer_fix_z;
  std::vector<Eigen::VectorXd> magnetometer_innovations;
  std::vector<Eigen::MatrixXd> magnetometer_S;

  const double dvl_period_min = config["dvl_period_min"].as<double>();
  const double dvl_period_max = config["dvl_period_max"].as<double>();
  double next_dvl_time = sample_uniform(rng, dvl_period_min, dvl_period_max);
  std::vector<int> dvl_fix_steps;
  std::vector<Eigen::VectorXd> dvl_fix_z;
  std::vector<Eigen::VectorXd> dvl_innovations;
  std::vector<Eigen::MatrixXd> dvl_S;

  for (int k = 0; k < timesteps; k++) {
    const double t = k * ins_config.dt;
    FilterPredict prediction = ekf.predict(x, P, inputs[k]);
    x = prediction.x_predicted;
    P = prediction.P_predicted;

    FilterUpdate gnss_update;
    if (t >= next_gnss_time) {
      gnss_update = ekf.update(x, measurements[k], P, inputs[k],
                               gnss_measurement_model);
      gnss_fix_steps.push_back(k);
      gnss_fix_z.push_back(measurements[k]);
      gnss_innovations.push_back(gnss_update.innovation);
      gnss_S.push_back(gnss_update.S);
      next_gnss_time += sample_uniform(rng, gnss_period_min, gnss_period_max);
      x = gnss_update.x_updated;
      P = gnss_update.P_updated;
    } else {
      gnss_update = FilterUpdate{
          .x_updated = x,
          .innovation = Eigen::VectorXd::Zero(2),
          .S = Eigen::MatrixXd::Zero(2, 2),
          .P_updated = P,
      };
    }

    if (t >= next_magnetometer_time) {
      FilterUpdate magnetometer_update =
          ekf.update(x, magnetometer_measurements[k], P, inputs[k],
                     magnetometer_measurement_model);
      magnetometer_fix_steps.push_back(k);
      magnetometer_fix_z.push_back(magnetometer_measurements[k]);
      magnetometer_innovations.push_back(magnetometer_update.innovation);
      magnetometer_S.push_back(magnetometer_update.S);
      next_magnetometer_time +=
          sample_uniform(rng, magnetometer_period_min, magnetometer_period_max);
      x = magnetometer_update.x_updated;
      P = magnetometer_update.P_updated;
    }

    if (t >= next_dvl_time) {
      FilterUpdate dvl_update = ekf.update(x, dvl_measurements[k], P, inputs[k],
                                           dvl_measurement_model);
      dvl_fix_steps.push_back(k);
      dvl_fix_z.push_back(dvl_measurements[k]);
      dvl_innovations.push_back(dvl_update.innovation);
      dvl_S.push_back(dvl_update.S);
      next_dvl_time += sample_uniform(rng, dvl_period_min, dvl_period_max);
      x = dvl_update.x_updated;
      P = dvl_update.P_updated;
    }

    // filter.csv gets the state after every sensor that fired this step,
    // innovation/S stay GNSS-only
    gnss_update.x_updated = x;
    gnss_update.P_updated = P;
    ekf_result.add(prediction, gnss_update);
  }

  // 5. ERTS backward pass, same as the RTS one with inputs[k]
  ERTSSmoother smoother(smoother_config);
  const std::vector<FilterPredict> &predictions = ekf_result.predictions();
  const std::vector<FilterUpdate> &updates = ekf_result.updates();

  std::vector<SmootherUpdate> smoothed(timesteps);
  smoothed[timesteps - 1] = SmootherUpdate{updates[timesteps - 1].x_updated,
                                          updates[timesteps - 1].P_updated};
  for (int k = timesteps - 2; k >= 0; k--) {
    smoothed[k] = smoother.backward_recursion(updates[k], predictions[k + 1],
                                              smoothed[k + 1], inputs[k]);
  }

  SmootherResult ERTS_result(smoother_config);
  for (const SmootherUpdate &smoothed_step : smoothed) {
    ERTS_result.add(smoothed_step);
  }

  // 6. write truth.csv, filter.csv, smoother.csv to output_dir
  std::filesystem::create_directories(output_dir);

  truth_result.to_csv((output_dir / "truth.csv").string());
  ekf_result.to_csv((output_dir / "filter.csv").string());
  ERTS_result.to_csv((output_dir / "smoother.csv").string());

  gnss_measurement_model->write_fix_csv(output_dir / "gnss_fixes.csv",
                                        gnss_fix_steps, gnss_fix_z,
                                        gnss_innovations, gnss_S);
  magnetometer_measurement_model->write_fix_csv(
      output_dir / "magnetometer_fixes.csv", magnetometer_fix_steps,
      magnetometer_fix_z, magnetometer_innovations, magnetometer_S);
  dvl_measurement_model->write_fix_csv(output_dir / "dvl_fixes.csv",
                                       dvl_fix_steps, dvl_fix_z,
                                       dvl_innovations, dvl_S);

  std::cout << "Wrote " << timesteps << " timesteps to " << output_dir << "\n";
}
