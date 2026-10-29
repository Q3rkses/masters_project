#include "scenarios/scenarios.hpp"

#include "estimation.hpp"
#include "filters/extended_kalman_filter.hpp"
#include "filters/iterated_extended_kalman_filter.hpp"
#include "filters/unscented_kalman_filter.hpp"
#include "models/measurement_models.hpp"
#include "models/nonlinear_models.hpp"
#include "results.hpp"
#include "sensors.hpp"
#include "simulation.hpp"
#include "smoothers/extended_rauch_tung_striebel_smoother.hpp"
#include "smoothers/unscented_rauch_tung_striebel_smoother.hpp"
#include "trajectory.hpp"
#include "utilities.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <utility>
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
// actually gets simulated, "filter_" for what the EKF/ERTS are modeled with.
INS2DConfig read_ins_config(const YAML::Node &config, const std::string &prefix = "") {
  INS2DConfig ins{};
  ins.dt = config["dt"].as<double>();
  ins.gyro_time_constant = config[prefix + "gyro_time_constant"].as<double>();
  ins.accelerometer_time_constant = read_vector(config[prefix + "accelerometer_time_constant"]);
  ins.gyro_noise_variance = config[prefix + "gyro_noise_variance"].as<double>();
  ins.accelerometer_noise_variance = read_vector(config[prefix + "accelerometer_noise_variance"]);
  ins.gyro_white_noise_intensity = config[prefix + "gyro_white_noise_intensity"].as<double>();
  ins.accelerometer_white_noise_intensity = read_vector(config[prefix + "accelerometer_white_noise_intensity"]);
  ins.gyro_random_walk_variance = 0.0;
  ins.accelerometer_random_walk_variance = Eigen::Vector2d::Zero();
  return ins;
}

// A config only has to list the Levenberg-Marquardt parameters it wants to
// change, the rest fall back to these defaults.
LevenbergMarquardtConfig read_lm_config(const YAML::Node &config) {
  return LevenbergMarquardtConfig{
      .lambda_initial = config["lambda_initial"].as<double>(1e-2),
      .lambda_min = config["lambda_min"].as<double>(1e-12),
      .lambda_max = config["lambda_max"].as<double>(1e12),
      .lambda_factor = config["lambda_factor"].as<double>(10.0),
      .max_iter = config["max_iter"].as<int>(50),
      .max_rejections = config["max_rejections"].as<int>(30),
      .step_tolerance = config["step_tolerance"].as<double>(1e-8),
      .cost_relative_tolerance = config["cost_relative_tolerance"].as<double>(1e-12),
  };
} //clang-format on

// The motion model is a 2D strapdown INS, x_k = f(x_{k-1}, u_k) + w_k with
// w_k ~ N(0, Q). GNSS and the magnetometer observe the state linearly,
// z_k = H x_k + v_k with v_k ~ N(0, R). The DVL reports velocity in the body
// frame, so it needs the rotation matrix and is the one nonlinear sensor,
// z_k = h(x_k) + v_k with v_k ~ N(0, R). The order of the sensors decides the
// order their noise and schedules are drawn in.
std::vector<Sensor> build_sensors(const YAML::Node &config) {
  std::vector<Sensor> sensors;

  const double gnss_r = config["gnss_noise_variance"].as<double>();
  const Eigen::MatrixXd gnss_R = gnss_r * Eigen::MatrixXd::Identity(2, 2);
  sensors.push_back(
      Sensor{"gnss",
             std::make_shared<GNSSMeasurementModel>(GNSSConfig{.position_noise = gnss_R}),
             gnss_R, {}, {}});

  // the magnetometer reports heading directly
  const double heading_variance = config["magnetometer_noise_variance"].as<double>();
  sensors.push_back(
      Sensor{"magnetometer",
             std::make_shared<MagnetometerMeasurementModel>(
                 MagnetometerConfig{.heading_noise_variance = heading_variance}),
             Eigen::MatrixXd::Constant(1, 1, heading_variance), {}, {}});

  // the DVL reports body-frame velocity
  const Eigen::VectorXd dvl_r = read_vector(config["dvl_noise_variance"]);
  const Eigen::MatrixXd dvl_R = Eigen::MatrixXd(dvl_r.asDiagonal());
  sensors.push_back(
      Sensor{"dvl",
             std::make_shared<DVLMeasurementModel>(DVLConfig{.velocity_noise = dvl_R}),
             dvl_R, {}, {}});

  return sensors;
}

// The true states and the IMU readings that drive the motion model.
struct SimulatedData {
  TruthResult truth_result;
  std::vector<Input> inputs;
};

// Moves along the configured path at constant speed and simulates the IMU
// biases and noise and the sensor noise around it. The measurement of every
// sensor is stored in the sensor, for every step whether or not the sensor
// fires there. The order the random series are drawn in decides the data a
// seed gives: the IMU series first, then the noise of each sensor in order.
SimulatedData simulate_truth_and_imu(const YAML::Node &config,
                                     std::mt19937_64 &rng, const int timesteps,
                                     const INS2DConfig &ins_config,
                                     std::vector<Sensor> &sensors) {
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

  const Trajectory trajectory = build_trajectory(config);
  const double path_speed = config["path_speed"].as<double>();

  GaussianWhiteNoise accelerometer_white_noise(
      accelerometer_white_measurement_noise_config);
  GaussianWhiteNoise gyro_white_noise(gyro_white_measurement_noise_config);
  // each of these is simulated for the whole run up front, one entry per k
  const std::vector<Eigen::VectorXd> accelerometer_bias_series =
      accelerometer_gauss_markov.simulate(timesteps);
  const std::vector<Eigen::VectorXd> gyro_bias_series =
      gyro_gauss_markov.simulate(timesteps);
  const std::vector<Eigen::VectorXd> accelerometer_white_series =
      accelerometer_white_noise.simulate(timesteps);
  const std::vector<Eigen::VectorXd> gyro_white_series =
      gyro_white_noise.simulate(timesteps);
  std::vector<std::vector<Eigen::VectorXd>> sensor_noise_series;
  for (const Sensor &sensor : sensors) {
    SimulationConfig sensor_noise_config{
        .rng = rng,
        .covariance = sensor.noise_covariance,
        .mean = Eigen::VectorXd::Zero(sensor.noise_covariance.rows())};
    GaussianWhiteNoise sensor_noise(sensor_noise_config);
    sensor_noise_series.push_back(sensor_noise.simulate(timesteps));
  }

  TruthResult truth_result;
  std::vector<Input> inputs;
  inputs.reserve(timesteps);
  for (Sensor &sensor : sensors) {
    sensor.measurements.clear();
    sensor.measurements.reserve(timesteps);
  }

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
    const Eigen::Vector2d acceleration_nav = path_speed * path_speed * sample.curvature * normal;
    // the IMU measures specific force in the body frame, not the nav frame
    const Eigen::Vector2d acceleration_body = rotation_matrix_z_2D(psi).transpose() * acceleration_nav;

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

    for (std::size_t i = 0; i < sensors.size(); i++) {
      sensors[i].measurements.push_back(sensors[i].model->h(x_true, reading) +
                                        sensor_noise_series[i][k]);
    }

    truth_result.add(x_true);
    inputs.push_back(reading);
  }

  return SimulatedData{std::move(truth_result), std::move(inputs)};
}

// A filter's run together with the smoother's run over it.
struct EstimatorRun {
  FilterRun filtered;
  SmootherResult smoothed;
};

EstimatorRun run_estimator(BayesianFilter &filter, BayesianSmoother &smoother,
                           const FilterConfig &filter_config,
                           const SmootherConfig &smoother_config,
                           const std::vector<Input> &inputs,
                           const std::vector<Sensor> &sensors) {
  FilterRun filtered = run_filter(filter, filter_config, inputs, sensors);
  SmootherResult smoothed =
      run_smoother(smoother, smoother_config, filtered.result, inputs);
  return EstimatorRun{std::move(filtered), std::move(smoothed)};
}

// Writes <prefix>filter.csv and <prefix>smoother.csv, and the fix file of
// each sensor. The fix files hold innovations and S, which depend on the
// filter, so each filter gets its own set. The EKF has no prefix, the others
// are prefixed ukf_ and iekf_.
void write_estimator_run(const std::filesystem::path &output_dir,
                         const std::string &prefix, const EstimatorRun &run) {
  run.filtered.result.to_csv((output_dir / (prefix + "filter.csv")).string());
  run.smoothed.to_csv((output_dir / (prefix + "smoother.csv")).string());

  for (const SensorInnovationLog &log : run.filtered.sensor_logs) {
    write_fix_csv(output_dir / (prefix + log.sensor + "_fixes.csv"),
                  log.dimension, log.steps, log.z, log.innovations, log.S);
  }
}

// Everything that is simulated: the models, the true path, the IMU readings
// and the measurements and schedule of every sensor.
struct StrapdownINS2DScenario {
  FilterConfig filter_config;
  SmootherConfig smoother_config;
  SimulatedData simulated;
  std::vector<Sensor> sensors;
};

StrapdownINS2DScenario build_strapdown_ins_2d(const YAML::Node &config,
                                              std::mt19937_64 &rng,
                                              const int timesteps) {
  // the filter's own belief about the IMU noise is kept separate from the
  // noise the truth is simulated with
  auto motion_model =
      std::make_shared<StrapdownINS2D>(read_ins_config(config, "filter_"));
  const INS2DConfig ins_config = read_ins_config(config);

  const Eigen::VectorXd x0 = read_vector(config["x0"]);
  const Eigen::MatrixXd p0 =
      Eigen::MatrixXd(read_vector(config["p0_diagonal"]).asDiagonal());

  std::vector<Sensor> sensors = build_sensors(config);
  SimulatedData simulated =
      simulate_truth_and_imu(config, rng, timesteps, ins_config, sensors);

  // the schedule is drawn once, so every filter sees the same fixes
  std::vector<std::string> names;
  for (const Sensor &sensor : sensors) {
    names.push_back(sensor.name);
  }
  const std::vector<std::vector<bool>> schedules =
      build_sensor_schedules(config, rng, names, timesteps, ins_config.dt);
  for (std::size_t i = 0; i < sensors.size(); i++) {
    sensors[i].due = schedules[i];
  }

  return StrapdownINS2DScenario{
      FilterConfig{.x_prior = x0, .P_prior = p0, .motion_model = motion_model},
      SmootherConfig{.x_prior = x0, .P_prior = p0, .motion_model = motion_model},
      std::move(simulated), std::move(sensors)};
}

// Runs the EKF with the ERTS and the UKF with the URTSS over the scenario and
// writes their results, the IEKF too for configs that set run_iekf.
void run_strapdown_ins_2d_estimators(const YAML::Node &config,
                                     const StrapdownINS2DScenario &scenario,
                                     const std::filesystem::path &output_dir) {
  const FilterConfig &filter_config = scenario.filter_config;
  const SmootherConfig &smoother_config = scenario.smoother_config;
  const std::vector<Input> &inputs = scenario.simulated.inputs;

  ExtendedKalmanFilter ekf(filter_config);
  ERTSSmoother erts(smoother_config);
  EstimatorRun ekf_run = run_estimator(ekf, erts, filter_config,
                                       smoother_config, inputs, scenario.sensors);

  const TransformConfig transform_config{
      .alpha = config["ukf_alpha"].as<double>(1.0),
      .beta = config["ukf_beta"].as<double>(2.0),
      .kappa = config["ukf_kappa"].as<double>(0.0),
      .dimension = static_cast<int>(filter_config.x_prior.size())};
  UnscentedKalmanFilter ukf(filter_config, transform_config);
  URTSSmoother urts(smoother_config, transform_config);
  EstimatorRun ukf_run = run_estimator(ukf, urts, filter_config,
                                       smoother_config, inputs, scenario.sensors);

  std::filesystem::create_directories(output_dir);
  scenario.simulated.truth_result.to_csv((output_dir / "truth.csv").string());
  write_estimator_run(output_dir, "", ekf_run);
  write_estimator_run(output_dir, "ukf_", ukf_run);

  // The IEKF's smoother is the plain ERTS over its output, only the filter
  // iterates.
  if (config["run_iekf"].as<bool>(false)) {
    const LevenbergMarquardtConfig lm_config = read_lm_config(config);
    IteratedExtendedKalmanFilter iekf(filter_config, lm_config);
    EstimatorRun iekf_run = run_estimator(iekf, erts, filter_config,
                                          smoother_config, inputs, scenario.sensors);
    write_estimator_run(output_dir, "iekf_", iekf_run);
  }
}

void run_strapdown_ins_2d(const YAML::Node &config,
                          const std::filesystem::path &output_dir) {
  std::mt19937_64 rng(config["seed"].as<std::uint64_t>());
  const int timesteps = config["timesteps"].as<int>();

  const StrapdownINS2DScenario scenario =
      build_strapdown_ins_2d(config, rng, timesteps);
  run_strapdown_ins_2d_estimators(config, scenario, output_dir);

  std::cout << "Wrote " << timesteps << " timesteps to " << output_dir << "\n";
}
