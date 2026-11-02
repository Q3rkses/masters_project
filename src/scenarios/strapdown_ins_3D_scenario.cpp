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
#include "utilities/file_utilities.hpp"
#include "utilities/math_utilities.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace {

// clang-format off
// prefix picks which set of IMU noise keys to read: "" for the truth that
// actually gets simulated, "filter_" for what the EKF/ERTS are modeled with.
INS3DConfig read_ins_3d_config(const YAML::Node &config, const std::string &prefix = "") {
  INS3DConfig ins{};
  ins.dt = config["dt"].as<double>();
  // gravity points down in the navigation frame, which is NED
  ins.gravity = Eigen::Vector3d(0.0, 0.0, config["gravity"].as<double>(9.81));
  ins.gyro_time_constant = read_vector(config[prefix + "gyro_time_constant"]);
  ins.accelerometer_time_constant = read_vector(config[prefix + "accelerometer_time_constant"]);
  ins.gyro_noise_variance = read_vector(config[prefix + "gyro_noise_variance"]);
  ins.accelerometer_noise_variance = read_vector(config[prefix + "accelerometer_noise_variance"]);
  ins.gyro_white_noise_intensity = read_vector(config[prefix + "gyro_white_noise_intensity"]);
  ins.accelerometer_white_noise_intensity = read_vector(config[prefix + "accelerometer_white_noise_intensity"]);
  return ins;
}
// clang-format on

// The motion model is a 3D strapdown INS, x_k = f(x_{k-1}, u_k) + w_k with
// w_k ~ N(0, Q), where the noise lives on the 15D error state. GNSS and the
// altimeter observe the position linearly, the magnetometer reports the yaw
// of the attitude and the DVL reports velocity in the body frame, so those two
// are nonlinear, z_k = h(x_k) + v_k with v_k ~ N(0, R). The order of the
// sensors decides the order their noise and schedules are drawn in.
std::vector<Sensor> build_sensors(const YAML::Node &config) {
  std::vector<Sensor> sensors;

  const double gnss_r = config["gnss_noise_variance"].as<double>();
  const Eigen::MatrixXd gnss_R = gnss_r * Eigen::MatrixXd::Identity(3, 3);
  sensors.push_back(
      Sensor{"gnss",
             std::make_shared<GNSS3DMeasurementModel>(GNSS3DConfig{.position_noise = gnss_R}),
             gnss_R, {}, {}});

  // the magnetometer reports the yaw of the attitude
  const double heading_variance = config["magnetometer_noise_variance"].as<double>();
  sensors.push_back(
      Sensor{"magnetometer",
             std::make_shared<Magnetometer3DMeasurementModel>(
                 Magnetometer3DConfig{.heading_noise_variance = heading_variance}),
             Eigen::MatrixXd::Constant(1, 1, heading_variance), {}, {}});

  // the DVL reports body-frame velocity
  const Eigen::VectorXd dvl_r = read_vector(config["dvl_noise_variance"]);
  const Eigen::MatrixXd dvl_R = Eigen::MatrixXd(dvl_r.asDiagonal());
  sensors.push_back(
      Sensor{"dvl",
             std::make_shared<DVL3DMeasurementModel>(DVL3DConfig{.velocity_noise = dvl_R}),
             dvl_R, {}, {}});

  // the altimeter reports the vertical position
  const double altimeter_variance = config["altimeter_noise_variance"].as<double>();
  sensors.push_back(
      Sensor{"altimeter",
             std::make_shared<AltimeterMeasurementModel>(
                 AltimeterConfig{.z_noise_variance = altimeter_variance}),
             Eigen::MatrixXd::Constant(1, 1, altimeter_variance), {}, {}});

  return sensors;
}

// The true states and the IMU readings that drive the motion model.
struct SimulatedData {
  TruthResult truth_result;
  std::vector<Input> inputs;
};

// Follows the configured 3D path and simulates the IMU biases and noise and
// the sensor noise around it. The measurement of every sensor is stored in the
// sensor, for every step whether or not the sensor fires there. The order the
// random series are drawn in decides the data a seed gives: the IMU series
// first, then the noise of each sensor in order.
SimulatedData simulate_truth_and_imu(const YAML::Node &config,
                                     std::mt19937_64 &rng, const int timesteps,
                                     const INS3DConfig &ins_config,
                                     std::vector<Sensor> &sensors) {
  SimulationConfig accelerometer_gauss_markov_config{
      .rng = rng,
      .covariance = Eigen::MatrixXd(ins_config.accelerometer_noise_variance.asDiagonal()),
      .mean = Eigen::VectorXd::Zero(3),
      .timeconstant = ins_config.accelerometer_time_constant,
  };
  SimulationConfig accelerometer_white_measurement_noise_config{
      .rng = rng,
      .covariance = Eigen::MatrixXd(ins_config.accelerometer_white_noise_intensity.asDiagonal()),
      .mean = Eigen::VectorXd::Zero(3),
  };
  SimulationConfig gyro_gauss_markov_config{
      .rng = rng,
      .covariance = Eigen::MatrixXd(ins_config.gyro_noise_variance.asDiagonal()),
      .mean = Eigen::VectorXd::Zero(3),
      .timeconstant = ins_config.gyro_time_constant,
  };
  SimulationConfig gyro_white_measurement_noise_config{
      .rng = rng,
      .covariance = Eigen::MatrixXd(ins_config.gyro_white_noise_intensity.asDiagonal()),
      .mean = Eigen::VectorXd::Zero(3),
  };

  GaussMarkov gyro_gauss_markov{gyro_gauss_markov_config, ins_config.dt};
  GaussMarkov accelerometer_gauss_markov{accelerometer_gauss_markov_config, ins_config.dt};

  const Trajectory3D trajectory = build_trajectory_3d(config);

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
    const PathSample3D sample = trajectory.sample(k * ins_config.dt);

    const Eigen::Quaterniond &attitude = sample.attitude;

    // the IMU measures specific force in the body frame, not the nav frame
    const Eigen::Vector3d specific_force_body =
        attitude.toRotationMatrix().transpose() * (sample.acceleration - ins_config.gravity);

    Eigen::VectorXd x_true(16);
    x_true << sample.position, sample.velocity, quaternion_to_vector(attitude),
        accelerometer_bias_series[k], gyro_bias_series[k];

    // reading = ideal + true bias + white noise
    const Eigen::Vector3d accelerometer_reading = specific_force_body +
        accelerometer_bias_series[k] + accelerometer_white_series[k];
    const Eigen::Vector3d gyro_reading =
        sample.angular_rate + gyro_bias_series[k] + gyro_white_series[k];
    Eigen::VectorXd reading_vector(6);
    reading_vector << accelerometer_reading, gyro_reading;
    Input reading(reading_vector);

    for (std::size_t i = 0; i < sensors.size(); i++) {
      Eigen::VectorXd measurement = sensors[i].model->h(x_true, reading);
      sensors[i].measurements.push_back(sensors[i].model->composition_plus(
          measurement, sensor_noise_series[i][k]));
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
struct StrapdownINS3DScenario {
  FilterConfig filter_config;
  SmootherConfig smoother_config;
  SimulatedData simulated;
  std::vector<Sensor> sensors;
};

StrapdownINS3DScenario build_strapdown_ins_3d(const YAML::Node &config,
                                              std::mt19937_64 &rng,
                                              const int timesteps) {
  // the filter's own belief about the IMU noise is kept separate from the
  // noise the truth is simulated with
  auto motion_model =
      std::make_shared<StrapdownINS3D>(read_ins_3d_config(config, "filter_"));
  const INS3DConfig ins_config = read_ins_3d_config(config);

  // the state is 16D, the covariance is on the 15D error state
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

  return StrapdownINS3DScenario{
      FilterConfig{.x_prior = x0, .P_prior = p0, .motion_model = motion_model},
      SmootherConfig{.x_prior = x0, .P_prior = p0, .motion_model = motion_model},
      std::move(simulated), std::move(sensors)};
}

// Runs the EKF with the ERTS and the UKF with the URTSS over the scenario and
// writes their results, the IEKF too for configs that set run_iekf.
void run_strapdown_ins_3d_estimators(const YAML::Node &config,
                                     const StrapdownINS3DScenario &scenario,
                                     const std::filesystem::path &output_dir) {
  const FilterConfig &filter_config = scenario.filter_config;
  const SmootherConfig &smoother_config = scenario.smoother_config;
  const std::vector<Input> &inputs = scenario.simulated.inputs;

  ExtendedKalmanFilter ekf(filter_config);
  ERTSSmoother erts(smoother_config);
  EstimatorRun ekf_run = run_estimator(ekf, erts, filter_config,
                                       smoother_config, inputs, scenario.sensors);

  // the unscented transform works on the error state, so on the covariance
  const TransformConfig transform_config{
      .alpha = config["ukf_alpha"].as<double>(1.0),
      .beta = config["ukf_beta"].as<double>(2.0),
      .kappa = config["ukf_kappa"].as<double>(0.0),
      .dimension = static_cast<int>(filter_config.P_prior.rows())};
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

} // namespace

void run_strapdown_ins_3d(const YAML::Node &config,
                          const std::filesystem::path &output_dir) {
  std::mt19937_64 rng(config["seed"].as<std::uint64_t>());
  const int timesteps = config["timesteps"].as<int>();

  const StrapdownINS3DScenario scenario =
      build_strapdown_ins_3d(config, rng, timesteps);
  run_strapdown_ins_3d_estimators(config, scenario, output_dir);

  std::cout << "Wrote " << timesteps << " timesteps to " << output_dir << "\n";
}
