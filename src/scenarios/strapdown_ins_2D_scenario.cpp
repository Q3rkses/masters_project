#include "scenarios/scenarios.hpp"

#include "filters/extended_kalman_filter.hpp"
#include "filters/unscented_kalman_filter.hpp"
#include "models/linear_models.hpp"
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
} //clang-format on

// what one sensor reported and how the filter saw it, one entry per fix,
// this is what the write_fix_csv of the measurement models take
struct FixLog {
  std::vector<int> steps;
  std::vector<Eigen::VectorXd> z;
  std::vector<Eigen::VectorXd> innovations;
  std::vector<Eigen::MatrixXd> S;

  void add(const int k, const Eigen::VectorXd &measurement,
           const FilterUpdate &update) {
    steps.push_back(k);
    z.push_back(measurement);
    innovations.push_back(update.innovation);
    S.push_back(update.S);
  }
};

struct ForwardPass {
  FilterResult result;
  FixLog gnss_fixes;
  FixLog magnetometer_fixes;
  FixLog dvl_fixes;
};

// Forward pass of any filter. Predict once per step, then each due sensor
// updates in turn, building on whatever the previous one this step already
// did. Innovations and S of every sensor go to its own fix log, filter.csv
// only gets the state.
ForwardPass run_forward_pass(BayesianFilter &filter,
                             const FilterConfig &filter_config,
                             const std::vector<Input> &inputs,
                             const Sensor &gnss, const Sensor &magnetometer,
                             const Sensor &dvl) {
  ForwardPass pass{FilterResult(filter_config), {}, {}, {}};
  Eigen::VectorXd x = filter_config.x_prior;
  Eigen::MatrixXd P = filter_config.P_prior;

  const int timesteps = inputs.size();
  for (int k = 0; k < timesteps; k++) {
    FilterPredict prediction = filter.predict(x, P, inputs[k]);
    x = prediction.x_predicted;
    P = prediction.P_predicted;

    if (gnss.due[k]) {
      FilterUpdate gnss_update =
          filter.update(x, gnss.measurements[k], P, inputs[k], gnss.model);
      pass.gnss_fixes.add(k, gnss.measurements[k], gnss_update);
      x = gnss_update.x_updated;
      P = gnss_update.P_updated;
    }

    if (magnetometer.due[k]) {
      FilterUpdate magnetometer_update = filter.update(
          x, magnetometer.measurements[k], P, inputs[k], magnetometer.model);
      pass.magnetometer_fixes.add(k, magnetometer.measurements[k],
                                  magnetometer_update);
      x = magnetometer_update.x_updated;
      P = magnetometer_update.P_updated;
    }

    if (dvl.due[k]) {
      FilterUpdate dvl_update =
          filter.update(x, dvl.measurements[k], P, inputs[k], dvl.model);
      pass.dvl_fixes.add(k, dvl.measurements[k], dvl_update);
      x = dvl_update.x_updated;
      P = dvl_update.P_updated;
    }

    // filter.csv gets the state after every sensor that fired this step,
    // innovation and S are left empty
    pass.result.add(prediction,
                    FilterUpdate{.x_updated = x, .P_updated = P});
  }
  return pass;
}

// Backward pass of any smoother over a recorded forward pass. The forward
// pass predicts step k with inputs[k], so the transition from step k to
// step k + 1 used inputs[k + 1]
SmootherResult run_backward_pass(BayesianSmoother &smoother,
                                 const SmootherConfig &smoother_config,
                                 const FilterResult &filter_result,
                                 const std::vector<Input> &inputs) {
  const std::vector<FilterPredict> &predictions = filter_result.predictions();
  const std::vector<FilterUpdate> &updates = filter_result.updates();

  const int timesteps = inputs.size();
  std::vector<SmootherUpdate> smoothed(timesteps);
  smoothed[timesteps - 1] = SmootherUpdate{updates[timesteps - 1].x_updated,
                                           updates[timesteps - 1].P_updated};
  for (int k = timesteps - 2; k >= 0; k--) {
    smoothed[k] = smoother.backward_recursion(updates[k], predictions[k + 1],
                                              smoothed[k + 1], inputs[k + 1]);
  }

  SmootherResult smoother_result(smoother_config);
  for (const SmootherUpdate &smoothed_step : smoothed) {
    smoother_result.add(smoothed_step);
  }
  return smoother_result;
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

  // 4. sensor schedule, drawn once so the EKF and the UKF see the same fixes
  const SensorSchedule schedule =
      build_sensor_schedule(config, rng, timesteps, ins_config.dt);
  const Sensor gnss{gnss_measurement_model, measurements, schedule.gnss};
  const Sensor magnetometer{magnetometer_measurement_model,
                            magnetometer_measurements, schedule.magnetometer};
  const Sensor dvl{dvl_measurement_model, dvl_measurements, schedule.dvl};

  // 5. EKF forward pass and ERTS backward pass
  ExtendedKalmanFilter ekf(filter_config);
  ForwardPass ekf_pass =
      run_forward_pass(ekf, filter_config, inputs, gnss, magnetometer, dvl);
  ERTSSmoother erts(smoother_config);
  SmootherResult erts_result =
      run_backward_pass(erts, smoother_config, ekf_pass.result, inputs);

  // 6. UKF forward pass and URTSS backward pass on the same data
  const TransformConfig transform_config{
      .alpha = config["ukf_alpha"].as<double>(),
      .beta = config["ukf_beta"].as<double>(),
      .kappa = config["ukf_kappa"].as<double>(),
      .dimension = static_cast<int>(x0.size())};
  UnscentedKalmanFilter ukf(filter_config, transform_config);
  ForwardPass ukf_pass =
      run_forward_pass(ukf, filter_config, inputs, gnss, magnetometer, dvl);
  URTSSmoother urts(smoother_config, transform_config);
  SmootherResult urts_result =
      run_backward_pass(urts, smoother_config, ukf_pass.result, inputs);

  // 7. write truth.csv, filter.csv, smoother.csv, ukf_filter.csv and
  // ukf_smoother.csv to output_dir. The fix files hold innovations and S,
  // which depend on the filter, so each filter gets its own, the UKF's
  // prefixed with ukf_
  std::filesystem::create_directories(output_dir);

  truth_result.to_csv((output_dir / "truth.csv").string());
  ekf_pass.result.to_csv((output_dir / "filter.csv").string());
  erts_result.to_csv((output_dir / "smoother.csv").string());
  ukf_pass.result.to_csv((output_dir / "ukf_filter.csv").string());
  urts_result.to_csv((output_dir / "ukf_smoother.csv").string());

  auto write_fix_files = [&](const ForwardPass &pass, const std::string &prefix) {
    gnss_measurement_model->write_fix_csv(
        output_dir / (prefix + "gnss_fixes.csv"), pass.gnss_fixes.steps,
        pass.gnss_fixes.z, pass.gnss_fixes.innovations, pass.gnss_fixes.S);
    magnetometer_measurement_model->write_fix_csv(
        output_dir / (prefix + "magnetometer_fixes.csv"),
        pass.magnetometer_fixes.steps, pass.magnetometer_fixes.z,
        pass.magnetometer_fixes.innovations, pass.magnetometer_fixes.S);
    dvl_measurement_model->write_fix_csv(
        output_dir / (prefix + "dvl_fixes.csv"), pass.dvl_fixes.steps,
        pass.dvl_fixes.z, pass.dvl_fixes.innovations, pass.dvl_fixes.S);
  };
  write_fix_files(ekf_pass, "");
  write_fix_files(ukf_pass, "ukf_");

  std::cout << "Wrote " << timesteps << " timesteps to " << output_dir << "\n";
}
