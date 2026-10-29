#include "estimation.hpp"

void SensorInnovationLog::add(const int k, const Eigen::VectorXd &measurement,
                              const FilterUpdate &update) {
  steps.push_back(k);
  z.push_back(measurement);
  innovations.push_back(update.innovation);
  S.push_back(update.S);
}

FilterRun run_filter(BayesianFilter &filter, const FilterConfig &filter_config,
                     const std::vector<Input> &inputs,
                     const std::vector<Sensor> &sensors) {
  FilterRun run{FilterResult(filter_config), {}};
  for (const Sensor &sensor : sensors) {
    run.sensor_logs.push_back(SensorInnovationLog{
        sensor.name, static_cast<int>(sensor.noise_covariance.rows()), {}, {}, {}, {}});
  }

  Eigen::VectorXd x = filter_config.x_prior;
  Eigen::MatrixXd P = filter_config.P_prior;

  const int timesteps = inputs.size();
  for (int k = 0; k < timesteps; k++) {
    FilterPredict prediction = filter.predict(x, P, inputs[k]);
    x = prediction.x_predicted;
    P = prediction.P_predicted;

    for (std::size_t i = 0; i < sensors.size(); i++) {
      const Sensor &sensor = sensors[i];
      if (!sensor.due[k]) {
        continue;
      }
      FilterUpdate update =
          filter.update(x, sensor.measurements[k], P, inputs[k], sensor.model);
      run.sensor_logs[i].add(k, sensor.measurements[k], update);
      x = update.x_updated;
      P = update.P_updated;
    }

    run.result.add(prediction, FilterUpdate{.x_updated = x, .P_updated = P});
  }
  return run;
}

SmootherResult run_smoother(BayesianSmoother &smoother,
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
