#include "kalman_filter.hpp"
#include "models.hpp"
#include "rauch_tung_striebel_smoother.hpp"
#include "results.hpp"
#include "simulation.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>

int main() {
  const int timesteps = 1000;
  std::mt19937_64 rng(42);

  SimulationConfig gwn_config{.rng = rng, .variance = 0.5, .mean = 0.0};
  GaussianWhiteNoise gwn(gwn_config);
  Eigen::VectorXd gwn_samples = gwn.simulate(timesteps);

  SimulationConfig rw_config{.rng = rng, .variance = 0.05, .mean = 0.0};
  RandomWalk random_walk(rw_config);
  Eigen::VectorXd rw_samples = random_walk.simulate(timesteps);

  std::filesystem::path output_dir =
      std::filesystem::path(PROJECT_ROOT_DIR) / "data";
  std::filesystem::create_directories(output_dir);
  std::filesystem::path output_file = output_dir / "1000timesteps.csv";

  std::ofstream out(output_file);
  out << "timestep,gwn,random_walk\n";
  for (int i = 0; i < timesteps; i++) {
    out << i << "," << gwn_samples(i) << "," << rw_samples(i) << "\n";
  }

  std::cout << "Wrote " << timesteps << " rows to " << output_file << "\n";

  return 0;
}
