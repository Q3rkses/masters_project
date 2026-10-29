#include "scenarios/scenarios.hpp"

#include <algorithm>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

// The scenario a config asks for with its scenario key. Configs without the
// key are the 2D strapdown INS.
void run_scenario(const YAML::Node &config, const fs::path &output_dir) {
  const std::string scenario =
      config["scenario"].as<std::string>("strapdown_ins_2d");
  if (scenario == "strapdown_ins_2d") {
    run_strapdown_ins_2d(config, output_dir);
  } else if (scenario == "random_walk") {
    run_random_walk(config, output_dir);
  } else {
    throw std::runtime_error("unknown scenario: " + scenario);
  }
}

const std::string OUTPUT_DIR = "data";
const std::string CONFIG_PATH = "configs/strapdown_ins_2D.yaml";

// A case is a directory with a config.yaml in it, and its csv files are
// written to the data folder next to that config.
const std::string CASE_CONFIG = "config.yaml";
const std::string CASE_OUTPUT = "data";

// The directory itself if it is a case, otherwise every case found below it
// at any depth, in path order.
std::vector<fs::path> find_cases(const fs::path &root) {
  if (fs::exists(root / CASE_CONFIG)) {
    return {root};
  }

  std::vector<fs::path> cases;
  for (const fs::directory_entry &entry : fs::recursive_directory_iterator(root)) {
    if (entry.is_regular_file() && entry.path().filename() == CASE_CONFIG) {
      cases.push_back(entry.path().parent_path());
    }
  }
  std::sort(cases.begin(), cases.end());
  return cases;
}

// usage: kalman_filtering_and_smoothing [case_dir | directory_of_cases]
//
// With no argument, runs configs/strapdown_ins_2D.yaml and writes to data/.
// With a case directory, runs <case_dir>/config.yaml and writes to
// <case_dir>/data. With a directory that is not a case itself, runs every
// case below it, e.g. results/INS2D regenerates all of that scenario's data.
int main(int argc, char *argv[]) {
  if (argc == 1) {
    const fs::path project_root(PROJECT_ROOT_DIR);
    const YAML::Node config =
        YAML::LoadFile((project_root / CONFIG_PATH).string());
    run_scenario(config, project_root / OUTPUT_DIR);
    return 0;
  }

  if (argc != 2 || !fs::is_directory(argv[1])) {
    std::cerr << "usage: " << argv[0] << " [case_dir | directory_of_cases]\n";
    return 1;
  }

  const std::vector<fs::path> cases = find_cases(argv[1]);
  if (cases.empty()) {
    std::cerr << "no " << CASE_CONFIG << " found in or below " << argv[1]
              << "\n";
    return 1;
  }

  // one failing case should not stop the others from being regenerated
  int failures = 0;
  for (const fs::path &case_dir : cases) {
    std::cout << "== " << case_dir.string() << "\n";
    try {
      const YAML::Node config =
          YAML::LoadFile((case_dir / CASE_CONFIG).string());
      run_scenario(config, case_dir / CASE_OUTPUT);
    } catch (const std::exception &error) {
      std::cerr << "failed: " << error.what() << "\n";
      failures++;
    }
  }
  return failures == 0 ? 0 : 1;
}
