#include "scenarios/scenarios.hpp"

#include <filesystem>
#include <yaml-cpp/yaml.h>

const std::string OUTPUT_DIR = "data";
const std::string CONFIG_PATH = "configs/strapdown_ins_2D.yaml";

int main() {
  const std::filesystem::path project_root(PROJECT_ROOT_DIR);

  // current possible scenarios are found in project_root/configs/
  const YAML::Node config =
      YAML::LoadFile((project_root / CONFIG_PATH).string());

  run_random_walk(config, project_root / OUTPUT_DIR);

  return 0;
}
