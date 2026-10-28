#include "scenarios/scenarios.hpp"

#include <filesystem>
#include <yaml-cpp/yaml.h>

const std::string OUTPUT_DIR = "data";
const std::string CONFIG_PATH = "configs/strapdown_ins_2D.yaml";

int main(int argc, char *argv[]) {
  const std::filesystem::path project_root(PROJECT_ROOT_DIR);

  // current possible scenarios are found in project_root/configs/. A config
  // path and an output directory can be given as arguments, so a run does not
  // have to overwrite configs/ or data/
  const std::filesystem::path config_path =
      argc > 1 ? std::filesystem::path(argv[1]) : project_root / CONFIG_PATH;
  const std::filesystem::path output_dir =
      argc > 2 ? std::filesystem::path(argv[2]) : project_root / OUTPUT_DIR;
  const YAML::Node config = YAML::LoadFile(config_path.string());

  run_strapdown_ins_2d(config, output_dir);

  return 0;
}
