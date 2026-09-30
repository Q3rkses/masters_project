cmake_minimum_required(VERSION 3.8)
project(kalman_filtering_and_smoothing)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(kalman_lib
  src/models/models.cpp
  src/models/linear_models.cpp
  src/models/nonlinear_models.cpp
  src/models/measurement_models.cpp
  src/filters/kalman_filter.cpp
  src/filters/extended_kalman_filter.cpp
  src/smoothers/rauch_tung_striebel_smoother.cpp
  src/smoothers/extended_rauch_tung_striebel_smoother.cpp
  src/simulation.cpp
  src/results.cpp
  src/trajectory.cpp
  src/scenarios/random_walk_scenario.cpp
  src/scenarios/strapdown_ins_2D_scenario.cpp
)

include_directories(lib)
target_include_directories(kalman_lib PUBLIC lib)
target_link_libraries(kalman_lib PUBLIC Eigen3::Eigen yaml-cpp)

add_executable(kalman_filtering_and_smoothing
  src/main.cpp
)
target_link_libraries(kalman_filtering_and_smoothing PRIVATE kalman_lib)
target_compile_definitions(kalman_filtering_and_smoothing PRIVATE
  PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
)
